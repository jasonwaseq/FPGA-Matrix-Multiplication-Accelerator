// UART Top Wrapper for DVS Gesture Accelerator with Temporal Voxel Binning
// Receives DVS events via UART, processes through voxel-based classifier,
// sends gesture results back
//
// UART Protocol (115200 baud, 8N1):
//   RX: 5 bytes per event [X_HI, X_LO, Y_HI, Y_LO, POL]
//       X, Y are 9-bit coordinates (0-319), split into high/low bytes
//       X_HI[0] = X[8], X_LO = X[7:0]
//       Y_HI[0] = Y[8], Y_LO = Y[7:0]
//       POL[0] = polarity (1=ON, 0=OFF)
//   TX: 1 byte when gesture detected [0xA0 | gesture]
//       gesture: 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT
//   Echo test: Send 0xFF, receive 0x55
//   Status query: Send 0xFE, receive [0xB0 | current_bin]

module uart_gesture_top #(
    parameter CLK_FREQ     = 12_000_000,
    parameter BAUD_RATE    = 115200,
    parameter WINDOW_US    = 400,        // 400µs temporal window
    parameter NUM_BINS     = 8,          // 8 temporal bins (50µs each)
    parameter GRID_SIZE    = 16,         // 16x16 compressed grid
    parameter SENSOR_RES   = 320         // 320x320 DVS resolution
)(
    input  logic clk,
    
    // UART interface
    input  logic uart_rx,
    output logic uart_tx,
    
    // Debug LED (active low for iCEBreaker accent LEDs)
    output logic led_heartbeat,
    
    // Optional debug outputs
    output logic led_gesture_valid
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    // =========================================================================
    // Power-On Reset (holds reset for first 15+ clock cycles)
    // =========================================================================
    reg [4:0] por_cnt = 5'd0;
    wire      rst;
    
    always @(posedge clk) begin
        if (&por_cnt)
            por_cnt <= por_cnt;
        else
            por_cnt <= por_cnt + 1'b1;
    end
    assign rst = ~&por_cnt;  // Active high reset for first 31 clocks

    // =========================================================================
    // UART RX/TX Instances
    // =========================================================================
    logic [7:0] rx_data;
    logic rx_valid;
    
    logic [7:0] tx_data;
    logic tx_valid;
    logic tx_busy;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart_rx (
        .clk(clk),
        .rst(rst),
        .rx(uart_rx),
        .data(rx_data),
        .valid(rx_valid)
    );
    
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart_tx (
        .clk(clk),
        .rst(rst),
        .data(tx_data),
        .valid(tx_valid),
        .tx(uart_tx),
        .busy(tx_busy)
    );

    // =========================================================================
    // DVS Gesture Accelerator Instance
    // =========================================================================
    logic [1:0] gesture;
    logic gesture_valid;
    logic event_ready;
    logic ring_swap;
    logic [2:0] current_bin;
    
    // Event interface to accelerator
    logic event_valid;
    logic [8:0] event_x, event_y;
    logic event_polarity;

    dvs_gesture_accel #(
        .CLK_FREQ_HZ(CLK_FREQ),
        .WINDOW_US(WINDOW_US),
        .NUM_BINS(NUM_BINS),
        .GRID_SIZE(GRID_SIZE),
        .SENSOR_RES(SENSOR_RES),
        .COUNTER_BITS(4),
        .FIFO_DEPTH(8)
    ) u_accel (
        .clk(clk),
        .rst(rst),
        .event_valid(event_valid),
        .event_x(event_x),
        .event_y(event_y),
        .event_polarity(event_polarity),
        .event_ts(16'd0),
        .event_ready(event_ready),
        .gesture(gesture),
        .gesture_valid(gesture_valid),
        .ring_swap(ring_swap),
        .current_bin(current_bin)
    );

    // =========================================================================
    // Heartbeat LED (shows FPGA is running)
    // =========================================================================
    logic [23:0] heartbeat_cnt;
    always @(posedge clk) begin
        if (rst)
            heartbeat_cnt <= 24'd0;
        else
            heartbeat_cnt <= heartbeat_cnt + 1'b1;
    end
    assign led_heartbeat = ~heartbeat_cnt[22];  // Active low, ~3Hz blink

    // Gesture valid indicator (active low, brief pulse stretched for visibility)
    logic [19:0] gesture_led_cnt;
    always @(posedge clk) begin
        if (rst) begin
            gesture_led_cnt <= '0;
        end else if (gesture_valid) begin
            gesture_led_cnt <= {20{1'b1}};  // ~87ms at 12MHz
        end else if (gesture_led_cnt > 0) begin
            gesture_led_cnt <= gesture_led_cnt - 1'b1;
        end
    end
    assign led_gesture_valid = ~(gesture_led_cnt > 0);  // Active low

    // =========================================================================
    // TX State Machine
    // =========================================================================
    typedef enum logic [1:0] {
        TX_IDLE,
        TX_SEND_ECHO,
        TX_SEND_GESTURE,
        TX_SEND_STATUS
    } tx_state_t;
    
    tx_state_t tx_state;
    logic [1:0] pending_gesture;
    logic [2:0] pending_bin;

    // =========================================================================
    // RX Packet Assembly State Machine
    // =========================================================================
    // 5-byte packet: [X_HI, X_LO, Y_HI, Y_LO, POL]
    typedef enum logic [2:0] {
        PKT_X_HI,
        PKT_X_LO,
        PKT_Y_HI,
        PKT_Y_LO,
        PKT_POL
    } pkt_state_t;
    
    pkt_state_t pkt_state;
    logic [8:0] saved_x, saved_y;
    logic saved_pol;

    // =========================================================================
    // Main Control Logic
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            pkt_state <= PKT_X_HI;
            saved_x <= 9'd0;
            saved_y <= 9'd0;
            saved_pol <= 1'b0;
            event_valid <= 1'b0;
            event_x <= 9'd0;
            event_y <= 9'd0;
            event_polarity <= 1'b0;
            tx_state <= TX_IDLE;
            tx_data <= 8'd0;
            tx_valid <= 1'b0;
            pending_gesture <= 2'd0;
            pending_bin <= 3'd0;
        end else begin
            // Default: clear single-cycle signals
            event_valid <= 1'b0;
            tx_valid <= 1'b0;
            
            // Handle received bytes
            if (rx_valid) begin
                // Check for special commands first (when expecting first byte)
                if (pkt_state == PKT_X_HI) begin
                    if (rx_data == 8'hFF) begin
                        // Echo test command
                        if (tx_state == TX_IDLE) begin
                            tx_state <= TX_SEND_ECHO;
                        end
                    end else if (rx_data == 8'hFE) begin
                        // Status query command
                        if (tx_state == TX_IDLE) begin
                            pending_bin <= current_bin;
                            tx_state <= TX_SEND_STATUS;
                        end
                    end else begin
                        // Normal packet - save X high bit
                        saved_x[8] <= rx_data[0];
                        pkt_state <= PKT_X_LO;
                    end
                end else begin
                    // Continue packet assembly
                    case (pkt_state)
                        PKT_X_LO: begin
                            saved_x[7:0] <= rx_data;
                            pkt_state <= PKT_Y_HI;
                        end
                        PKT_Y_HI: begin
                            saved_y[8] <= rx_data[0];
                            pkt_state <= PKT_Y_LO;
                        end
                        PKT_Y_LO: begin
                            saved_y[7:0] <= rx_data;
                            pkt_state <= PKT_POL;
                        end
                        PKT_POL: begin
                            // Complete packet - send to accelerator if ready
                            if (event_ready) begin
                                event_x <= saved_x;
                                event_y <= saved_y;
                                event_polarity <= rx_data[0];
                                event_valid <= 1'b1;
                            end
                            // Note: if not ready, event is dropped (could add overflow counter)
                            pkt_state <= PKT_X_HI;
                        end
                        default: pkt_state <= PKT_X_HI;
                    endcase
                end
            end
            
            // Capture gesture when detected (queue for TX)
            if (gesture_valid && tx_state == TX_IDLE) begin
                pending_gesture <= gesture;
                tx_state <= TX_SEND_GESTURE;
            end
            
            // TX state machine
            case (tx_state)
                TX_IDLE: begin
                    // Wait for something to send
                end
                
                TX_SEND_ECHO: begin
                    if (!tx_busy) begin
                        tx_data <= 8'h55;
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                TX_SEND_GESTURE: begin
                    if (!tx_busy) begin
                        // Format: 0xAG where G = gesture (0-3)
                        tx_data <= {4'hA, 2'b00, pending_gesture};
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                TX_SEND_STATUS: begin
                    if (!tx_busy) begin
                        // Format: 0xBB where B = current bin (0-7)
                        tx_data <= {4'hB, 1'b0, pending_bin};
                        tx_valid <= 1'b1;
                        tx_state <= TX_IDLE;
                    end
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
endmodule
