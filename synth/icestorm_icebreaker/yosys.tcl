read_verilog -sv synth/build/rtl.sv2v.v
synth_ice40 -top matmul_top -json synth/icestorm_icebreaker/build/synth.json
