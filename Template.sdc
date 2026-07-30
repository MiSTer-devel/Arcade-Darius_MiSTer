derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# ============================================================
# Multicycle SOLO sulle istanze chip cen-paced (schema BoogieWings:
# "NO multicycle globale"). Il resto di darius_audio_z80 (adapter DDR3,
# req/ack, ce_cnt, mixer, adaptor savestate) gira su clk PURO a 96MHz
# = single-cycle: la STA deve vederlo e il fitter deve chiuderlo.
# Budget = caso peggiore OSD clk_sel 8MHz (ce_div 12) -> 12/11.
# ============================================================
set_multicycle_path -setup -from [get_registers {*z80_audio*}] -to [get_registers {*z80_audio*}] 12
set_multicycle_path -hold  -from [get_registers {*z80_audio*}] -to [get_registers {*z80_audio*}] 11
set_multicycle_path -setup -from [get_registers {*z80_adpcm*}] -to [get_registers {*z80_adpcm*}] 12
set_multicycle_path -hold  -from [get_registers {*z80_adpcm*}] -to [get_registers {*z80_adpcm*}] 11
set_multicycle_path -setup -from [get_registers {*u_ym1*}] -to [get_registers {*u_ym1*}] 12
set_multicycle_path -hold  -from [get_registers {*u_ym1*}] -to [get_registers {*u_ym1*}] 11
set_multicycle_path -setup -from [get_registers {*u_ym2*}] -to [get_registers {*u_ym2*}] 12
set_multicycle_path -hold  -from [get_registers {*u_ym2*}] -to [get_registers {*u_ym2*}] 11
set_multicycle_path -setup -from [get_registers {*u_msm5205*}] -to [get_registers {*u_msm5205*}] 12
set_multicycle_path -hold  -from [get_registers {*u_msm5205*}] -to [get_registers {*u_msm5205*}] 11
