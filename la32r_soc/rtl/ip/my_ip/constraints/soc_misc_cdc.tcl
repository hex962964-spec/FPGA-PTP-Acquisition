# Intentional two-stage level synchronizers and reset-converter entries only.
# Tcl constraint-set entry, implementation-only/LATE. No clock-group cuts.
namespace eval soc_misc_cdc {
 variable audit {}
 proc require {x label} {if {[llength $x]==0} {error "SOC_MISC_CDC missing $label"};return $x}
 proc cell {name} {return [require [get_cells -quiet -hier -filter [format {NAME == "%s" && REF_NAME =~ FD*} $name]] $name]}
 proc period {cells} {
  set clocks [require [get_clocks -quiet -of_objects [get_pins -quiet -of_objects $cells -filter {REF_PIN_NAME == C}]] clocks]
  set p 1e9;foreach clock $clocks {set v [get_property PERIOD $clock];if {$v<$p} {set p $v}};return $p
 }
}
foreach {inst source stem} {u_confreg confreg_int_tmp confreg_int_r u_axi_fft_top fft_finish_tmp_r fft_finish_r} {
 set src [soc_misc_cdc::cell "$inst/${source}_reg"]
 set first [soc_misc_cdc::cell [format {%s/%s_reg[0]} $inst $stem]]
 set second [soc_misc_cdc::cell [format {%s/%s_reg[1]} $inst $stem]]
 set d [get_pins -quiet -of_objects $first -filter {REF_PIN_NAME == D}]
 set actual [get_cells -quiet -of_objects [all_fanin -flat -startpoints_only -to $d] -filter {REF_NAME =~ FD*}]
 if {[llength $actual]!=1 || [lindex $actual 0] ne $src} {error "Unexpected level-source topology $inst/$stem: $actual"}
 set budget [expr {min([soc_misc_cdc::period $src],[soc_misc_cdc::period $first])}]
 set_max_delay -datapath_only $budget -from $src -to $d
 lappend soc_misc_cdc::audit [list level_entry $inst $stem $budget]
 puts "SOC_MISC_LEVEL_ENTRY=$inst/$stem bits=1 max=${budget}ns"
}
set reset_count 0
foreach bridge {u_Axi_CDC u_axi_fft_top/u_Axi_CDC_fft} {
 foreach fifo {awFifo bFifo} {
  set inst "$bridge/$fifo/bufferCC_12"
  set first [soc_misc_cdc::cell "$inst/buffers_0_reg"]
  set second [soc_misc_cdc::cell "$inst/buffers_1_reg"]
  # These particular scalar BufferCC instances are reset converters, not data
  # buses: D0 is tied high, D1 is Q0, both FFs asynchronously clear to zero.
  set d0 [get_pins -quiet -of_objects $first -filter {REF_PIN_NAME == D}]
  set d1 [get_pins -quiet -of_objects $second -filter {REF_PIN_NAME == D}]
  set vcc [get_cells -quiet -of_objects [get_pins -quiet -of_objects [get_nets -quiet -segments -of_objects $d0] -filter {DIRECTION == OUT}] -filter {REF_NAME == VCC}]
  if {[llength $vcc]==0} {error "Reset converter D0 is not proven constant1: $inst"}
  set actual [get_cells -quiet -of_objects [all_fanin -flat -startpoints_only -to $d1] -filter {REF_NAME =~ FD*}]
  if {[llength $actual]!=1 || [lindex $actual 0] ne $first} {error "Reset converter is not a two-stage chain: $inst $actual"}
  foreach ff [list $first $second] {if {![get_property ASYNC_REG $ff]} {error "Reset converter lacks ASYNC_REG: $ff"}}
  set clr [soc_misc_cdc::require [get_pins -quiet -of_objects [list $first $second] -filter {REF_PIN_NAME == CLR}] "$inst CLR"]
  if {[llength $clr]!=2} {error "Unexpected reset converter CLR width $inst"}
  # Raw deassertion is allowed at the synchronizer's own asynchronous CLR.
  # Only these exact eight CLR pins are excepted. Stage0->stage1 D setup and
  # stage1 Q->all downstream asynchronous controls remain normally timed.
  set_false_path -to $clr
  incr reset_count [llength $clr]
  lappend soc_misc_cdc::audit [list reset_entry $inst [llength $clr]]
  puts "SOC_MISC_RESET_ENTRY=$inst CLR_PINS=[llength $clr] CONST1=1 TWO_STAGE=1"
 }
}
if {$reset_count!=8} {error "Expected exactly8 reset-converter CLR entry pins"}
puts "SOC_MISC_CDC_GROUPS=[llength $soc_misc_cdc::audit] RESET_CLR_PINS=$reset_count"
