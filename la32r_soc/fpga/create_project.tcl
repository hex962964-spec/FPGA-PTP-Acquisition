# Run with: vivado -mode batch -source create_project.tcl
# Optional first argument: a NEW project output directory.
set script_dir [file dirname [file normalize [info script]]]
set root [file dirname $script_dir]
set project_path [file join $script_dir project_configured]
if {[llength $argv] > 0} {set project_path [file normalize [lindex $argv 0]]}
if {[file exists $project_path]} {error "Output already exists: $project_path. Choose a new directory to preserve previous results."}
create_project Loongson_Soc_configured $project_path -part xc7a200tfbg484-2
proc add_rtl_tree {dir} {
    foreach p [glob -nocomplain -directory $dir *] {
        if {[file isdirectory $p]} {
            if {![string match PLL* [file tail $p]]} {add_rtl_tree $p}
        } elseif {[file extension $p] in {.v .sv .h .vh}} {add_files -norecurse $p}
    }
}
add_rtl_tree [file join $root rtl]
foreach ip [list [file join $root rtl/ip/PLL_2019_2/clk_pll.xci] [file join $root rtl/ip/PLL_2025_2/rgmii_rx_pll/rgmii_rx_pll.xci]] {import_files -norecurse $ip}
set ram_init [file normalize [file join $root sdk/axi_ram.mif]]
if {![file exists $ram_init]} {error "Missing software image: $ram_init. Build the selected SDK example first."}
add_files -norecurse $ram_init
set_property verilog_define [list "SRAM_INIT_FILE_PATH=\"$ram_init\""] [get_filesets sources_1]
set_property verilog_define [list "SRAM_INIT_FILE_PATH=\"$ram_init\""] [get_filesets sim_1]
set_property include_dirs [list [file join $root rtl]] [get_filesets sources_1]
set_property include_dirs [list [file join $root rtl]] [get_filesets sim_1]
add_files -fileset sim_1 -norecurse [glob -directory [file join $root sim] *.v]
add_files -fileset constrs_1 -norecurse [file join $root fpga/constraints/soc.xdc]
set_property top soc_top [get_filesets sources_1]
set_property top tb_top [get_filesets sim_1]
set_property xsim.simulate.log_all_signals false [get_filesets sim_1]
set_property xsim.simulate.runtime 10ms [get_filesets sim_1]
foreach ip [get_ips] {
    if {[get_property IS_LOCKED $ip]} {upgrade_ip $ip}
}
generate_target all [get_ips]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
report_ip_status -file [file join $project_path ip_status.rpt]
