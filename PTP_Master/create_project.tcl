# Run with: vivado -mode batch -source create_project.tcl
set root [file dirname [file normalize [info script]]]
set out [file join $root project_configured]
if {[llength $argv] > 0} {set out [file normalize [lindex $argv 0]]}
if {[file exists $out]} {error "Output already exists: $out. Choose a new directory."}
create_project PTP_Master_configured $out -part xc7a35tfgg484-2
proc add_rtl_tree {dir} {
    foreach p [glob -nocomplain -directory $dir *] {
        if {[file isdirectory $p]} {add_rtl_tree $p} elseif {[file extension $p] in {.v .sv .h .vh}} {add_files -norecurse $p}
    }
}
add_rtl_tree [file join $root PTP_Master.srcs/sources_1/new]
foreach ip [glob -directory [file join $root PTP_Master.srcs/sources_1/ip] */*.xci] {import_files -norecurse $ip}
add_files -fileset constrs_1 -norecurse [file join $root PTP_Master.srcs/constrs_1/new/ptp_master.xdc]
set_property top observer_top [get_filesets sources_1]
set cdc [file join $root PTP_Master.srcs/constrs_1/new/ptp_cdc.xdc]
add_files -fileset constrs_1 -norecurse $cdc
set_property used_in_synthesis false [get_files $cdc]
set_property PROCESSING_ORDER LATE [get_files $cdc]
set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

foreach ip [get_ips] {if {[get_property IS_LOCKED $ip]} {upgrade_ip $ip}}
generate_target all [get_ips]
update_compile_order -fileset sources_1
report_ip_status -file [file join $out ip_status.rpt]
