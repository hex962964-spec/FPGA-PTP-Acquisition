set root [file dirname [file normalize [info script]]]
foreach rel {la32r_soc/fpga/project/Loongson_Soc.xpr PTP_Master/PTP_Master.xpr} {
    open_project [file join $root $rel]
    set checkpoints [get_files -quiet -all *.dcp]
    if {[llength $checkpoints] > 0} {remove_files $checkpoints}
    if {[string match *Loongson_Soc.xpr $rel]} {
        set ram [file normalize [file join $root la32r_soc/sdk/axi_ram.mif]]
        foreach fs [get_filesets] {
            if {[get_property FILESET_TYPE $fs] in {DesignSrcs SimulationSrcs}} {
                set_property verilog_define [list "SRAM_INIT_FILE_PATH=\"$ram\""] $fs
            }
        }
    }
    close_project
}
puts "Prepared the two copied projects. No synthesis or programming was run."
