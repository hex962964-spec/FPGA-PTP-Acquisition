# Scoped CDC contracts for stable req/ack bundles and registered Gray pointers.
# Implementation-only: clocks and rebuilt synthesized hierarchy must exist.
# No clock-group cuts, no blanket data/reset false paths. Downstream same-domain
# logic, synchronizer stage1->stage2, FIFO read addresses and payload pipelines
# retain normal setup/hold timing.
namespace eval ptp_cdc {
    variable audit {}
    variable clock_register_cache {}
    proc require {v label} {if {[llength $v]==0} {error "PTP CDC no objects: $label"};return $v}
    proc instances {name} {
        # top_phy_axis_mac_rx/tx share a textual prefix but are not the wrapper.
        if {$name eq "top_phy"} {
            return [get_cells -quiet -hier -filter {NAME == "u_top_phy" && IS_PRIMITIVE == 0}]
        }
        return [get_cells -quiet -hier -filter [format {REF_NAME =~ "%s*" || ORIG_REF_NAME == "%s"} $name $name]]
    }
    proc bank {inst stems {required 1}} {
        set out {}
        foreach stem $stems {
            set out [concat $out [get_cells -quiet -hier -filter [format {NAME =~ "%s/%s_reg*" && IS_PRIMITIVE == 1 && REF_NAME =~ FD*} $inst $stem]]]
        }
        set out [lsort -unique $out]
        if {$required} {require $out "$inst/$stems"}
        return $out
    }
    proc bit0 {inst stem} {
        return [require [get_cells -quiet -hier -filter [format {NAME == "%s/%s_reg[0]"} $inst $stem]] "$inst/$stem bit0"]
    }
    proc period {cells label} {
        set clocks [require [get_clocks -quiet -of_objects [get_pins -quiet -of_objects $cells -filter {REF_PIN_NAME == C}]] "$label clocks"]
        set p 1.0e9
        foreach c $clocks {set n [get_property PERIOD $c];if {$n<=0} {error "Invalid clock $c"};if {$n<$p} {set p $n}}
        return $p
    }
    proc sync {cells label} {
        variable audit
        set d [require [get_pins -quiet -of_objects $cells -filter {REF_PIN_NAME == D}] "$label D"]
        set p [period $cells $label]
        # Entry FF may sample an asynchronous control, but keep physical routing
        # bounded to one destination period. Stage2 is deliberately not excepted.
        set start_pins [all_fanin -flat -startpoints_only -to $d]
        set source [require [get_cells -quiet -of_objects $start_pins -filter {IS_PRIMITIVE == 1 && REF_NAME =~ FD*}] "$label source FF"]
        set_max_delay -datapath_only $p -from $source -to $d
        lappend audit [list control_entry $label [llength $d] $p]
        puts "PTP_CDC control $label pins=[llength $d] max=${p}ns"
    }
    proc bundle {src dst budget label} {
        variable audit
        require $src "$label source";require $dst "$label destination"
        # Synthesis may share a Gray MSB with the binary pointer, or share
        # identical shadow/last_sent bits. Resolve real D fan-in FFs in the
        # source clock(s), so canonical-name aliases cannot escape the budget.
        variable clock_register_cache
        set src_clocks [get_clocks -quiet -of_objects [get_pins -quiet -of_objects $src -filter {REF_PIN_NAME == C}]]
        set candidates [get_cells -quiet -of_objects [all_fanin -flat -startpoints_only -to [get_pins -quiet -of_objects $dst -filter {REF_PIN_NAME == D}]] -filter {IS_PRIMITIVE == 1 && REF_NAME =~ FD*}]
        foreach clock $src_clocks {
            if {![dict exists $clock_register_cache $clock]} {
                set regs {}
                foreach cell [all_registers -clock $clock -cells] {dict set regs $cell 1}
                dict set clock_register_cache $clock $regs
            }
            set regs [dict get $clock_register_cache $clock]
            foreach cell $candidates {if {[dict exists $regs $cell]} {lappend src $cell}}
        }
        set src [lsort -unique $src]
        set_max_delay -datapath_only $budget -from $src -to $dst
        set_bus_skew $budget -from $src -to $dst
        lappend audit [list stable_bundle $label [llength $src] [llength $dst] $budget]
        puts "PTP_CDC bundle $label from=[llength $src] to=[llength $dst] max/skew=${budget}ns"
    }
}
set tb [ptp_cdc::require [ptp_cdc::instances ptp_timebase_cdc] timebase_instances]
foreach i $tb {
    set tick_src [ptp_cdc::bank $i src_tick_gray_r]
    set epoch_src [ptp_cdc::bank $i src_epoch_gray_r]
    # Registered Gray changes at most one bit per source tick/epoch transition.
    ptp_cdc::bundle $tick_src [ptp_cdc::bank $i dst_tick_sync1_r] [ptp_cdc::period $tick_src tick] "$i/tick_gray"
    ptp_cdc::bundle $epoch_src [ptp_cdc::bank $i dst_epoch_sync1_r] [ptp_cdc::period $epoch_src epoch] "$i/epoch_gray"
    set dest [ptp_cdc::bank $i dst_anchor_r]
    # Snapshot remains unchanged until synchronized destination ACK. Detection
    # uses two request FFs before capture; <=1 dst period leaves a full guard.
    ptp_cdc::bundle [ptp_cdc::bank $i src_snapshot_r] $dest [ptp_cdc::period $dest anchor] "$i/anchor"
    foreach s {dst_req_sync_r dst_valid_sync_r src_ack_sync_r} {ptp_cdc::sync [ptp_cdc::bit0 $i $s] "$i/$s"}
}
foreach i [ptp_cdc::require [ptp_cdc::instances net_acq_cfg_cdc_bridge] config_instances] {
    set dst [ptp_cdc::bank $i eth_cfg_bus 0]
    # The legacy RX-ToD config bridge has no data consumer: RX time now comes
    # from the authority timebase. Synthesis removes its entire output bank but
    # leaves request/ACK progress used by software. Log that exact case; all
    # other configuration banks remain mandatory.
    if {[llength $dst]} {
        ptp_cdc::bundle [ptp_cdc::bank $i sys_cfg_hold_r] $dst [ptp_cdc::period $dst cfg] "$i/config"
    } elseif {[file tail $i] eq "u_ptp_rx_cfg_cdc_bridge"} {
        lappend ptp_cdc::audit [list optimized_unused_bundle "$i/config" 0]
        puts "PTP_CDC optimized unused RX-ToD config data bank: $i"
    } else {error "PTP CDC missing live config destination: $i"}
    foreach s {req_toggle_sync1_eth_r ack_toggle_sync1_sys_r} {ptp_cdc::sync [ptp_cdc::bank $i $s] "$i/$s"}
}
foreach typ {net_acq_status_cdc_bridge net_acq_dbgext_cdc_bridge net_acq_txsched_status_cdc_bridge ptp_rx_status_cdc} {
    foreach i [ptp_cdc::require [ptp_cdc::instances $typ] $typ] {
        if {$typ eq "net_acq_dbgext_cdc_bridge"} {
            set src [ptp_cdc::bank $i dbgext_shadow_eth_r]
            set dst [ptp_cdc::bank $i {sys_arb_* sys_backend_backpressure_cycles sys_ptp_tx_*}]
            set controls {dbgext_ack_toggle_sync1_eth_r dbgext_req_toggle_sync1_sys_r}
        } else {
            set src [ptp_cdc::bank $i status_shadow_eth_r]
            switch $typ {
                net_acq_status_cdc_bridge {set stems {sys_stat_tx_packet_count sys_stat_tx_byte_count sys_stat_drop_count sys_stat_last_* sys_phy_*}}
                net_acq_txsched_status_cdc_bridge {set stems {sys_txs_status sys_txs_held_count sys_txs_pass_count sys_txs_late_count sys_txs_slot_miss_count sys_txs_last_*}}
                ptp_rx_status_cdc {set stems {sys_accept_count sys_drop_count sys_last_error sys_fifo_overflow_sticky}}
            }
            set dst [ptp_cdc::bank $i $stems]
            set controls {status_ack_toggle_sync1_eth_r status_req_toggle_sync1_sys_r}
        }
        ptp_cdc::bundle $src $dst [ptp_cdc::period $dst status] "$i/status"
        foreach s $controls {ptp_cdc::sync [ptp_cdc::bank $i $s] "$i/$s"}
        if {$typ eq "net_acq_status_cdc_bridge"} {
            foreach s {busy_sync1_sys_r tx_busy_sync1_sys_r} {ptp_cdc::sync [ptp_cdc::bank $i $s] "$i/$s"}
        }
    }
}
foreach i [ptp_cdc::require [ptp_cdc::instances ptp_txstamp_cdc_bridge] txstamp_instances] {
    set src [ptp_cdc::bank $i src_ts_ns_hold_r];set dst [ptp_cdc::bank $i dst_ts_ns]
    ptp_cdc::bundle $src $dst [ptp_cdc::period $dst stamp] "$i/timestamp"
    set tag_src [ptp_cdc::bank $i src_ts_tag_hold_r 0];set tag_dst [ptp_cdc::bank $i dst_ts_tag 0]
    if {[llength $tag_src] && [llength $tag_dst]} {ptp_cdc::bundle $tag_src $tag_dst [ptp_cdc::period $tag_dst tag] "$i/tag"}
    foreach s {dst_ack_toggle_sync1_src_r src_req_toggle_sync1_dst_r} {ptp_cdc::sync [ptp_cdc::bank $i $s] "$i/$s"}
}
foreach i [ptp_cdc::require [ptp_cdc::instances net_acq_pulse_cdc] pulse_instances] {ptp_cdc::sync [ptp_cdc::bank $i dst_toggle_sync1_r] "$i/pulse"}
# FIFO payloads now use synchronous dual-clock block RAM, so no mem->consumer
# data-path exception is needed. Constrain only registered Gray->entry banks.
foreach typ {net_acq_pkt_async_fifo ptp_rx_async_fifo} {
    foreach i [ptp_cdc::require [ptp_cdc::instances $typ] $typ] {
        foreach {src dst} {wr_gray_r wr_gray_sync1_rd_r rd_gray_r rd_gray_sync1_wr_r} {
            set a [ptp_cdc::bank $i $src];set b [ptp_cdc::bank $i $dst]
            ptp_cdc::bundle $a $b [ptp_cdc::period $a gray] "$i/$src"
        }
    }
}
foreach i [ptp_cdc::require [ptp_cdc::instances top_phy] phy_instances] {
    foreach s {phy_rst_sync_rx_r phy_mac_usable_sync_rx_r} {ptp_cdc::sync [ptp_cdc::bit0 $i $s] "$i/$s"}
}
foreach i [ptp_cdc::require [ptp_cdc::instances ptp_cfg_cmd_manager] command_instances] {
    foreach s {ptp_tx_busy_sys_r ptp_tx_cmd_pending_sys_r} {ptp_cdc::sync [ptp_cdc::bit0 $i $s] "$i/$s"}
}
puts "PTP_CDC groups=[llength $ptp_cdc::audit]"
