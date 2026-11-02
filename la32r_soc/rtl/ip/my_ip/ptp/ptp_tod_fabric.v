`timescale 1ns / 1ps
`default_nettype none
// R1 common-time architecture: TX is the ONLY software-adjustable authority.
// sys/RX reconstruct its time through coherent affine cross-timestamps.
// Legacy input ports retained to avoid a wrapper/CSR ABI change; unused legacy
// sys/RX controls are intentional and must not be re-enabled as extra clocks.
module ptp_tod_fabric #(
    parameter integer P_MAX_ADJ_PPB = 1562500,
    parameter integer P_SYS_PERIOD_PS = 20000,
    parameter integer P_RX_PERIOD_PS = 8000,
    parameter integer P_SYS_CALIBRATION_PS = 0,
    parameter integer P_RX_CALIBRATION_PS = 0
)(
    input  wire         clk_in,
    input  wire         resetn,
    input  wire         eth_tx_clk,
    input  wire         eth_tx_resetn,
    input  wire         eth_rx_clk,
    input  wire         eth_rx_resetn,

    // ------------------------------------------------------------------------
    // Legacy sys control ports retained; authority uses TX active config only.
    // ------------------------------------------------------------------------
    input  wire         cfg_ptp_enable,
    input  wire         cfg_ptp_hold,
    input  wire         cfg_ptp_update_pulse,
    input  wire         cfg_ptp_set_time_pulse,
    input  wire         cfg_ptp_step_pulse,
    input  wire [63:0]  cfg_ptp_set_time_ns,
    input  wire [31:0]  cfg_ptp_step_ns,
    input  wire [31:0]  cfg_ptp_freq_adj_ppb,

    // ------------------------------------------------------------------------
    // eth_tx-domain active config from ptp_cfg_cmd_manager
    // ------------------------------------------------------------------------
    input  wire         tx_cfg_enable_eth,
    input  wire         tx_cfg_hold_eth,
    input  wire         tx_cfg_update_pulse_eth,
    input  wire         tx_cfg_set_time_pulse_eth,
    input  wire         tx_cfg_step_pulse_eth,
    input  wire [63:0]  tx_cfg_set_time_ns_eth,
    input  wire [31:0]  tx_cfg_step_ns_eth,
    input  wire [31:0]  tx_cfg_freq_adj_ppb_eth,

    // ------------------------------------------------------------------------
    // Legacy RX control ports retained; not applied to a second local clock.
    // ------------------------------------------------------------------------
    input  wire         rx_tod_enable_eth,
    input  wire         rx_tod_hold_eth,
    input  wire         rx_tod_update_pulse_eth,
    input  wire         rx_tod_set_time_pulse_eth,
    input  wire         rx_tod_step_pulse_eth,
    input  wire [63:0]  rx_tod_set_time_ns_eth,
    input  wire [31:0]  rx_tod_step_ns_eth,
    input  wire [31:0]  rx_tod_freq_adj_ppb_eth,

    // ------------------------------------------------------------------------
    // Per-domain ToD outputs
    // ------------------------------------------------------------------------
    output wire [63:0]  ptp_sys_tod_time_ns,
    output wire         ptp_sys_tod_time_valid,
    output wire [63:0]  ptp_tod_time_ns_eth,
    output wire         ptp_tod_time_valid_eth,
    output wire [63:0]  ptp_tod_time_ns_rx,
    output wire         ptp_tod_time_valid_rx
);
    wire [31:0] authority_frac_w, authority_epoch_w;
    wire [63:0] authority_inc_w;
    // Status/ILA taps. Existing CSR frequency shadow remains a REQUEST readback.
    wire tx_freq_busy_w, tx_freq_applied_w, tx_freq_limited_w;
    wire signed [31:0] tx_freq_applied_ppb_w;
    wire signed [63:0] tx_freq_delta_fp_w;
    wire [95:0] sys_time_fp_w, rx_time_fp_w;
    wire [31:0] sys_epoch_w, rx_epoch_w, sys_age_ticks_w, rx_age_ticks_w;
    wire sys_monotonic_clamp_w, rx_monotonic_clamp_w;

    ptp_tod_counter #(
        .P_TIME_W(64), .P_FRAC_W(32), .P_ADJ_W(32),
        .P_NS_PER_CYCLE(8), .P_INIT_VALID(0), .P_MAX_ADJ_PPB(P_MAX_ADJ_PPB)
    ) u_ptp_tod_counter_eth (
        .clk(eth_tx_clk), .rst_n(eth_tx_resetn),
        .cfg_enable(tx_cfg_enable_eth), .cfg_hold(tx_cfg_hold_eth),
        .set_valid(tx_cfg_update_pulse_eth && tx_cfg_set_time_pulse_eth),
        .set_time_ns(tx_cfg_set_time_ns_eth),
        .step_valid(tx_cfg_update_pulse_eth && tx_cfg_step_pulse_eth),
        .step_ns(tx_cfg_step_ns_eth),
        .freq_adj_valid(tx_cfg_update_pulse_eth),
        .freq_adj_ppb(tx_cfg_freq_adj_ppb_eth),
        .tod_time_ns(ptp_tod_time_ns_eth), .tod_time_frac(authority_frac_w),
        .tod_time_valid(ptp_tod_time_valid_eth), .pps_pulse(),
        .freq_adj_ppb_cur(tx_freq_applied_ppb_w),
        .freq_adj_busy(tx_freq_busy_w), .freq_adj_applied(tx_freq_applied_w),
        .freq_adj_limited(tx_freq_limited_w), .freq_adj_fp_tick_cur(tx_freq_delta_fp_w),
        .tod_inc_fp(authority_inc_w), .tod_epoch(authority_epoch_w)
    );
    (* KEEP_HIERARCHY = "TRUE" *) ptp_timebase_cdc #(
        .P_SRC_PERIOD_PS(8000), .P_DST_PERIOD_PS(P_SYS_PERIOD_PS),
        .P_CALIBRATION_PS(P_SYS_CALIBRATION_PS)
    ) u_sys_time_bridge (
        .src_clk(eth_tx_clk), .src_resetn(eth_tx_resetn),
        .src_time_fp({ptp_tod_time_ns_eth,authority_frac_w}),
        .src_inc_fp(authority_inc_w), .src_epoch(authority_epoch_w),
        .src_time_valid(ptp_tod_time_valid_eth),
        .dst_clk(clk_in), .dst_resetn(resetn),
        .dst_time_fp(sys_time_fp_w), .dst_time_valid(ptp_sys_tod_time_valid),
        .dst_epoch(sys_epoch_w), .dst_anchor_age_ticks(sys_age_ticks_w),
        .dst_monotonic_clamp(sys_monotonic_clamp_w)
    );
    (* KEEP_HIERARCHY = "TRUE" *) ptp_timebase_cdc #(
        .P_SRC_PERIOD_PS(8000), .P_DST_PERIOD_PS(P_RX_PERIOD_PS),
        .P_CALIBRATION_PS(P_RX_CALIBRATION_PS)
    ) u_rx_time_bridge (
        .src_clk(eth_tx_clk), .src_resetn(eth_tx_resetn),
        .src_time_fp({ptp_tod_time_ns_eth,authority_frac_w}),
        .src_inc_fp(authority_inc_w), .src_epoch(authority_epoch_w),
        .src_time_valid(ptp_tod_time_valid_eth),
        .dst_clk(eth_rx_clk), .dst_resetn(eth_rx_resetn),
        .dst_time_fp(rx_time_fp_w), .dst_time_valid(ptp_tod_time_valid_rx),
        .dst_epoch(rx_epoch_w), .dst_anchor_age_ticks(rx_age_ticks_w),
        .dst_monotonic_clamp(rx_monotonic_clamp_w)
    );
    assign ptp_sys_tod_time_ns = sys_time_fp_w[95:32];
    assign ptp_tod_time_ns_rx = rx_time_fp_w[95:32];
endmodule
`default_nettype wire
