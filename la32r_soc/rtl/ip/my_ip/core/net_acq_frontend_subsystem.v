`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_frontend_subsystem
// Description:
//   System-clock-domain front-end wrapper for the net_acq acquisition path.
//
//   This module deliberately keeps the original front-end behavior unchanged,
//   but gathers the following logic into a single place so that
//   net_acq_wrapper.v can become a thin integration shell:
//     1) acquisition_core
//     2) passive tx_debug_sink
//     3) packet timestamp source selection / PTP-ToD qualification
//
//   Notes:
//     - PTP-corrected ToD is only exposed to acquisition_core after software has
//       both enabled PTP and issued at least one explicit set_time pulse.
//     - R1: legacy fallback is allowed only before first qualified PTP time.
//       Later invalidity latches a stop-required timestamp fault in the core.
//////////////////////////////////////////////////////////////////////////////////
module net_acq_frontend_subsystem(
    input  wire         clk_in,
    input  wire         resetn,

    // ------------------------------------------------------------------------
    // Front-end control from CSR / wrapper
    // ------------------------------------------------------------------------
    input  wire         cfg_enable,
    input  wire         cfg_soft_reset_pulse,
    input  wire         cfg_clear_status_pulse,
    input  wire [31:0]  cfg_sample_period,
    input  wire [15:0]  cfg_samples_per_packet,
    input  wire [3:0]   cfg_mode,
    input  wire [15:0]  cfg_channel_id,
    input  wire         cfg_src_sel,
    input  wire [1:0]   cfg_time_quality,

    // ------------------------------------------------------------------------
    // PTP-qualified system-domain ToD input
    // ------------------------------------------------------------------------
    input  wire         cfg_ptp_enable,
    input  wire         cfg_ptp_set_time_pulse,
    input  wire [63:0]  ptp_sys_tod_time_ns,
    input  wire         ptp_sys_tod_time_valid,

    // ------------------------------------------------------------------------
    // External sample input (sys_clk domain)
    // ------------------------------------------------------------------------
    input  wire [15:0]  sample_data_in,
    input  wire         sample_valid_in,
    output wire         sample_ready_out,

    // ------------------------------------------------------------------------
    // Packet output stream (sys_clk domain)
    // ------------------------------------------------------------------------
    output wire [15:0]  pkt_data,
    output wire         pkt_valid,
    output wire         pkt_last,
    input  wire         pkt_ready,

    // ------------------------------------------------------------------------
    // Raw acquisition_core status (used by commit/safety logic)
    // ------------------------------------------------------------------------
    output wire         core_stat_busy,
    output wire         core_stat_error,
    output wire [31:0]  core_stat_fifo_level,
    output wire [31:0]  core_stat_packet_count,
    output wire [63:0]  core_stat_last_timestamp,

    // ------------------------------------------------------------------------
    // Passive monitor / debug sink outputs
    // ------------------------------------------------------------------------
    output wire         dbg_error_sticky,
    output wire [31:0]  dbg_good_packet_count,
    output wire [31:0]  dbg_bad_packet_count,
    output wire [31:0]  dbg_last_seq_id,
    output wire [63:0]  dbg_last_timestamp,
    output wire [15:0]  dbg_last_sample_count,
    output wire [15:0]  dbg_last_payload_count,
    output wire [15:0]  dbg_last_error_code,
    output wire [15:0]  dbg_last_error_word_index,
    output wire [7:0]   dbg_last_protocol_version,
    output wire [1:0]   dbg_last_quality
);

    // ------------------------------------------------------------------------
    // Only after software has explicitly executed a set_time pulse do we treat
    // the system-domain PTP ToD as the calibrated packet timestamp source.
    // ------------------------------------------------------------------------
    reg ptp_sys_tod_initialized_r;
    wire [63:0] core_timestamp_ns_w;
    wire        core_timestamp_valid_w;
    wire        core_enable_w;

    assign core_timestamp_ns_w    = ptp_sys_tod_time_ns;
    assign core_timestamp_valid_w = ptp_sys_tod_initialized_r &&
                                    cfg_ptp_enable &&
                                    ptp_sys_tod_time_valid;
    assign core_enable_w          = cfg_enable && !cfg_soft_reset_pulse;

    always @(posedge clk_in or negedge resetn) begin
        if (!resetn)
            ptp_sys_tod_initialized_r <= 1'b0;
        else if (!cfg_ptp_enable)
            ptp_sys_tod_initialized_r <= 1'b0;
        else if (cfg_ptp_set_time_pulse)
            ptp_sys_tod_initialized_r <= 1'b1;
    end

    // ------------------------------------------------------------------------
    // R1 acquisition_core includes a sticky timestamp-source validity guard.
    // ------------------------------------------------------------------------
    acquisition_core u_acquisition_core (
        .clk_in                  (clk_in),
        .resetn                  (resetn),
        .cfg_enable              (core_enable_w),
        .cfg_clear_status        (cfg_clear_status_pulse),
        .cfg_sample_period       (cfg_sample_period),
        .cfg_samples_per_packet  (cfg_samples_per_packet),
        .cfg_mode                (cfg_mode),
        .cfg_channel_id          (cfg_channel_id),
        .cfg_src_sel             (cfg_src_sel),
        .cfg_time_quality        (cfg_time_quality),
        .ext_timestamp_ns        (core_timestamp_ns_w),
        .ext_timestamp_valid     (core_timestamp_valid_w),
        .sample_data_in          (sample_data_in),
        .sample_valid_in         (sample_valid_in),
        .sample_ready_out        (sample_ready_out),
        .pkt_data                (pkt_data),
        .pkt_valid               (pkt_valid),
        .pkt_last                (pkt_last),
        .pkt_ready               (pkt_ready),
        .stat_busy               (core_stat_busy),
        .stat_error              (core_stat_error),
        .stat_fifo_level         (core_stat_fifo_level),
        .stat_packet_count       (core_stat_packet_count),
        .stat_last_timestamp     (core_stat_last_timestamp)
    );

    // ------------------------------------------------------------------------
    // Passive tx_debug_sink observes the real packet handshakes that leave the
    // front-end. When the wrapper later redirects pkt_ready to the CDC FIFO,
    // this sink still naturally tracks the true accepted stream.
    // ------------------------------------------------------------------------
    tx_debug_sink u_tx_debug_sink (
        .clk_in                    (clk_in),
        .resetn                    (resetn),
        .enable                    (cfg_enable),
        .clear                     (cfg_soft_reset_pulse),
        .clear_status              (cfg_clear_status_pulse),
        .pkt_data                  (pkt_data),
        .pkt_valid                 (pkt_valid),
        .pkt_last                  (pkt_last),
        .pkt_ready                 (pkt_ready),
        .dbg_error_sticky          (dbg_error_sticky),
        .dbg_good_packet_count     (dbg_good_packet_count),
        .dbg_bad_packet_count      (dbg_bad_packet_count),
        .dbg_last_seq_id           (dbg_last_seq_id),
        .dbg_last_timestamp        (dbg_last_timestamp),
        .dbg_last_sample_count     (dbg_last_sample_count),
        .dbg_last_payload_count    (dbg_last_payload_count),
        .dbg_last_error_code       (dbg_last_error_code),
        .dbg_last_error_word_index (dbg_last_error_word_index),
        .dbg_last_protocol_version (dbg_last_protocol_version),
        .dbg_last_quality          (dbg_last_quality)
    );

endmodule
