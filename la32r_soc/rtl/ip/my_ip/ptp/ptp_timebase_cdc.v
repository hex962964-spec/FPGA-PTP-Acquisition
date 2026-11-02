`timescale 1ns / 1ps
`default_nettype none
// R1: TX-authority affine time mapper. This is NOT a second adjustable clock.
// Source tick is unit-step binary with REGISTERED Gray encoding. The timestamp
// itself is only crossed as a held/acknowledged bundled-data anchor.
//
// Output convention: registered timestamp estimates authority time at the most
// recent dst rising edge (NOT the next edge). Consumers latch it one edge later,
// just as they latch a normal ToD register. Calibrate that event pipeline at MAC.
// Five destination periods + half a source period compensate nominal age from
// synchronizer entry to registered output. Quantization/metastability uncertainty
// remains; P_CALIBRATION_PS is NOT a universal PHY-latency correction.
//
// The two resets jointly reset THIS BRIDGE, with synchronous release at each end.
// A destination reset never resets the TX authority counter.
module ptp_timebase_cdc #(
    parameter integer P_SRC_PERIOD_PS = 8000,
    parameter integer P_DST_PERIOD_PS = 20000,
    parameter integer P_CALIBRATION_PS = 0,
    parameter integer P_SNAPSHOT_INTERVAL_CYCLES = 64,
    parameter integer P_MAX_ANCHOR_AGE_TICKS = 4096,
    parameter integer P_STOP_TIMEOUT_CYCLES = 32,
    parameter [31:0] P_TICK_INIT = 32'd0
)(
    input wire src_clk, input wire src_resetn,
    input wire [95:0] src_time_fp,
    input wire [63:0] src_inc_fp,
    input wire [31:0] src_epoch,
    input wire src_time_valid,
    input wire dst_clk, input wire dst_resetn,
    output reg [95:0] dst_time_fp,
    output reg dst_time_valid,
    output reg [31:0] dst_epoch,
    output reg [31:0] dst_anchor_age_ticks,
    output reg dst_monotonic_clamp
);
    localparam integer C_AGE_PS = 5*P_DST_PERIOD_PS + P_SRC_PERIOD_PS/2 + P_CALIBRATION_PS;
    localparam [31:0] C_AGE_TICKS_Q16 = (64'd65536 * C_AGE_PS) / P_SRC_PERIOD_PS;
    // synthesis translate_off
    initial begin
        if (P_SRC_PERIOD_PS < 1000 || P_DST_PERIOD_PS <= 0 || C_AGE_PS < 0 ||
            C_AGE_PS > 1000000 || P_SNAPSHOT_INTERVAL_CYCLES < 4 ||
            P_MAX_ANCHOR_AGE_TICKS >= 65536 || P_MAX_ANCHOR_AGE_TICKS < 128 ||
            P_SNAPSHOT_INTERVAL_CYCLES >= P_MAX_ANCHOR_AGE_TICKS/2 ||
            P_STOP_TIMEOUT_CYCLES < 8)
            $fatal(1, "ptp_timebase_cdc: unsupported timing parameters");
    end
    // synthesis translate_on

    wire bridge_arst_n = src_resetn && dst_resetn;
    (* ASYNC_REG="TRUE", SHREG_EXTRACT="NO" *) reg [1:0] src_reset_sync_r, dst_reset_sync_r;
    always @(posedge src_clk or negedge bridge_arst_n)
        if (!bridge_arst_n) src_reset_sync_r <= 0;
        else src_reset_sync_r <= {src_reset_sync_r[0],1'b1};
    always @(posedge dst_clk or negedge bridge_arst_n)
        if (!bridge_arst_n) dst_reset_sync_r <= 0;
        else dst_reset_sync_r <= {dst_reset_sync_r[0],1'b1};
    wire s_resetn = src_reset_sync_r[1];
    wire d_resetn = dst_reset_sync_r[1];

    function [31:0] f_gray_to_binary;
        input [31:0] gray;
        integer i;
        begin
            f_gray_to_binary[31] = gray[31];
            for (i=30; i>=0; i=i-1)
                f_gray_to_binary[i] = f_gray_to_binary[i+1] ^ gray[i];
        end
    endfunction

    reg [31:0] src_tick_r;
    reg [31:0] src_tick_gray_r, src_epoch_gray_r;
    reg [223:0] src_snapshot_r; // epoch:32 | inc:64 | tick:32 | time:96
    reg src_req_r, src_anchor_sent_r;
    reg [31:0] src_interval_r;
    reg dst_ack_r;
    (* ASYNC_REG="TRUE", SHREG_EXTRACT="NO" *) reg [1:0] src_ack_sync_r;
    wire [31:0] src_tick_next_w = src_tick_r + 32'd1;
    wire source_busy_w = src_req_r != src_ack_sync_r[1];
    wire anchor_due_w = !src_anchor_sent_r ||
        (src_interval_r >= P_SNAPSHOT_INTERVAL_CYCLES-1) ||
        (src_snapshot_r[223:192] != src_epoch) ||
        (src_snapshot_r[191:128] != src_inc_fp);

    always @(posedge src_clk or negedge s_resetn) begin
        if (!s_resetn) begin
            src_tick_r <= P_TICK_INIT;
            src_tick_gray_r <= P_TICK_INIT ^ (P_TICK_INIT >> 1);
            src_epoch_gray_r <= 0;
            src_snapshot_r <= 0;
            src_req_r <= 0;
            src_anchor_sent_r <= 0;
            src_interval_r <= 0;
            src_ack_sync_r <= 0;
        end else begin
            src_tick_r <= src_tick_next_w;
            src_tick_gray_r <= src_tick_next_w ^ (src_tick_next_w >> 1);
            src_epoch_gray_r <= src_epoch ^ (src_epoch >> 1);
            src_ack_sync_r <= {src_ack_sync_r[0],dst_ack_r};
            if (src_interval_r < P_SNAPSHOT_INTERVAL_CYCLES)
                src_interval_r <= src_interval_r + 32'd1;
            if (!source_busy_w && src_time_valid && anchor_due_w) begin
                // All four fields refer to the same source pre-edge state.
                // Do not overwrite until the matching destination ACK returns.
                src_snapshot_r <= {src_epoch,src_inc_fp,src_tick_r,src_time_fp};
                src_req_r <= ~src_req_r;
                src_anchor_sent_r <= 1'b1;
                src_interval_r <= 0;
            end
        end
    end

    (* ASYNC_REG="TRUE", SHREG_EXTRACT="NO" *) reg [31:0] dst_tick_sync1_r, dst_tick_sync2_r;
    (* ASYNC_REG="TRUE", SHREG_EXTRACT="NO" *) reg [31:0] dst_epoch_sync1_r, dst_epoch_sync2_r;
    (* ASYNC_REG="TRUE", SHREG_EXTRACT="NO" *) reg [1:0] dst_req_sync_r, dst_valid_sync_r;
    reg dst_req_seen_r, dst_have_anchor_r;
    reg [223:0] dst_anchor_r;
    reg [31:0] dst_last_tick_r, dst_idle_cycles_r;
    wire [31:0] observed_tick_w = f_gray_to_binary(dst_tick_sync2_r);
    wire [31:0] observed_epoch_w = f_gray_to_binary(dst_epoch_sync2_r);
    wire [31:0] delta_ticks_w = observed_tick_w - dst_anchor_r[127:96];
    wire live_w = dst_valid_sync_r[1] && (dst_idle_cycles_r < P_STOP_TIMEOUT_CYCLES);

    reg [15:0] p0_delta_r;
    reg [95:0] p0_time_r;
    reg [39:0] p0_inc_r; // positive Q32 increment < 256 ns/source tick
    reg [31:0] p0_epoch_r, p0_age_r;
    reg p0_valid_r;
    reg [55:0] p1_elapsed_r;
    reg [71:0] p1_age_product_r;
    reg [95:0] p1_time_r;
    reg [31:0] p1_epoch_r, p1_age_r;
    reg p1_valid_r, have_output_r;
    reg [56:0] p2_delta_r;
    reg [95:0] p2_time_r;
    reg [31:0] p2_epoch_r, p2_age_r;
    reg p2_valid_r;
    wire [96:0] candidate_w = {1'b0,p2_time_r} + {{40{1'b0}},p2_delta_r};

    always @(posedge dst_clk or negedge d_resetn) begin
        if (!d_resetn) begin
            dst_tick_sync1_r <= 0; dst_tick_sync2_r <= 0;
            dst_epoch_sync1_r <= 0; dst_epoch_sync2_r <= 0;
            dst_req_sync_r <= 0; dst_valid_sync_r <= 0;
            dst_req_seen_r <= 0; dst_ack_r <= 0;
            dst_anchor_r <= 0; dst_have_anchor_r <= 0;
            dst_last_tick_r <= 0; dst_idle_cycles_r <= 0;
            p0_delta_r <= 0; p0_time_r <= 0; p0_inc_r <= 0;
            p0_epoch_r <= 0; p0_age_r <= 0; p0_valid_r <= 0;
            p1_elapsed_r <= 0; p1_age_product_r <= 0; p1_time_r <= 0;
            p1_epoch_r <= 0; p1_age_r <= 0; p1_valid_r <= 0;
            p2_delta_r <= 0; p2_time_r <= 0; p2_epoch_r <= 0;
            p2_age_r <= 0; p2_valid_r <= 0;
            dst_time_fp <= 0; dst_time_valid <= 0; dst_epoch <= 0;
            dst_anchor_age_ticks <= 0; dst_monotonic_clamp <= 0;
            have_output_r <= 0;
        end else begin
            dst_tick_sync1_r <= src_tick_gray_r;
            dst_tick_sync2_r <= dst_tick_sync1_r;
            dst_epoch_sync1_r <= src_epoch_gray_r;
            dst_epoch_sync2_r <= dst_epoch_sync1_r;
            dst_req_sync_r <= {dst_req_sync_r[0],src_req_r};
            dst_valid_sync_r <= {dst_valid_sync_r[0],src_time_valid};
            if (dst_req_sync_r[1] != dst_req_seen_r) begin
                dst_anchor_r <= src_snapshot_r;
                dst_have_anchor_r <= 1'b1;
                dst_req_seen_r <= dst_req_sync_r[1];
                dst_ack_r <= dst_req_sync_r[1];
            end
            dst_last_tick_r <= observed_tick_w;
            if (observed_tick_w != dst_last_tick_r) dst_idle_cycles_r <= 0;
            else if (dst_idle_cycles_r < P_STOP_TIMEOUT_CYCLES)
                dst_idle_cycles_r <= dst_idle_cycles_r + 32'd1;

            // Pipeline A: sample one coherent mapping and source tick.
            p0_delta_r <= delta_ticks_w[15:0];
            p0_time_r <= dst_anchor_r[95:0];
            p0_inc_r <= dst_anchor_r[167:128];
            p0_epoch_r <= dst_anchor_r[223:192];
            p0_age_r <= delta_ticks_w;
            p0_valid_r <= dst_have_anchor_r && live_w &&
                (delta_ticks_w <= P_MAX_ANCHOR_AGE_TICKS) &&
                (dst_anchor_r[191:168] == 24'd0) &&
                (dst_anchor_r[223:192] == observed_epoch_w);
            // Pipeline B: source-tick extrapolation and nominal age compensation.
            p1_elapsed_r <= p0_delta_r * p0_inc_r;
            p1_age_product_r <= p0_inc_r * C_AGE_TICKS_Q16;
            p1_time_r <= p0_time_r;
            p1_epoch_r <= p0_epoch_r;
            p1_age_r <= p0_age_r;
            p1_valid_r <= p0_valid_r;
            // Pipeline C: combine the two short deltas. No cascaded 96-bit adds.
            p2_delta_r <= {1'b0,p1_elapsed_r} + {1'b0,p1_age_product_r[71:16]};
            p2_time_r <= p1_time_r;
            p2_epoch_r <= p1_epoch_r;
            p2_age_r <= p1_age_r;
            p2_valid_r <= p1_valid_r;
            // Pipeline D: one wide add; qualify against CURRENT epoch/liveness.
            dst_time_valid <= 1'b0;
            dst_monotonic_clamp <= 1'b0;
            if (p2_valid_r && live_w && (p2_epoch_r == observed_epoch_w) && !candidate_w[96]) begin
                dst_time_valid <= 1'b1;
                dst_epoch <= p2_epoch_r;
                dst_anchor_age_ticks <= p2_age_r;
                have_output_r <= 1'b1;
                if (have_output_r && (dst_epoch == p2_epoch_r) &&
                    (candidate_w[95:0] < dst_time_fp)) begin
                    // Rate-anchor changes may make a sub-tick negative estimate.
                    // Hold (never regress) within an epoch; no hidden time step.
                    dst_monotonic_clamp <= 1'b1;
                end else begin
                    dst_time_fp <= candidate_w[95:0];
                end
            end
        end
    end
endmodule
`default_nettype wire
