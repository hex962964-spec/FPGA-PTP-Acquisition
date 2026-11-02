`timescale 1ns / 1ps
`default_nettype none
// R1: integer-ns/Q32 authority counter. See docs/02_architecture.md.
// Existing named ports are retained. New status ports are optional for callers.
// freq_adj_valid: latest-wins request; restarts a 64-cycle exact converter.
// SET assigns the next counter value. STEP adds to the normal running tick.
// No fractional conversion/division is present in the per-tick feedback path.
module ptp_tod_counter #(
    parameter integer P_TIME_W = 64,
    parameter integer P_FRAC_W = 32,
    parameter integer P_ADJ_W = 32,
    parameter integer P_NS_PER_CYCLE = 8,
    parameter integer P_ENABLE_PPS = 0,
    parameter [P_TIME_W-1:0] P_INIT_TIME_NS = {P_TIME_W{1'b0}},
    parameter integer P_INIT_VALID = 1,
    parameter integer P_MAX_ADJ_PPB = 1562500
)(
    input wire clk, input wire rst_n,
    input wire cfg_enable, input wire cfg_hold,
    input wire set_valid, input wire [P_TIME_W-1:0] set_time_ns,
    input wire step_valid, input wire signed [31:0] step_ns,
    input wire freq_adj_valid,
    input wire signed [P_ADJ_W-1:0] freq_adj_ppb,
    output wire [P_TIME_W-1:0] tod_time_ns,
    output wire [P_FRAC_W-1:0] tod_time_frac,
    output reg tod_time_valid,
    output reg signed [P_ADJ_W-1:0] freq_adj_ppb_cur,
    output wire pps_pulse,
    output reg freq_adj_busy,
    output reg freq_adj_applied,
    output reg freq_adj_limited,
    output wire signed [63:0] freq_adj_fp_tick_cur,
    output wire [63:0] tod_inc_fp,
    output reg [31:0] tod_epoch
);
    localparam integer C_FP_W = P_TIME_W + P_FRAC_W;
    localparam [63:0] C_BASE_FP = (64'd1 * P_NS_PER_CYCLE) << P_FRAC_W;
    localparam [63:0] C_ONE_SEC_FP = 64'd1000000000 << P_FRAC_W;
    localparam signed [31:0] C_LIMIT = P_MAX_ADJ_PPB;

    // Verified parameter envelope; product + rounding fits unsigned 64 bits.
    // synthesis translate_off
    initial begin
        if (P_TIME_W < 64 || P_ADJ_W != 32 || P_FRAC_W < 16 || P_FRAC_W > 32 ||
            P_NS_PER_CYCLE < 1 || P_NS_PER_CYCLE > 32 ||
            P_MAX_ADJ_PPB < 1 || P_MAX_ADJ_PPB > 10000000)
            $fatal(1, "ptp_tod_counter: unsupported parameter combination");
    end
    // synthesis translate_on

    reg [C_FP_W-1:0] tod_fp_r;
    reg signed [63:0] freq_delta_r;
    reg [63:0] div_bits_r;
    reg [29:0] div_rem_r;
    reg [6:0] div_count_r;
    reg req_negative_r, req_limited_r;
    reg signed [31:0] req_ppb_r;
    reg running_prev_r;
    wire running_w = cfg_enable && !cfg_hold;

    wire signed [31:0] clamped_ppb_w =
        ($signed(freq_adj_ppb) > C_LIMIT) ? C_LIMIT :
        (($signed(freq_adj_ppb) < -C_LIMIT) ? -C_LIMIT : $signed(freq_adj_ppb));
    wire [31:0] magnitude_w = clamped_ppb_w[31] ?
        $unsigned(-clamped_ppb_w) : $unsigned(clamped_ppb_w);
    wire [63:0] numerator_w = C_BASE_FP * {32'd0, magnitude_w} + 64'd500000000;
    wire [30:0] trial_rem_w = {div_rem_r, div_bits_r[63]};
    wire quotient_bit_w = trial_rem_w >= 31'd1000000000;
    wire [30:0] reduced_rem_w = quotient_bit_w ?
        trial_rem_w - 31'd1000000000 : trial_rem_w;
    wire [63:0] quotient_next_w = {div_bits_r[62:0], quotient_bit_w};
    wire commit_w = freq_adj_busy && (div_count_r == 7'd1) && !freq_adj_valid;

    assign tod_time_ns = tod_fp_r[C_FP_W-1:P_FRAC_W];
    assign tod_time_frac = tod_fp_r[P_FRAC_W-1:0];
    assign freq_adj_fp_tick_cur = freq_delta_r;
    assign tod_inc_fp = running_w ? (C_BASE_FP + $unsigned(freq_delta_r)) : 64'd0;

    wire signed [C_FP_W:0] step_ext_w =
        $signed({{(C_FP_W+1-32){step_ns[31]}}, step_ns}) <<< P_FRAC_W;
    reg signed [C_FP_W:0] sum_v;
    reg [C_FP_W-1:0] next_tod_w;
    always @* begin
        sum_v = $signed({1'b0,tod_fp_r}) +
                $signed({{(C_FP_W+1-64){1'b0}},tod_inc_fp});
        if (step_valid) sum_v = sum_v + step_ext_w;
        if (set_valid)
            next_tod_w = {set_time_ns,{P_FRAC_W{1'b0}}};
        else if (sum_v < 0)
            next_tod_w = {C_FP_W{1'b0}};
        else
            next_tod_w = sum_v[C_FP_W-1:0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tod_fp_r <= {P_INIT_TIME_NS,{P_FRAC_W{1'b0}}};
            tod_time_valid <= (P_INIT_VALID != 0);
            freq_delta_r <= 64'sd0;
            freq_adj_ppb_cur <= 0;
            freq_adj_busy <= 1'b0;
            freq_adj_applied <= 1'b0;
            freq_adj_limited <= 1'b0;
            div_bits_r <= 0;
            div_rem_r <= 0;
            div_count_r <= 0;
            req_ppb_r <= 0;
            req_negative_r <= 0;
            req_limited_r <= 0;
            running_prev_r <= 0;
            tod_epoch <= 0;
        end else begin
            tod_fp_r <= next_tod_w;
            if (set_valid) tod_time_valid <= 1'b1;
            running_prev_r <= running_w;
            // Rate-only updates are continuous: bridges refresh their affine
            // anchors without invalidating the sampling time on every FREQ.
            if (set_valid || step_valid || (running_w != running_prev_r))
                tod_epoch <= tod_epoch + 32'd1;
            freq_adj_applied <= 1'b0;
            if (freq_adj_valid) begin
                div_bits_r <= numerator_w;
                div_rem_r <= 0;
                div_count_r <= 7'd64;
                freq_adj_busy <= 1'b1;
                req_ppb_r <= clamped_ppb_w;
                req_negative_r <= clamped_ppb_w[31];
                req_limited_r <= ($signed(freq_adj_ppb) != clamped_ppb_w);
            end else if (freq_adj_busy) begin
                div_bits_r <= quotient_next_w;
                div_rem_r <= reduced_rem_w[29:0];
                div_count_r <= div_count_r - 7'd1;
                if (commit_w) begin
                    freq_delta_r <= req_negative_r ? -$signed(quotient_next_w) : $signed(quotient_next_w);
                    freq_adj_ppb_cur <= req_ppb_r;
                    freq_adj_limited <= req_limited_r;
                    freq_adj_busy <= 1'b0;
                    freq_adj_applied <= 1'b1;
                end
            end
        end
    end

    // Optional PPS logic is elaborated away when P_ENABLE_PPS == 0.
    // Explicit SET/STEP re-anchor the subsecond phase without a fake PPS.
    generate if (P_ENABLE_PPS != 0) begin : g_pps
        reg [63:0] subsec_r;
        reg pps_r;
        wire [64:0] next_subsec_w = {1'b0,subsec_r} + {1'b0,tod_inc_fp};
        assign pps_pulse = pps_r;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                subsec_r <= (P_INIT_TIME_NS % 64'd1000000000) << P_FRAC_W;
                pps_r <= 1'b0;
            end else begin
                pps_r <= 1'b0;
                if (set_valid || step_valid) begin
                    subsec_r <= next_tod_w % C_ONE_SEC_FP;
                end else if (next_subsec_w >= {1'b0,C_ONE_SEC_FP}) begin
                    subsec_r <= next_subsec_w[63:0] - C_ONE_SEC_FP;
                    pps_r <= tod_time_valid;
                end else begin
                    subsec_r <= next_subsec_w[63:0];
                end
            end
        end
    end else begin : g_no_pps
        assign pps_pulse = 1'b0;
    end endgenerate
endmodule
`default_nettype wire
