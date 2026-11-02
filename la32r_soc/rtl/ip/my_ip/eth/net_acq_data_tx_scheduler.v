`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_data_tx_scheduler
// Description:
//   TSN-lite v1 slot-based data egress scheduler.
//
//   Placement:
//     net_acq_eth_backend_top -> net_acq_data_tx_scheduler -> tx_arbiter
//
//   Scope of v1:
//     1) Only gate the normal data path; PTP path is left untouched.
//     2) Only decide whether the FIRST byte of a frame may start in the current
//        time slot. Once the first byte handshakes, the rest of the frame is
//        drained continuously.
//     3) Use eth_tx-domain ToD as the only authoritative scheduling time base.
//     4) Export counters and last plan/actual/delta information for bring-up
//        and future TSN-lite proof.
//
//   Design notes:
//     - The scheduler is intentionally non-buffering. It relies on the existing
//       backend store-and-forward staging. Therefore s_tvalid may stay high for
//       the first byte while the scheduler holds s_tready low.
//     - current_slot_start_ns_r tracks the current candidate slot. Software is
//       expected to program cfg_base_time_ns close to the current ToD (typical
//       TSN usage) so the one-period-per-cycle catch-up logic converges quickly.
//////////////////////////////////////////////////////////////////////////////////
module net_acq_data_tx_scheduler #(
    parameter integer P_DATA_W  = 8,
    parameter integer P_TUSER_W = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ------------------------------------------------------------------------
    // Common control (eth_tx domain)
    // ------------------------------------------------------------------------
    input  wire                     cfg_enable,
    input  wire                     cfg_soft_reset,
    input  wire                     cfg_clear_status,

    // ------------------------------------------------------------------------
    // Active scheduler configuration (eth_tx domain)
    // ------------------------------------------------------------------------
    input  wire                     cfg_sched_update_pulse,
    input  wire                     cfg_sched_enable,
    input  wire                     cfg_sched_block_when_time_invalid,
    input  wire                     cfg_sched_strict_window,
    input  wire [63:0]              cfg_base_time_ns,
    input  wire [31:0]              cfg_period_ns,
    input  wire [31:0]              cfg_slot_offset_ns,
    input  wire [31:0]              cfg_slot_width_ns,
    input  wire [31:0]              cfg_advance_ns,
    input  wire [31:0]              cfg_late_tol_ns,

    // ------------------------------------------------------------------------
    // Authoritative eth_tx ToD
    // ------------------------------------------------------------------------
    input  wire [63:0]              tod_time_ns,
    input  wire                     tod_time_valid,

    // ------------------------------------------------------------------------
    // Input byte stream from data backend
    // ------------------------------------------------------------------------
    input  wire [P_DATA_W-1:0]      s_tdata,
    input  wire                     s_tvalid,
    input  wire                     s_tlast,
    output wire                     s_tready,
    input  wire [P_TUSER_W-1:0]     s_tuser,

    // ------------------------------------------------------------------------
    // Output byte stream to tx_arbiter
    // ------------------------------------------------------------------------
    output wire [P_DATA_W-1:0]      m_tdata,
    output wire                     m_tvalid,
    output wire                     m_tlast,
    input  wire                     m_tready,
    output wire [P_TUSER_W-1:0]     m_tuser,

    // ------------------------------------------------------------------------
    // Status / observability (eth_tx domain)
    // ------------------------------------------------------------------------
    output wire [31:0]              stat_status,
    output reg  [31:0]              stat_held_count,
    output reg  [31:0]              stat_pass_count,
    output reg  [31:0]              stat_late_count,
    output reg  [31:0]              stat_slot_miss_count,
    output reg  [63:0]              stat_last_plan_ns,
    output reg  [63:0]              stat_last_actual_ns,
    output reg  [31:0]              stat_last_delta_ns
);

    // TXS_STATUS bit contract (v1, frozen in Stage A):
    //   bit0  : scheduler enable active
    //   bit1  : authoritative eth_tx ToD currently valid
    //   bit2  : a frame is waiting to be launched
    //   bit3  : release gate already armed for the pending frame
    //   bit4  : a frame is currently draining through the scheduler
    //   bit5  : active scheduler configuration is structurally valid
    //   bit6  : block_when_time_invalid active
    //   bit7  : strict_window active
    //   bit8  : last launch was marked late
    //   bit9  : last launch missed the current slot window
    //   bit10 : release window currently open
    //   bit11 : scheduler is in bypass mode
    localparam [31:0] STATUS_BIT_ENABLE            = 32'h0000_0001;
    localparam [31:0] STATUS_BIT_TOD_VALID         = 32'h0000_0002;
    localparam [31:0] STATUS_BIT_PENDING           = 32'h0000_0004;
    localparam [31:0] STATUS_BIT_RELEASE_ARMED     = 32'h0000_0008;
    localparam [31:0] STATUS_BIT_FRAME_ACTIVE      = 32'h0000_0010;
    localparam [31:0] STATUS_BIT_CFG_VALID         = 32'h0000_0020;
    localparam [31:0] STATUS_BIT_BLOCK_ON_BAD_TIME = 32'h0000_0040;
    localparam [31:0] STATUS_BIT_STRICT_WINDOW     = 32'h0000_0080;
    localparam [31:0] STATUS_BIT_LAST_LATE         = 32'h0000_0100;
    localparam [31:0] STATUS_BIT_LAST_SLOT_MISS    = 32'h0000_0200;
    localparam [31:0] STATUS_BIT_WINDOW_OPEN       = 32'h0000_0400;
    localparam [31:0] STATUS_BIT_BYPASS            = 32'h0000_0800;

    reg         frame_active_r;
    reg         release_armed_r;
    reg         waiting_accounted_r;
    reg         last_launch_late_r;
    reg         last_launch_slot_miss_r;
    reg [63:0]  current_slot_start_ns_r;
    reg [63:0]  armed_release_open_ns_r;
    reg [63:0]  armed_slot_end_ns_r;

    wire        sched_cfg_valid_w;
    wire        sched_time_usable_w;
    wire        pending_frame_w;
    wire [63:0] release_open_ns_w;
    wire [63:0] slot_end_ns_w;
    wire        release_window_open_w;
    wire        current_slot_expired_w;
    wire        open_now_w;
    wire        allow_first_byte_w;
    wire        allow_stream_w;
    wire        first_byte_fire_w;
    wire        any_byte_fire_w;
    wire [63:0] plan_time_now_w;
    wire [63:0] actual_time_now_w;
    wire [63:0] slot_start_plus_period_w;
    wire [63:0] late_deadline_ns_w;
    // Keep the plan-vs-actual subtraction as an explicit 64-bit wire so the
    // code stays Verilog-2001 friendly.  Vivado accepts slicing a named vector,
    // but may reject direct expression slicing such as (a - b)[31:0] in .v
    // files when not parsed as SystemVerilog.
    wire [63:0] actual_minus_plan_w;
    wire        actual_late_w;
    wire        actual_slot_miss_w;
    wire        armed_past_end_w;
    wire        armed_window_eligible_w;

    // {equal, greater}; combine byte comparisons in a balanced lexicographic
    // tree rather than making a 64-bit subtract carry the release decision.
    function [1:0] f_cmp64;
        input [63:0] a;
        input [63:0] b;
        reg [7:0] g8,e8;
        reg [3:0] g16,e16;
        reg [1:0] g32,e32;
        reg g64,e64;
        integer i;
        begin
            for (i=0; i<8; i=i+1) begin
                g8[i] = a[i*8 +: 8] > b[i*8 +: 8];
                e8[i] = a[i*8 +: 8] == b[i*8 +: 8];
            end
            for (i=0; i<4; i=i+1) begin
                g16[i] = g8[2*i+1] | (e8[2*i+1] & g8[2*i]);
                e16[i] = e8[2*i+1] & e8[2*i];
            end
            for (i=0; i<2; i=i+1) begin
                g32[i] = g16[2*i+1] | (e16[2*i+1] & g16[2*i]);
                e32[i] = e16[2*i+1] & e16[2*i];
            end
            g64 = g32[1] | (e32[1] & g32[0]);
            e64 = e32[1] & e32[0];
            f_cmp64 = {e64,g64};
        end
    endfunction

    function f_ge64;
        input [63:0] a;
        input [63:0] b;
        reg [1:0] comparison;
        begin comparison=f_cmp64(a,b); f_ge64=|comparison; end
    endfunction

    function f_gt64;
        input [63:0] a;
        input [63:0] b;
        reg [1:0] comparison;
        begin comparison=f_cmp64(a,b); f_gt64=comparison[0]; end
    endfunction

    function [63:0] f_add64_32;
        input [63:0] a;
        input [31:0] b;
        begin
            f_add64_32 = a + {32'd0, b};
        end
    endfunction

    function [63:0] f_sub64_32_sat0;
        input [63:0] a;
        input [31:0] b;
        begin
            if (a >= {32'd0, b})
                f_sub64_32_sat0 = a - {32'd0, b};
            else
                f_sub64_32_sat0 = 64'd0;
        end
    endfunction

    assign sched_cfg_valid_w = (cfg_period_ns != 32'd0) &&
                               (cfg_slot_width_ns != 32'd0) &&
                               (cfg_slot_offset_ns < cfg_period_ns) &&
                               (cfg_slot_width_ns  <= cfg_period_ns);

    assign sched_time_usable_w = tod_time_valid || (!cfg_sched_block_when_time_invalid);
    assign pending_frame_w     = (!frame_active_r) && s_tvalid;
    assign release_open_ns_w   = f_sub64_32_sat0(current_slot_start_ns_r, cfg_advance_ns);
    assign slot_end_ns_w       = f_add64_32(current_slot_start_ns_r, cfg_slot_width_ns);
    assign slot_start_plus_period_w = f_add64_32(current_slot_start_ns_r, cfg_period_ns);
    assign current_slot_expired_w   = tod_time_valid && (tod_time_ns > slot_end_ns_w);
    assign release_window_open_w    = sched_time_usable_w && (tod_time_ns >= release_open_ns_w);

    // Prepare a pending frame before its release time where possible. Only
    // register inputs depend on the slot arithmetic; no same-cycle bypass.
    // Active configuration changes are atomic and carry cfg_sched_update_pulse.
    assign open_now_w = cfg_enable && cfg_sched_enable && sched_cfg_valid_w &&
                        (!cfg_soft_reset) && (!cfg_sched_update_pulse) &&
                        (!frame_active_r) && (!release_armed_r) && pending_frame_w &&
                        sched_time_usable_w &&
                        ((!cfg_sched_strict_window) || (!current_slot_expired_w));

    assign armed_past_end_w = tod_time_valid && f_gt64(tod_time_ns,armed_slot_end_ns_r);
    assign armed_window_eligible_w = sched_time_usable_w &&
                                     f_ge64(tod_time_ns,armed_release_open_ns_r) &&
                                     ((!cfg_sched_strict_window) || (!armed_past_end_w));

    // Payload cache needs no reset: release_armed_r is reset/invalidated before
    // it can be used. Preparation adds at least one clock, but can run before
    // the window opens. The actual first handshake always uses live ToD.
    always @(posedge clk) begin
        if (open_now_w) begin
            armed_release_open_ns_r <= release_open_ns_w;
            armed_slot_end_ns_r <= slot_end_ns_w;
        end
    end

    assign allow_first_byte_w = (!cfg_enable) ? 1'b0 :
                                (!cfg_sched_enable) ? 1'b1 :
                                (release_armed_r && armed_window_eligible_w &&
                                 !cfg_soft_reset && !cfg_sched_update_pulse);

    assign allow_stream_w = (!cfg_enable) ? 1'b0 :
                            frame_active_r ? 1'b1 : allow_first_byte_w;

    assign m_tdata  = s_tdata;
    assign m_tlast  = s_tlast;
    assign m_tuser  = s_tuser;
    assign m_tvalid = s_tvalid && allow_stream_w;
    assign s_tready = m_tready && allow_stream_w;

    assign first_byte_fire_w = (!frame_active_r) && s_tvalid && s_tready;
    assign any_byte_fire_w   = s_tvalid && s_tready;

    assign plan_time_now_w     = armed_release_open_ns_r;
    assign actual_time_now_w   = tod_time_valid ? tod_time_ns : 64'd0;
    assign actual_minus_plan_w = actual_time_now_w - plan_time_now_w;
    assign late_deadline_ns_w  = f_add64_32(plan_time_now_w, cfg_late_tol_ns);
    assign actual_late_w       = tod_time_valid && (actual_time_now_w > late_deadline_ns_w);
    assign actual_slot_miss_w  = tod_time_valid && f_gt64(actual_time_now_w,armed_slot_end_ns_r);

    assign stat_status =
        (cfg_sched_enable                 ? STATUS_BIT_ENABLE            : 32'd0) |
        (tod_time_valid                   ? STATUS_BIT_TOD_VALID         : 32'd0) |
        (pending_frame_w                  ? STATUS_BIT_PENDING           : 32'd0) |
        (release_armed_r                  ? STATUS_BIT_RELEASE_ARMED     : 32'd0) |
        (frame_active_r                   ? STATUS_BIT_FRAME_ACTIVE      : 32'd0) |
        (sched_cfg_valid_w                ? STATUS_BIT_CFG_VALID         : 32'd0) |
        (cfg_sched_block_when_time_invalid? STATUS_BIT_BLOCK_ON_BAD_TIME : 32'd0) |
        (cfg_sched_strict_window          ? STATUS_BIT_STRICT_WINDOW     : 32'd0) |
        (last_launch_late_r               ? STATUS_BIT_LAST_LATE         : 32'd0) |
        (last_launch_slot_miss_r          ? STATUS_BIT_LAST_SLOT_MISS    : 32'd0) |
        (release_window_open_w            ? STATUS_BIT_WINDOW_OPEN       : 32'd0) |
        ((!cfg_sched_enable)              ? STATUS_BIT_BYPASS            : 32'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_active_r           <= 1'b0;
            release_armed_r          <= 1'b0;
            waiting_accounted_r      <= 1'b0;
            last_launch_late_r       <= 1'b0;
            last_launch_slot_miss_r  <= 1'b0;
            current_slot_start_ns_r  <= 64'd0;
            stat_held_count          <= 32'd0;
            stat_pass_count          <= 32'd0;
            stat_late_count          <= 32'd0;
            stat_slot_miss_count     <= 32'd0;
            stat_last_plan_ns        <= 64'd0;
            stat_last_actual_ns      <= 64'd0;
            stat_last_delta_ns       <= 32'd0;
        end else begin
            if (cfg_clear_status) begin
                stat_held_count      <= 32'd0;
                stat_pass_count      <= 32'd0;
                stat_late_count      <= 32'd0;
                stat_slot_miss_count <= 32'd0;
                stat_last_plan_ns    <= 64'd0;
                stat_last_actual_ns  <= 64'd0;
                stat_last_delta_ns   <= 32'd0;
                last_launch_late_r   <= 1'b0;
                last_launch_slot_miss_r <= 1'b0;
            end

            // Re-initialize the scheduling epoch when the ACTIVE scheduler bank
            // changes (cfg_sched_update_pulse is generated by the ETH safe-commit
            // epoch), or when the whole data path is soft-reset/disabled.
            if (cfg_soft_reset || (!cfg_enable) || cfg_sched_update_pulse) begin
                current_slot_start_ns_r <= f_add64_32(cfg_base_time_ns, cfg_slot_offset_ns);
                frame_active_r          <= 1'b0;
                release_armed_r         <= 1'b0;
                waiting_accounted_r     <= 1'b0;
                last_launch_late_r      <= 1'b0;
                last_launch_slot_miss_r <= 1'b0;
            end else if (!cfg_sched_enable || !sched_cfg_valid_w) begin
                // Scheduler bypass mode: keep status state simple.
                frame_active_r      <= any_byte_fire_w ? (!s_tlast) : frame_active_r;
                release_armed_r     <= 1'b0;
                waiting_accounted_r <= 1'b0;
                if (frame_active_r && any_byte_fire_w && s_tlast)
                    frame_active_r <= 1'b0;
            end else begin
                // No pending first-byte competition: advance slot cursor if the
                // currently tracked slot is already behind current ToD.
                if ((!frame_active_r) && (!release_armed_r) && sched_time_usable_w &&
                    current_slot_expired_w && !open_now_w) begin
                    current_slot_start_ns_r <= slot_start_plus_period_w;
                end

                // When a frame is pending before release is allowed, count one hold.
                if (pending_frame_w && !waiting_accounted_r && !first_byte_fire_w) begin
                    waiting_accounted_r <= 1'b1;
                    stat_held_count     <= stat_held_count + 32'd1;
                end

                // Open gate once the current slot becomes eligible.
                if (open_now_w) begin
                    release_armed_r      <= 1'b1;
                    stat_last_plan_ns    <= release_open_ns_w;
                end

                // If strict_window is enabled and the frame still has not started
                // by the time the current slot expires, close the gate and defer
                // the same pending frame to the next slot.
                if ((!frame_active_r) && release_armed_r && cfg_sched_strict_window &&
                    pending_frame_w && sched_time_usable_w && armed_past_end_w && !first_byte_fire_w) begin
                    release_armed_r         <= 1'b0;
                    current_slot_start_ns_r <= slot_start_plus_period_w;
                    stat_slot_miss_count    <= stat_slot_miss_count + 32'd1;
                    last_launch_slot_miss_r <= 1'b1;
                    // waiting_accounted_r stays asserted: this is still the same frame.
                end

                // First-byte launch bookkeeping.
                if (first_byte_fire_w) begin
                    release_armed_r         <= 1'b0;
                    frame_active_r          <= !s_tlast;
                    waiting_accounted_r     <= 1'b0;
                    // Preparation freezes this slot. A non-strict late launch
                    // consumes it once, then advances exactly one period.
                    if (actual_slot_miss_w && !cfg_sched_strict_window)
                        current_slot_start_ns_r <= slot_start_plus_period_w;
                    stat_pass_count         <= stat_pass_count + 32'd1;
                    stat_last_plan_ns       <= plan_time_now_w;
                    stat_last_actual_ns     <= actual_time_now_w;
                    stat_last_delta_ns      <= actual_minus_plan_w[31:0];
                    last_launch_late_r      <= actual_late_w;
                    last_launch_slot_miss_r <= actual_slot_miss_w;
                    if (actual_late_w)
                        stat_late_count <= stat_late_count + 32'd1;
                    if (actual_slot_miss_w && (!cfg_sched_strict_window))
                        stat_slot_miss_count <= stat_slot_miss_count + 32'd1;
                end else if (frame_active_r && any_byte_fire_w && s_tlast) begin
                    frame_active_r <= 1'b0;
                end

                // If the upstream drops valid before first launch (should not
                // happen with the current backend staging), clear the pending
                // tracking state defensively.
                if ((!frame_active_r) && (!s_tvalid)) begin
                    release_armed_r     <= 1'b0;
                    waiting_accounted_r <= 1'b0;
                end
            end
        end
    end

endmodule
