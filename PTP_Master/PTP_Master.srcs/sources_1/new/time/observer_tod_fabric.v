`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_tod_fabric
// -----------------------------------------------------------------------------
// Purpose:
//   Unified time-of-day fabric for the Observer FPGA.
//
// Design intent:
//   The Observer board uses eth_tx_clk as the master PTP time domain because
//   Sync TX timestamps are generated at the MAC TX/SFD boundary.  The RX domain
//   still needs a local ToD for future Delay_Req and sample-frame RX timestamps.
//   This module keeps the RX ToD in the same epoch by periodically copying a
//   stable TX-domain ToD snapshot into the RX domain through a req/ack CDC
//   handshake.
//
// Why this is needed:
//   The earlier bring-up build instantiated one free-running ToD counter in TX
//   and another free-running ToD counter in RX.  That is fine for simple frame
//   counting, but it is not suitable for PTP Delay_Resp generation because t1
//   and t4 must be expressed in the same master time scale.
//
// CDC policy:
//   - TX holds a ToD snapshot stable in tx_hold_* registers.
//   - TX toggles tx_req_toggle_r and waits for rx_ack_toggle_r.
//   - RX synchronizes the request toggle, samples the stable hold bank, loads
//     the RX counter, and toggles ACK.
//   - The multi-bit hold bank is stable while the request is in flight.
//
// Limitations:
//   This is an epoch-alignment fabric, not a phase-measurement servo.  It is
//   sufficient for bring-up and for keeping RX timestamps in the same nominal
//   Observer master time scale.  If sub-ns or calibrated RX timestamp accuracy
//   is needed later, add a fixed compensation term and/or an RX-clock calibration
//   path before using t4 as a metrology-grade timestamp.
//////////////////////////////////////////////////////////////////////////////////
module observer_tod_fabric #(
    parameter [31:0] STEP_NS              = 32'd8,
    parameter [31:0] ONE_SECOND_NS        = 32'd1000000000,
    parameter [31:0] RX_SYNC_INTERVAL_CYCLES = 32'd125000,
    // Max TX wait before re-issuing an RX snapshot request.  This recovers from
    // req/ack-toggle deadlock after RX clock reset/relock while a request is in
    // flight.  Default is 10 ms at 125 MHz.
    parameter [31:0] RX_ACK_TIMEOUT_CYCLES = 32'd1250000,
    parameter [31:0] RX_LOAD_COMP_NS      = 32'd0
)(
    input  wire        tx_clk,
    input  wire        tx_rst_n,
    input  wire        rx_clk,
    input  wire        rx_rst_n,
    input  wire        enable,
    input  wire        clear_pulse_tx,

    output wire [63:0] tx_tod_ns,
    output wire [47:0] tx_tod_seconds,
    output wire [31:0] tx_tod_nanoseconds,
    output wire        tx_tod_valid,
    output wire        tx_pps_pulse,

    output reg  [63:0] rx_tod_ns,
    output reg  [47:0] rx_tod_seconds,
    output reg  [31:0] rx_tod_nanoseconds,
    output reg         rx_tod_valid,
    output reg         rx_pps_pulse,

    output reg         rx_sync_pending,
    output reg  [31:0] rx_sync_count,
    output reg  [31:0] rx_sync_timeout_count
);

    // -------------------------------------------------------------------------
    // TX-domain master ToD.
    // -------------------------------------------------------------------------
    observer_tod_counter #(
        .STEP_NS       (STEP_NS),
        .ONE_SECOND_NS (ONE_SECOND_NS)
    ) u_tx_tod_counter (
        .clk             (tx_clk),
        .rst_n           (tx_rst_n),
        .enable          (enable),
        .clear_pulse     (clear_pulse_tx),
        .tod_ns          (tx_tod_ns),
        .tod_seconds     (tx_tod_seconds),
        .tod_nanoseconds (tx_tod_nanoseconds),
        .tod_valid       (tx_tod_valid),
        .pps_pulse       (tx_pps_pulse)
    );

    // -------------------------------------------------------------------------
    // TX -> RX epoch snapshot request.
    // -------------------------------------------------------------------------
    reg [63:0] tx_hold_ns_r;
    reg [47:0] tx_hold_seconds_r;
    reg [31:0] tx_hold_nanoseconds_r;
    reg        tx_req_toggle_r;
    reg [31:0] tx_sync_timer_r;
    reg [31:0] tx_ack_timeout_r;
    reg        tx_initial_sync_done_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg tx_ack_sync0_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg tx_ack_sync1_r;
    reg tx_ack_seen_r;

    wire tx_ack_event_w;
    assign tx_ack_event_w = (tx_ack_sync1_r != tx_ack_seen_r);

    wire tx_sync_timer_expired_w;
    assign tx_sync_timer_expired_w = (tx_sync_timer_r >= (RX_SYNC_INTERVAL_CYCLES - 32'd1));

    wire tx_ack_timeout_expired_w;
    assign tx_ack_timeout_expired_w = (tx_ack_timeout_r >= (RX_ACK_TIMEOUT_CYCLES - 32'd1));

    always @(posedge tx_clk or negedge tx_rst_n) begin
        if (!tx_rst_n) begin
            tx_hold_ns_r            <= 64'd0;
            tx_hold_seconds_r       <= 48'd0;
            tx_hold_nanoseconds_r   <= 32'd0;
            tx_req_toggle_r         <= 1'b0;
            tx_sync_timer_r         <= 32'd0;
            tx_ack_timeout_r        <= 32'd0;
            tx_initial_sync_done_r  <= 1'b0;
            rx_sync_pending         <= 1'b0;
            rx_sync_count           <= 32'd0;
            rx_sync_timeout_count   <= 32'd0;
            tx_ack_sync0_r          <= 1'b0;
            tx_ack_sync1_r          <= 1'b0;
            tx_ack_seen_r           <= 1'b0;
        end else begin
            tx_ack_sync0_r <= rx_ack_toggle_r;
            tx_ack_sync1_r <= tx_ack_sync0_r;

            if (!enable || clear_pulse_tx) begin
                tx_sync_timer_r        <= 32'd0;
                tx_ack_timeout_r       <= 32'd0;
                tx_initial_sync_done_r <= 1'b0;
                rx_sync_pending        <= 1'b0;
                tx_ack_seen_r          <= tx_ack_sync1_r;
            end else begin
                if (tx_ack_event_w) begin
                    // Always consume the synchronized ACK edge, but only count
                    // it as a completed snapshot when a request is actually in
                    // flight.  RX reset can move ack_toggle while TX is idle;
                    // treating that as a completed sync would mask recovery.
                    tx_ack_seen_r <= tx_ack_sync1_r;
                    if (rx_sync_pending) begin
                        rx_sync_pending        <= 1'b0;
                        rx_sync_count          <= rx_sync_count + 32'd1;
                        tx_sync_timer_r        <= 32'd0;
                        tx_ack_timeout_r       <= 32'd0;
                        tx_initial_sync_done_r <= 1'b1;
                    end
                end else if (rx_sync_pending) begin
                    if (tx_ack_timeout_expired_w && tx_tod_valid) begin
                        // Re-issue a fresh stable snapshot.  If the recovered RX
                        // clock reset/relocked while the previous request was in
                        // flight, RX may no longer see the original toggle edge.
                        // This bounded retry makes rx_tod_valid self-healing.
                        tx_hold_ns_r          <= tx_tod_ns;
                        tx_hold_seconds_r     <= tx_tod_seconds;
                        tx_hold_nanoseconds_r <= tx_tod_nanoseconds;
                        tx_req_toggle_r       <= ~tx_req_toggle_r;
                        tx_ack_timeout_r      <= 32'd0;
                        rx_sync_timeout_count <= rx_sync_timeout_count + 32'd1;
                    end else begin
                        tx_ack_timeout_r <= tx_ack_timeout_r + 32'd1;
                    end
                end else if (tx_tod_valid) begin
                    if (!tx_initial_sync_done_r || tx_sync_timer_expired_w) begin
                        // Launch a new stable snapshot to RX.
                        tx_hold_ns_r          <= tx_tod_ns;
                        tx_hold_seconds_r     <= tx_tod_seconds;
                        tx_hold_nanoseconds_r <= tx_tod_nanoseconds;
                        tx_req_toggle_r       <= ~tx_req_toggle_r;
                        rx_sync_pending       <= 1'b1;
                        tx_ack_timeout_r      <= 32'd0;
                    end else begin
                        tx_sync_timer_r <= tx_sync_timer_r + 32'd1;
                    end
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // RX-domain loadable ToD counter.  It is invalid until the first TX snapshot
    // arrives, so RX timestamps are not reported before the epochs are aligned.
    // -------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg rx_req_sync0_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg rx_req_sync1_r;
    reg rx_req_seen_r;
    reg rx_ack_toggle_r;

    wire [32:0] rx_next_nsec_sum_w;
    wire        rx_next_nsec_wrap_w;
    wire [32:0] rx_next_nsec_sub_w;
    wire [31:0] rx_next_nsec_w;

    wire [32:0] rx_load_nsec_sum_w;
    wire        rx_load_nsec_wrap_w;
    wire [32:0] rx_load_nsec_sub_w;
    wire [31:0] rx_load_nsec_w;
    wire [47:0] rx_load_sec_w;
    wire [63:0] rx_load_ns_w;

    assign rx_next_nsec_sum_w  = {1'b0, rx_tod_nanoseconds} + {1'b0, STEP_NS};
    assign rx_next_nsec_wrap_w = (rx_next_nsec_sum_w >= {1'b0, ONE_SECOND_NS});
    assign rx_next_nsec_sub_w  = rx_next_nsec_sum_w - {1'b0, ONE_SECOND_NS};
    assign rx_next_nsec_w      = rx_next_nsec_wrap_w ? rx_next_nsec_sub_w[31:0] : rx_next_nsec_sum_w[31:0];

    assign rx_load_nsec_sum_w  = {1'b0, tx_hold_nanoseconds_r} + {1'b0, RX_LOAD_COMP_NS};
    assign rx_load_nsec_wrap_w = (rx_load_nsec_sum_w >= {1'b0, ONE_SECOND_NS});
    assign rx_load_nsec_sub_w  = rx_load_nsec_sum_w - {1'b0, ONE_SECOND_NS};
    assign rx_load_nsec_w      = rx_load_nsec_wrap_w ? rx_load_nsec_sub_w[31:0] : rx_load_nsec_sum_w[31:0];
    assign rx_load_sec_w       = tx_hold_seconds_r + {47'd0, rx_load_nsec_wrap_w};
    assign rx_load_ns_w        = tx_hold_ns_r + {32'd0, RX_LOAD_COMP_NS};

    always @(posedge rx_clk or negedge rx_rst_n) begin
        if (!rx_rst_n) begin
            rx_req_sync0_r      <= 1'b0;
            rx_req_sync1_r      <= 1'b0;
            rx_req_seen_r       <= 1'b0;
            rx_ack_toggle_r     <= 1'b0;
            rx_tod_ns           <= 64'd0;
            rx_tod_seconds      <= 48'd0;
            rx_tod_nanoseconds  <= 32'd0;
            rx_tod_valid        <= 1'b0;
            rx_pps_pulse        <= 1'b0;
        end else begin
            rx_pps_pulse   <= 1'b0;
            rx_req_sync0_r <= tx_req_toggle_r;
            rx_req_sync1_r <= rx_req_sync0_r;

            if (!enable) begin
                rx_tod_valid <= 1'b0;
            end else if (rx_req_sync1_r != rx_req_seen_r) begin
                rx_req_seen_r      <= rx_req_sync1_r;
                rx_tod_ns          <= rx_load_ns_w;
                rx_tod_seconds     <= rx_load_sec_w;
                rx_tod_nanoseconds <= rx_load_nsec_w;
                rx_tod_valid       <= 1'b1;
                rx_ack_toggle_r    <= ~rx_ack_toggle_r;
            end else if (rx_tod_valid) begin
                rx_tod_ns          <= rx_tod_ns + {32'd0, STEP_NS};
                rx_tod_nanoseconds <= rx_next_nsec_w;
                if (rx_next_nsec_wrap_w) begin
                    rx_tod_seconds <= rx_tod_seconds + 48'd1;
                    rx_pps_pulse   <= 1'b1;
                end
            end
        end
    end

endmodule
`default_nettype wire
