`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_axis_mac_rx
// -----------------------------------------------------------------------------
// Purpose:
//   1) Receive 8-bit GMII-like RX data from rgmii_rx_if.
//   2) Strip 7x55 + D5 preamble/SFD.
//   3) Strip the trailing 4-byte FCS.
//   4) Output a board-independent MAC-side RX byte stream.
//   5) Timestamp the start of every well-formed frame at SFD.
//
// Architecture notes:
//   - This block is now a pure L2/L1 boundary helper. It no longer knows about
//     UDP port 319 or any PTP message semantics.
//   - The downstream ptp_rx_filter is responsible for deciding whether the frame
//     is Sync / Follow_Up / Delay_Resp and whether the SFD timestamp is needed.
//   - The RX timestamp output keeps the historical port names
//     (ptp_rxstamp_ns/valid) to minimize integration changes, but the semantic is
//     now: "current frame SFD timestamp".
//
// Timing note:
//   - rgmii_rx_if introduces one byte assembly latency. To move the timestamp
//     closer to the line-side SFD instant, a fixed 8 ns compensation is applied
//     when ptp_tod_time_valid is high.
//////////////////////////////////////////////////////////////////////////////////
module observer_axis_mac_rx(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         rx_enable,

    input  wire [7:0]   gmii_rxd,
    input  wire         gmii_rx_dv,
    input  wire         gmii_rx_er,

    input  wire [63:0]  ptp_tod_time_ns,
    input  wire [47:0]  ptp_tod_seconds,
    input  wire [31:0]  ptp_tod_nanoseconds,
    input  wire         ptp_tod_time_valid,

    output reg  [7:0]   m_axis_tdata,
    output reg          m_axis_tvalid,
    output reg          m_axis_tlast,

    output reg  [63:0]  ptp_rxstamp_ns,
    output reg  [47:0]  ptp_rxstamp_seconds,
    output reg  [31:0]  ptp_rxstamp_nanoseconds,
    output reg          ptp_rxstamp_valid,

    output wire         rx_busy,
    output reg  [31:0]  stat_frame_count,
    output reg  [31:0]  stat_drop_count,
    output reg  [15:0]  stat_last_error
);

    localparam [1:0] ST_IDLE     = 2'd0;
    localparam [1:0] ST_PREAMBLE = 2'd1;
    localparam [1:0] ST_FRAME    = 2'd2;
    localparam [1:0] ST_DROP     = 2'd3;

    localparam [15:0] ERR_NONE         = 16'h0000;
    localparam [15:0] ERR_BAD_PREAMBLE = 16'h0001;
    localparam [15:0] ERR_RX_ER        = 16'h0002;
    localparam [15:0] ERR_SHORT_FRAME  = 16'h0003;

    reg [1:0]  state_r;
    reg [2:0]  preamble_cnt_r;

    reg [7:0]  hist0_r;
    reg [7:0]  hist1_r;
    reg [7:0]  hist2_r;
    reg [7:0]  hist3_r;
    reg [15:0] frame_byte_count_r;

    reg        pending_valid_r;
    reg [7:0]  pending_data_r;

    assign rx_busy = (state_r != ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r            <= ST_IDLE;
            preamble_cnt_r     <= 3'd0;
            hist0_r            <= 8'd0;
            hist1_r            <= 8'd0;
            hist2_r            <= 8'd0;
            hist3_r            <= 8'd0;
            frame_byte_count_r <= 16'd0;
            pending_valid_r    <= 1'b0;
            pending_data_r     <= 8'd0;

            m_axis_tdata       <= 8'd0;
            m_axis_tvalid      <= 1'b0;
            m_axis_tlast       <= 1'b0;
            ptp_rxstamp_ns          <= 64'd0;
            ptp_rxstamp_seconds     <= 48'd0;
            ptp_rxstamp_nanoseconds <= 32'd0;
            ptp_rxstamp_valid       <= 1'b0;

            stat_frame_count   <= 32'd0;
            stat_drop_count    <= 32'd0;
            stat_last_error    <= ERR_NONE;
        end else begin
            m_axis_tdata      <= 8'd0;
            m_axis_tvalid     <= 1'b0;
            m_axis_tlast      <= 1'b0;
            ptp_rxstamp_valid <= 1'b0;

            if (!rx_enable) begin
                state_r            <= ST_IDLE;
                preamble_cnt_r     <= 3'd0;
                hist0_r            <= 8'd0;
                hist1_r            <= 8'd0;
                hist2_r            <= 8'd0;
                hist3_r            <= 8'd0;
                frame_byte_count_r <= 16'd0;
                pending_valid_r    <= 1'b0;
                pending_data_r     <= 8'd0;
            end else begin
                case (state_r)
                    ST_IDLE: begin
                        if (gmii_rx_dv) begin
                            if (gmii_rxd == 8'h55) begin
                                state_r         <= ST_PREAMBLE;
                                preamble_cnt_r  <= 3'd1;
                                stat_last_error <= ERR_NONE;
                            end else begin
                                state_r         <= ST_DROP;
                                stat_drop_count <= stat_drop_count + 32'd1;
                                stat_last_error <= ERR_BAD_PREAMBLE;
                            end
                        end
                    end

                    ST_PREAMBLE: begin
                        if (!gmii_rx_dv) begin
                            state_r         <= ST_IDLE;
                            preamble_cnt_r  <= 3'd0;
                            stat_drop_count <= stat_drop_count + 32'd1;
                            stat_last_error <= ERR_BAD_PREAMBLE;
                        end else if (preamble_cnt_r < 3'd7) begin
                            if (gmii_rxd == 8'h55) begin
                                preamble_cnt_r <= preamble_cnt_r + 3'd1;
                            end else begin
                                state_r         <= ST_DROP;
                                stat_drop_count <= stat_drop_count + 32'd1;
                                stat_last_error <= ERR_BAD_PREAMBLE;
                            end
                        end else begin
                            if (gmii_rxd == 8'hD5) begin
                                state_r            <= ST_FRAME;
                                frame_byte_count_r <= 16'd0;
                                pending_valid_r    <= 1'b0;
                                pending_data_r     <= 8'd0;
                                hist0_r            <= 8'd0;
                                hist1_r            <= 8'd0;
                                hist2_r            <= 8'd0;
                                hist3_r            <= 8'd0;

                                if (ptp_tod_time_valid) begin
                                    // Keep flat-ns and PTP seconds/nanoseconds views
                                    // consistent with the fixed 8 ns line-side SFD
                                    // compensation used by this RX boundary.
                                    ptp_rxstamp_ns <= (ptp_tod_time_ns >= 64'd8) ?
                                                       (ptp_tod_time_ns - 64'd8) : 64'd0;
                                    if (ptp_tod_time_ns < 64'd8) begin
                                        ptp_rxstamp_seconds     <= 48'd0;
                                        ptp_rxstamp_nanoseconds <= 32'd0;
                                    end else if (ptp_tod_nanoseconds >= 32'd8) begin
                                        ptp_rxstamp_seconds     <= ptp_tod_seconds;
                                        ptp_rxstamp_nanoseconds <= ptp_tod_nanoseconds - 32'd8;
                                    end else begin
                                        ptp_rxstamp_seconds     <= ptp_tod_seconds - 48'd1;
                                        ptp_rxstamp_nanoseconds <= ptp_tod_nanoseconds + 32'd999999992;
                                    end
                                    ptp_rxstamp_valid <= 1'b1;
                                end
                            end else begin
                                state_r         <= ST_DROP;
                                stat_drop_count <= stat_drop_count + 32'd1;
                                stat_last_error <= ERR_BAD_PREAMBLE;
                            end
                        end
                    end

                    ST_FRAME: begin
                        if (gmii_rx_er) begin
                            state_r         <= ST_DROP;
                            pending_valid_r <= 1'b0;
                            stat_drop_count <= stat_drop_count + 32'd1;
                            stat_last_error <= ERR_RX_ER;
                        end else if (gmii_rx_dv) begin
                            if (frame_byte_count_r >= 16'd4) begin
                                if (pending_valid_r) begin
                                    m_axis_tdata  <= pending_data_r;
                                    m_axis_tvalid <= 1'b1;
                                    m_axis_tlast  <= 1'b0;
                                end
                                pending_data_r  <= hist0_r;
                                pending_valid_r <= 1'b1;
                            end

                            hist0_r <= hist1_r;
                            hist1_r <= hist2_r;
                            hist2_r <= hist3_r;
                            hist3_r <= gmii_rxd;

                            frame_byte_count_r <= frame_byte_count_r + 16'd1;
                        end else begin
                            if (pending_valid_r) begin
                                m_axis_tdata     <= pending_data_r;
                                m_axis_tvalid    <= 1'b1;
                                m_axis_tlast     <= 1'b1;
                                pending_valid_r  <= 1'b0;
                                stat_frame_count <= stat_frame_count + 32'd1;
                                stat_last_error  <= ERR_NONE;
                            end else begin
                                stat_drop_count <= stat_drop_count + 32'd1;
                                stat_last_error <= ERR_SHORT_FRAME;
                            end

                            state_r            <= ST_IDLE;
                            frame_byte_count_r <= 16'd0;
                        end
                    end

                    ST_DROP: begin
                        if (!gmii_rx_dv)
                            state_r <= ST_IDLE;
                    end

                    default: begin
                        state_r <= ST_IDLE;
                    end
                endcase
            end
        end
    end

endmodule

`default_nettype wire
