`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_tx_mux
// -----------------------------------------------------------------------------
// Purpose:
//   Frame-aware 3:1 TX mux for the Observer FPGA.
//
// Priority:
//   s0: Delay_Resp responder traffic, highest priority between frames.
//   s1: Periodic PTP master Sync/Follow_Up traffic.
//   s2: Debug/test traffic, lowest priority.
//
// Once a frame starts, the selected source is held until its tlast byte is
// accepted.  No frame-level interleaving is allowed.  The downstream MAC TX block
// is responsible for preamble/SFD/FCS/IFG.
//
// tuser convention:
//   bit0   = SOF
//   bit1   = need_timestamp
//   bit3:2 = frame_type, 2'b01 means PTP event, 2'b10 means PTP general
//////////////////////////////////////////////////////////////////////////////////
module observer_tx_mux(
    input  wire       clk,
    input  wire       rst_n,

    // Highest-priority Delay_Resp source.
    input  wire [7:0] s0_tdata,
    input  wire       s0_tvalid,
    input  wire       s0_tlast,
    output wire       s0_tready,
    input  wire [3:0] s0_tuser,

    // Periodic PTP Sync/Follow_Up source.
    input  wire [7:0] s1_tdata,
    input  wire       s1_tvalid,
    input  wire       s1_tlast,
    output wire       s1_tready,
    input  wire [3:0] s1_tuser,

    // Low-priority debug/test source.
    input  wire [7:0] s2_tdata,
    input  wire       s2_tvalid,
    input  wire       s2_tlast,
    output wire       s2_tready,
    input  wire [3:0] s2_tuser,

    // Output to MAC TX.
    output reg  [7:0] m_tdata,
    output reg        m_tvalid,
    output reg        m_tlast,
    input  wire       m_tready,
    output reg  [3:0] m_tuser,

    output wire       busy,
    output reg  [31:0] stat_delay_resp_frame_count,
    output reg  [31:0] stat_ptp_frame_count,
    output reg  [31:0] stat_debug_frame_count
);

    localparam [1:0] SEL_NONE = 2'd0;
    localparam [1:0] SEL_DRESP= 2'd1;
    localparam [1:0] SEL_PTP  = 2'd2;
    localparam [1:0] SEL_DBG  = 2'd3;

    reg [1:0] sel_r;

    wire idle_w;
    wire out_fire_w;
    wire choose_dresp_w;
    wire choose_ptp_w;
    wire choose_dbg_w;
    wire [1:0] active_sel_w;

    assign idle_w         = (sel_r == SEL_NONE);
    assign out_fire_w     = m_tvalid && m_tready;
    assign choose_dresp_w = idle_w && s0_tvalid;
    assign choose_ptp_w   = idle_w && !s0_tvalid && s1_tvalid;
    assign choose_dbg_w   = idle_w && !s0_tvalid && !s1_tvalid && s2_tvalid;

    assign active_sel_w = choose_dresp_w ? SEL_DRESP :
                          choose_ptp_w   ? SEL_PTP   :
                          choose_dbg_w   ? SEL_DBG   : sel_r;

    assign s0_tready = (active_sel_w == SEL_DRESP) && m_tready;
    assign s1_tready = (active_sel_w == SEL_PTP)   && m_tready;
    assign s2_tready = (active_sel_w == SEL_DBG)   && m_tready;
    assign busy      = (active_sel_w != SEL_NONE);

    always @(*) begin
        case (active_sel_w)
            SEL_DRESP: begin
                m_tdata  = s0_tdata;
                m_tvalid = s0_tvalid;
                m_tlast  = s0_tlast;
                m_tuser  = s0_tuser;
            end
            SEL_PTP: begin
                m_tdata  = s1_tdata;
                m_tvalid = s1_tvalid;
                m_tlast  = s1_tlast;
                m_tuser  = s1_tuser;
            end
            SEL_DBG: begin
                m_tdata  = s2_tdata;
                m_tvalid = s2_tvalid;
                m_tlast  = s2_tlast;
                m_tuser  = s2_tuser;
            end
            default: begin
                m_tdata  = 8'd0;
                m_tvalid = 1'b0;
                m_tlast  = 1'b0;
                m_tuser  = 4'd0;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sel_r                       <= SEL_NONE;
            stat_delay_resp_frame_count <= 32'd0;
            stat_ptp_frame_count        <= 32'd0;
            stat_debug_frame_count      <= 32'd0;
        end else begin
            if (idle_w) begin
                if (s0_tvalid)
                    sel_r <= SEL_DRESP;
                else if (s1_tvalid)
                    sel_r <= SEL_PTP;
                else if (s2_tvalid)
                    sel_r <= SEL_DBG;
            end

            if (out_fire_w && m_tlast) begin
                if (active_sel_w == SEL_DRESP)
                    stat_delay_resp_frame_count <= stat_delay_resp_frame_count + 32'd1;
                else if (active_sel_w == SEL_PTP)
                    stat_ptp_frame_count <= stat_ptp_frame_count + 32'd1;
                else if (active_sel_w == SEL_DBG)
                    stat_debug_frame_count <= stat_debug_frame_count + 32'd1;
                sel_r <= SEL_NONE;
            end
        end
    end

endmodule
`default_nettype wire
