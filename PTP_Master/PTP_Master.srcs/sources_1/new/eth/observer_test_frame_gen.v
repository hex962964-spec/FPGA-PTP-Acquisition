`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_test_frame_gen
// -----------------------------------------------------------------------------
// Purpose:
//   Generate a small periodic Ethernet debug frame for RGMII TX bring-up.
//   This is a foundation-stage traffic source only.  Future PTP master TX logic
//   can replace it or share the TX mux.
//
// Frame format:
//   DST MAC  : parameter DEST_MAC, default broadcast
//   SRC MAC  : parameter LOCAL_MAC
//   EtherType: 0x88B5, locally used debug type
//   Payload  : "OBS-FND1" + frame_counter + tod_ns[31:0]
//
// Notes:
//   - The MAC TX block will add preamble, padding, FCS and IFG.
//   - tuser[0] = SOF on the first byte.
//////////////////////////////////////////////////////////////////////////////////
module observer_test_frame_gen #(
    parameter [47:0] LOCAL_MAC       = 48'h04_CB_01_DA_AA_DA,
    parameter [47:0] DEST_MAC        = 48'hFF_FF_FF_FF_FF_FF,
    parameter [31:0] INTERVAL_CYCLES = 32'd125000000,
    parameter        ENABLE_DEFAULT  = 1'b1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire [63:0] tod_ns,

    output reg  [7:0]  m_axis_tdata,
    output reg         m_axis_tvalid,
    output reg         m_axis_tlast,
    input  wire        m_axis_tready,
    output reg  [3:0]  m_axis_tuser,

    output reg  [31:0] stat_frame_count
);

    localparam [7:0] ETH_TYPE_H = 8'h88;
    localparam [7:0] ETH_TYPE_L = 8'hB5;
    localparam [7:0] FRAME_BYTES = 8'd46; // 14-byte header + 32-byte payload

    localparam [1:0] ST_WAIT = 2'd0;
    localparam [1:0] ST_SEND = 2'd1;

    reg [1:0]  state_r;
    reg [31:0] interval_cnt_r;
    reg [7:0]  byte_idx_r;
    reg [31:0] frame_counter_r;
    reg [31:0] tod_latched_r;

    wire fire_w;
    wire start_due_w;

    assign fire_w = m_axis_tvalid && m_axis_tready;
    assign start_due_w = enable && ENABLE_DEFAULT && (interval_cnt_r == 32'd0);

    function [7:0] frame_byte;
        input [7:0]  idx;
        input [31:0] frame_counter;
        input [31:0] tod_low;
        begin
            case (idx)
                // Destination MAC, wire order MSB first.
                8'd0:  frame_byte = DEST_MAC[47:40];
                8'd1:  frame_byte = DEST_MAC[39:32];
                8'd2:  frame_byte = DEST_MAC[31:24];
                8'd3:  frame_byte = DEST_MAC[23:16];
                8'd4:  frame_byte = DEST_MAC[15:8];
                8'd5:  frame_byte = DEST_MAC[7:0];
                // Source MAC.
                8'd6:  frame_byte = LOCAL_MAC[47:40];
                8'd7:  frame_byte = LOCAL_MAC[39:32];
                8'd8:  frame_byte = LOCAL_MAC[31:24];
                8'd9:  frame_byte = LOCAL_MAC[23:16];
                8'd10: frame_byte = LOCAL_MAC[15:8];
                8'd11: frame_byte = LOCAL_MAC[7:0];
                // EtherType.
                8'd12: frame_byte = ETH_TYPE_H;
                8'd13: frame_byte = ETH_TYPE_L;
                // Payload ASCII "OBS-FND1".
                8'd14: frame_byte = 8'h4F; // O
                8'd15: frame_byte = 8'h42; // B
                8'd16: frame_byte = 8'h53; // S
                8'd17: frame_byte = 8'h2D; // -
                8'd18: frame_byte = 8'h46; // F
                8'd19: frame_byte = 8'h4E; // N
                8'd20: frame_byte = 8'h44; // D
                8'd21: frame_byte = 8'h31; // 1
                // frame_counter, big-endian.
                8'd22: frame_byte = frame_counter[31:24];
                8'd23: frame_byte = frame_counter[23:16];
                8'd24: frame_byte = frame_counter[15:8];
                8'd25: frame_byte = frame_counter[7:0];
                // tod_low, big-endian.
                8'd26: frame_byte = tod_low[31:24];
                8'd27: frame_byte = tod_low[23:16];
                8'd28: frame_byte = tod_low[15:8];
                8'd29: frame_byte = tod_low[7:0];
                default: frame_byte = idx; // deterministic padding payload
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r          <= ST_WAIT;
            interval_cnt_r   <= 32'd0;
            byte_idx_r       <= 8'd0;
            frame_counter_r  <= 32'd0;
            tod_latched_r    <= 32'd0;
            m_axis_tdata     <= 8'd0;
            m_axis_tvalid    <= 1'b0;
            m_axis_tlast     <= 1'b0;
            m_axis_tuser     <= 4'd0;
            stat_frame_count <= 32'd0;
        end else begin
            case (state_r)
                ST_WAIT: begin
                    m_axis_tvalid <= 1'b0;
                    m_axis_tlast  <= 1'b0;
                    m_axis_tuser  <= 4'd0;

                    if (!enable) begin
                        interval_cnt_r <= 32'd0;
                    end else if (start_due_w) begin
                        byte_idx_r      <= 8'd0;
                        tod_latched_r   <= tod_ns[31:0];
                        m_axis_tdata    <= frame_byte(8'd0, frame_counter_r, tod_ns[31:0]);
                        m_axis_tvalid   <= 1'b1;
                        m_axis_tlast    <= (FRAME_BYTES == 8'd1);
                        m_axis_tuser    <= 4'b0001; // SOF
                        state_r         <= ST_SEND;
                    end else if (interval_cnt_r == 32'd0) begin
                        interval_cnt_r <= INTERVAL_CYCLES - 32'd1;
                    end else begin
                        interval_cnt_r <= interval_cnt_r - 32'd1;
                    end
                end

                ST_SEND: begin
                    if (fire_w) begin
                        if (byte_idx_r == (FRAME_BYTES - 8'd1)) begin
                            m_axis_tvalid    <= 1'b0;
                            m_axis_tlast     <= 1'b0;
                            m_axis_tuser     <= 4'd0;
                            frame_counter_r  <= frame_counter_r + 32'd1;
                            stat_frame_count <= stat_frame_count + 32'd1;
                            interval_cnt_r   <= INTERVAL_CYCLES - 32'd1;
                            state_r          <= ST_WAIT;
                        end else begin
                            byte_idx_r   <= byte_idx_r + 8'd1;
                            m_axis_tdata <= frame_byte(byte_idx_r + 8'd1, frame_counter_r, tod_latched_r);
                            m_axis_tlast <= ((byte_idx_r + 8'd1) == (FRAME_BYTES - 8'd1));
                            m_axis_tuser <= 4'd0;
                        end
                    end
                end

                default: begin
                    state_r <= ST_WAIT;
                end
            endcase
        end
    end

endmodule
`default_nettype wire
