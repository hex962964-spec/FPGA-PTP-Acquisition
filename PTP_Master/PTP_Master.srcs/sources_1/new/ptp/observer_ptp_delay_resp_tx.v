`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_ptp_delay_resp_tx
// -----------------------------------------------------------------------------
// Purpose:
//   Generate a PTPv2 E2E Delay_Resp general message in response to a parsed DUT
//   Delay_Req record.
//
// Frame model:
//   Output is a complete Ethernet frame body, without preamble/SFD/FCS:
//     Ethernet header + IPv4 header + UDP header + PTP Delay_Resp payload
//   Downstream observer_axis_mac_tx appends preamble/SFD/FCS/IFG.
//////////////////////////////////////////////////////////////////////////////////
module observer_ptp_delay_resp_tx #(
    parameter [47:0] LOCAL_MAC       = 48'h04_CB_01_DA_AA_DA,
    parameter [47:0] DUT_MAC         = 48'h00_0A_35_01_FE_C0,
    parameter [31:0] LOCAL_IP        = 32'hC0A8_0003,
    parameter [31:0] DUT_IP          = 32'hC0A8_000E,
    parameter [63:0] LOCAL_CLOCK_ID  = 64'h04CB_01FF_FEDA_AADA,
    parameter [15:0] LOCAL_PORT_NUM  = 16'd1,
    parameter [7:0]  PTP_DOMAIN      = 8'd0,
    parameter [7:0]  LOG_DELAY_RESP_INTERVAL = 8'h7F
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,

    input  wire        s_record_valid,
    output wire        s_record_ready,
    input  wire [15:0] s_sequence_id,
    input  wire [79:0] s_requesting_port_identity,
    input  wire [63:0] s_t4_ns,
    input  wire [47:0] s_t4_seconds,
    input  wire [31:0] s_t4_nanoseconds,

    output wire [7:0]  m_axis_tdata,
    output wire        m_axis_tvalid,
    output wire        m_axis_tlast,
    input  wire        m_axis_tready,
    output wire [3:0]  m_axis_tuser,

    output wire        busy,
    output reg  [31:0] stat_delay_resp_count,
    output reg  [15:0] stat_last_sequence_id,
    output reg  [63:0] stat_last_t4_ns
);

    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_SEND = 2'd1;

    localparam [3:0] MSG_DELAY_RESP = 4'h9;
    localparam [7:0] FRAME_BYTES    = 8'd96; // 14 + 20 + 8 + 54

    localparam [15:0] UDP_PORT_GENERAL = 16'd320;
    localparam [15:0] IPV4_TOTAL_LEN   = 16'd82;
    localparam [15:0] UDP_LEN          = 16'd62;
    localparam [15:0] PTP_MSG_LEN      = 16'd54;
    localparam [1:0]  FRAME_TYPE_PTP_GENERAL = 2'b10;

    reg [1:0]  state_r;
    reg [7:0]  byte_idx_r;
    reg [15:0] active_sequence_id_r;
    reg [15:0] active_ip_ident_r;
    reg [63:0] active_t4_ns_r;
    reg [47:0] active_t4_seconds_r;
    reg [31:0] active_t4_nanoseconds_r;
    reg [79:0] active_requesting_port_identity_r;

    wire fire_w;
    wire last_byte_w;

    assign fire_w          = m_axis_tvalid && m_axis_tready;
    assign last_byte_w     = (byte_idx_r == (FRAME_BYTES - 8'd1));
    assign s_record_ready  = enable && (state_r == ST_IDLE);
    assign m_axis_tvalid   = (state_r == ST_SEND);
    assign m_axis_tlast    = (state_r == ST_SEND) && last_byte_w;
    assign m_axis_tdata    = f_frame_byte(byte_idx_r,
                                          active_sequence_id_r,
                                          active_ip_ident_r,
                                          active_t4_seconds_r,
                                          active_t4_nanoseconds_r,
                                          active_requesting_port_identity_r);
    assign m_axis_tuser    = ((state_r == ST_SEND) && (byte_idx_r == 8'd0)) ?
                             {FRAME_TYPE_PTP_GENERAL, 1'b0, 1'b1} : 4'd0;
    assign busy            = (state_r != ST_IDLE);

    function [15:0] f_ipv4_checksum;
        input [15:0] ident;
        reg [31:0] sum;
        begin
            sum = 32'd0;
            sum = sum + 16'h4500;
            sum = sum + IPV4_TOTAL_LEN;
            sum = sum + ident;
            sum = sum + 16'h4000;
            sum = sum + 16'h4011;
            sum = sum + LOCAL_IP[31:16];
            sum = sum + LOCAL_IP[15:0];
            sum = sum + DUT_IP[31:16];
            sum = sum + DUT_IP[15:0];
            sum = {16'd0, sum[15:0]} + {16'd0, sum[31:16]};
            sum = {16'd0, sum[15:0]} + {16'd0, sum[31:16]};
            f_ipv4_checksum = ~sum[15:0];
        end
    endfunction

    function [7:0] f_timestamp_byte;
        input [3:0]  pos;
        input [47:0] sec;
        input [31:0] nsec;
        begin
            case (pos)
                4'd0: f_timestamp_byte = sec[47:40];
                4'd1: f_timestamp_byte = sec[39:32];
                4'd2: f_timestamp_byte = sec[31:24];
                4'd3: f_timestamp_byte = sec[23:16];
                4'd4: f_timestamp_byte = sec[15:8];
                4'd5: f_timestamp_byte = sec[7:0];
                4'd6: f_timestamp_byte = nsec[31:24];
                4'd7: f_timestamp_byte = nsec[23:16];
                4'd8: f_timestamp_byte = nsec[15:8];
                default: f_timestamp_byte = nsec[7:0];
            endcase
        end
    endfunction

    function [7:0] f_port_identity_byte;
        input [3:0]  pos;
        input [79:0] port_id;
        begin
            case (pos)
                4'd0: f_port_identity_byte = port_id[79:72];
                4'd1: f_port_identity_byte = port_id[71:64];
                4'd2: f_port_identity_byte = port_id[63:56];
                4'd3: f_port_identity_byte = port_id[55:48];
                4'd4: f_port_identity_byte = port_id[47:40];
                4'd5: f_port_identity_byte = port_id[39:32];
                4'd6: f_port_identity_byte = port_id[31:24];
                4'd7: f_port_identity_byte = port_id[23:16];
                4'd8: f_port_identity_byte = port_id[15:8];
                default: f_port_identity_byte = port_id[7:0];
            endcase
        end
    endfunction

    function [7:0] f_frame_byte;
        input [7:0]  idx;
        input [15:0] seq_id;
        input [15:0] ip_ident;
        input [47:0] t4_sec;
        input [31:0] t4_nsec;
        input [79:0] req_port_id;
        reg [15:0] ip_csum;
        begin
            ip_csum = f_ipv4_checksum(ip_ident);
            case (idx)
                8'd0:  f_frame_byte = DUT_MAC[47:40];
                8'd1:  f_frame_byte = DUT_MAC[39:32];
                8'd2:  f_frame_byte = DUT_MAC[31:24];
                8'd3:  f_frame_byte = DUT_MAC[23:16];
                8'd4:  f_frame_byte = DUT_MAC[15:8];
                8'd5:  f_frame_byte = DUT_MAC[7:0];
                8'd6:  f_frame_byte = LOCAL_MAC[47:40];
                8'd7:  f_frame_byte = LOCAL_MAC[39:32];
                8'd8:  f_frame_byte = LOCAL_MAC[31:24];
                8'd9:  f_frame_byte = LOCAL_MAC[23:16];
                8'd10: f_frame_byte = LOCAL_MAC[15:8];
                8'd11: f_frame_byte = LOCAL_MAC[7:0];
                8'd12: f_frame_byte = 8'h08;
                8'd13: f_frame_byte = 8'h00;

                8'd14: f_frame_byte = 8'h45;
                8'd15: f_frame_byte = 8'h00;
                8'd16: f_frame_byte = IPV4_TOTAL_LEN[15:8];
                8'd17: f_frame_byte = IPV4_TOTAL_LEN[7:0];
                8'd18: f_frame_byte = ip_ident[15:8];
                8'd19: f_frame_byte = ip_ident[7:0];
                8'd20: f_frame_byte = 8'h40;
                8'd21: f_frame_byte = 8'h00;
                8'd22: f_frame_byte = 8'd64;
                8'd23: f_frame_byte = 8'd17;
                8'd24: f_frame_byte = ip_csum[15:8];
                8'd25: f_frame_byte = ip_csum[7:0];
                8'd26: f_frame_byte = LOCAL_IP[31:24];
                8'd27: f_frame_byte = LOCAL_IP[23:16];
                8'd28: f_frame_byte = LOCAL_IP[15:8];
                8'd29: f_frame_byte = LOCAL_IP[7:0];
                8'd30: f_frame_byte = DUT_IP[31:24];
                8'd31: f_frame_byte = DUT_IP[23:16];
                8'd32: f_frame_byte = DUT_IP[15:8];
                8'd33: f_frame_byte = DUT_IP[7:0];

                8'd34: f_frame_byte = UDP_PORT_GENERAL[15:8];
                8'd35: f_frame_byte = UDP_PORT_GENERAL[7:0];
                8'd36: f_frame_byte = UDP_PORT_GENERAL[15:8];
                8'd37: f_frame_byte = UDP_PORT_GENERAL[7:0];
                8'd38: f_frame_byte = UDP_LEN[15:8];
                8'd39: f_frame_byte = UDP_LEN[7:0];
                8'd40: f_frame_byte = 8'h00;
                8'd41: f_frame_byte = 8'h00;

                8'd42: f_frame_byte = {4'h0, MSG_DELAY_RESP};
                8'd43: f_frame_byte = 8'h02;
                8'd44: f_frame_byte = PTP_MSG_LEN[15:8];
                8'd45: f_frame_byte = PTP_MSG_LEN[7:0];
                8'd46: f_frame_byte = PTP_DOMAIN;
                8'd47: f_frame_byte = 8'h00;
                8'd48: f_frame_byte = 8'h00;
                8'd49: f_frame_byte = 8'h00;
                8'd50,8'd51,8'd52,8'd53,8'd54,8'd55,8'd56,8'd57,
                8'd58,8'd59,8'd60,8'd61: f_frame_byte = 8'h00;
                8'd62: f_frame_byte = LOCAL_CLOCK_ID[63:56];
                8'd63: f_frame_byte = LOCAL_CLOCK_ID[55:48];
                8'd64: f_frame_byte = LOCAL_CLOCK_ID[47:40];
                8'd65: f_frame_byte = LOCAL_CLOCK_ID[39:32];
                8'd66: f_frame_byte = LOCAL_CLOCK_ID[31:24];
                8'd67: f_frame_byte = LOCAL_CLOCK_ID[23:16];
                8'd68: f_frame_byte = LOCAL_CLOCK_ID[15:8];
                8'd69: f_frame_byte = LOCAL_CLOCK_ID[7:0];
                8'd70: f_frame_byte = LOCAL_PORT_NUM[15:8];
                8'd71: f_frame_byte = LOCAL_PORT_NUM[7:0];
                8'd72: f_frame_byte = seq_id[15:8];
                8'd73: f_frame_byte = seq_id[7:0];
                8'd74: f_frame_byte = 8'h03;
                8'd75: f_frame_byte = LOG_DELAY_RESP_INTERVAL;

                8'd76: f_frame_byte = f_timestamp_byte(4'd0, t4_sec, t4_nsec);
                8'd77: f_frame_byte = f_timestamp_byte(4'd1, t4_sec, t4_nsec);
                8'd78: f_frame_byte = f_timestamp_byte(4'd2, t4_sec, t4_nsec);
                8'd79: f_frame_byte = f_timestamp_byte(4'd3, t4_sec, t4_nsec);
                8'd80: f_frame_byte = f_timestamp_byte(4'd4, t4_sec, t4_nsec);
                8'd81: f_frame_byte = f_timestamp_byte(4'd5, t4_sec, t4_nsec);
                8'd82: f_frame_byte = f_timestamp_byte(4'd6, t4_sec, t4_nsec);
                8'd83: f_frame_byte = f_timestamp_byte(4'd7, t4_sec, t4_nsec);
                8'd84: f_frame_byte = f_timestamp_byte(4'd8, t4_sec, t4_nsec);
                8'd85: f_frame_byte = f_timestamp_byte(4'd9, t4_sec, t4_nsec);

                8'd86: f_frame_byte = f_port_identity_byte(4'd0, req_port_id);
                8'd87: f_frame_byte = f_port_identity_byte(4'd1, req_port_id);
                8'd88: f_frame_byte = f_port_identity_byte(4'd2, req_port_id);
                8'd89: f_frame_byte = f_port_identity_byte(4'd3, req_port_id);
                8'd90: f_frame_byte = f_port_identity_byte(4'd4, req_port_id);
                8'd91: f_frame_byte = f_port_identity_byte(4'd5, req_port_id);
                8'd92: f_frame_byte = f_port_identity_byte(4'd6, req_port_id);
                8'd93: f_frame_byte = f_port_identity_byte(4'd7, req_port_id);
                8'd94: f_frame_byte = f_port_identity_byte(4'd8, req_port_id);
                default: f_frame_byte = f_port_identity_byte(4'd9, req_port_id);
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r                            <= ST_IDLE;
            byte_idx_r                        <= 8'd0;
            active_sequence_id_r              <= 16'd0;
            active_ip_ident_r                 <= 16'hD000;
            active_t4_ns_r                    <= 64'd0;
            active_t4_seconds_r               <= 48'd0;
            active_t4_nanoseconds_r           <= 32'd0;
            active_requesting_port_identity_r <= 80'd0;
            stat_delay_resp_count             <= 32'd0;
            stat_last_sequence_id             <= 16'd0;
            stat_last_t4_ns                   <= 64'd0;
        end else begin
            if (!enable) begin
                state_r    <= ST_IDLE;
                byte_idx_r <= 8'd0;
            end else begin
                case (state_r)
                    ST_IDLE: begin
                        byte_idx_r <= 8'd0;
                        if (s_record_valid && s_record_ready) begin
                            active_sequence_id_r              <= s_sequence_id;
                            active_t4_ns_r                    <= s_t4_ns;
                            active_t4_seconds_r               <= s_t4_seconds;
                            active_t4_nanoseconds_r           <= s_t4_nanoseconds;
                            active_requesting_port_identity_r <= s_requesting_port_identity;
                            active_ip_ident_r                 <= s_sequence_id ^ 16'hD000;
                            stat_last_sequence_id             <= s_sequence_id;
                            stat_last_t4_ns                   <= s_t4_ns;
                            state_r                           <= ST_SEND;
                        end
                    end

                    ST_SEND: begin
                        if (fire_w) begin
                            if (last_byte_w) begin
                                stat_delay_resp_count <= stat_delay_resp_count + 32'd1;
                                state_r               <= ST_IDLE;
                                byte_idx_r            <= 8'd0;
                            end else begin
                                byte_idx_r <= byte_idx_r + 8'd1;
                            end
                        end
                    end

                    default: begin
                        state_r <= ST_IDLE;
                    end
                endcase
            end
        end
    end

    // Keep flat-ns t4 visible to debug/optimization and avoid over-aggressive trimming.
    wire unused_keep_w;
    assign unused_keep_w = active_t4_ns_r[0];

endmodule
`default_nettype wire
