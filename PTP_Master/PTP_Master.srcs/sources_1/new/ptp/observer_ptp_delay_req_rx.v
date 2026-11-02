`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_ptp_delay_req_rx
// -----------------------------------------------------------------------------
// Purpose:
//   RX-domain parser for DUT PTPv2 Delay_Req frames.  The parser consumes the
//   MAC-side frame-body byte stream from observer_axis_mac_rx, binds the SFD
//   hardware timestamp to the current frame, and emits a one-cycle record pulse
//   for the TX-domain Delay_Resp generator.
//
// Supported frame form:
//   Ethernet II + IPv4(no options, IHL=5) + UDP + PTPv2 Delay_Req
//     UDP port 319, messageType=1, versionPTP=2, messageLength=44.
//
// No RX backpressure is available.  If the downstream record bridge is busy when
// a matching Delay_Req ends, this module drops the record and increments the
// drop counter.
//////////////////////////////////////////////////////////////////////////////////
module observer_ptp_delay_req_rx #(
    parameter [47:0] LOCAL_MAC            = 48'h04_CB_01_DA_AA_DA,
    parameter [47:0] DUT_MAC              = 48'h00_0A_35_01_FE_C0,
    parameter [31:0] LOCAL_IP             = 32'hC0A8_0003,
    parameter [31:0] DUT_IP               = 32'hC0A8_000E,
    parameter [7:0]  PTP_DOMAIN           = 8'd0,
    parameter        ACCEPT_PTP_MULTICAST = 1'b1,
    parameter        ACCEPT_BROADCAST_MAC = 1'b1
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         enable,

    input  wire [7:0]   s_axis_tdata,
    input  wire         s_axis_tvalid,
    input  wire         s_axis_tlast,

    input  wire [63:0]  s_frame_timestamp_ns,
    input  wire [47:0]  s_frame_timestamp_seconds,
    input  wire [31:0]  s_frame_timestamp_nanoseconds,
    input  wire         s_frame_timestamp_valid,

    output reg          m_record_valid,
    input  wire         m_record_ready,
    output reg  [15:0]  m_sequence_id,
    output reg  [79:0]  m_source_port_identity,
    output reg  [63:0]  m_t4_ns,
    output reg  [47:0]  m_t4_seconds,
    output reg  [31:0]  m_t4_nanoseconds,

    output reg  [31:0]  stat_delay_req_count,
    output reg  [31:0]  stat_delay_req_drop_count,
    output reg  [31:0]  stat_non_delay_ptp_count,
    output reg  [15:0]  stat_last_sequence_id,
    output reg  [63:0]  stat_last_t4_ns,
    output reg  [15:0]  stat_last_error
);

    localparam [15:0] ERR_NONE          = 16'h0000;
    localparam [15:0] ERR_NOT_DELAY_REQ = 16'h0001;
    localparam [15:0] ERR_NO_TIMESTAMP  = 16'h0002;
    localparam [15:0] ERR_RECORD_BUSY   = 16'h0003;
    localparam [15:0] ERR_BAD_FORMAT    = 16'h0004;

    localparam [15:0] ETH_TYPE_IPV4     = 16'h0800;
    localparam [7:0]  IPV4_PROTO_UDP    = 8'd17;
    localparam [15:0] UDP_PORT_EVENT    = 16'd319;
    localparam [15:0] PTP_DELAY_REQ_LEN = 16'd44;
    localparam [3:0]  MSG_DELAY_REQ     = 4'h1;
    localparam [47:0] PTP_MCAST_MAC     = 48'h01_1B_19_00_00_00;
    localparam [47:0] BROADCAST_MAC     = 48'hFF_FF_FF_FF_FF_FF;
    localparam [31:0] PTP_MCAST_IP      = 32'hE000_0181; // 224.0.1.129

    reg        in_frame_r;
    reg [15:0] byte_idx_r;

    reg        pending_ts_valid_r;
    reg [63:0] pending_ts_ns_r;
    reg [47:0] pending_ts_seconds_r;
    reg [31:0] pending_ts_nanoseconds_r;

    reg        frame_ts_valid_r;
    reg [63:0] frame_ts_ns_r;
    reg [47:0] frame_ts_seconds_r;
    reg [31:0] frame_ts_nanoseconds_r;

    reg [47:0] dst_mac_r;
    reg [47:0] src_mac_r;
    reg [15:0] eth_type_r;
    reg [7:0]  ip_version_ihl_r;
    reg [7:0]  ip_protocol_r;
    reg [31:0] ip_src_r;
    reg [31:0] ip_dst_r;
    reg [15:0] udp_src_port_r;
    reg [15:0] udp_dst_port_r;
    reg [3:0]  ptp_msg_type_r;
    reg [7:0]  ptp_version_r;
    reg [15:0] ptp_msg_len_r;
    reg [7:0]  ptp_domain_r;
    reg [79:0] ptp_source_port_id_r;
    reg [15:0] ptp_sequence_id_r;

    wire dst_mac_match_w;
    wire src_mac_match_w;
    wire ip_dst_match_w;
    wire ptp_candidate_w;
    wire delay_req_match_w;

    assign dst_mac_match_w = (dst_mac_r == LOCAL_MAC) ||
                             (ACCEPT_PTP_MULTICAST && (dst_mac_r == PTP_MCAST_MAC)) ||
                             (ACCEPT_BROADCAST_MAC && (dst_mac_r == BROADCAST_MAC));
    assign src_mac_match_w = (src_mac_r == DUT_MAC);
    assign ip_dst_match_w  = (ip_dst_r == LOCAL_IP) ||
                             (ACCEPT_PTP_MULTICAST && (ip_dst_r == PTP_MCAST_IP));

    assign ptp_candidate_w = (eth_type_r == ETH_TYPE_IPV4) &&
                             (ip_version_ihl_r == 8'h45) &&
                             (ip_protocol_r == IPV4_PROTO_UDP) &&
                             ((udp_src_port_r == UDP_PORT_EVENT) ||
                              (udp_dst_port_r == UDP_PORT_EVENT)) &&
                             (ptp_version_r[3:0] == 4'h2);

    assign delay_req_match_w = enable && frame_ts_valid_r &&
                               dst_mac_match_w && src_mac_match_w &&
                               (ip_src_r == DUT_IP) && ip_dst_match_w &&
                               ptp_candidate_w &&
                               (ptp_msg_type_r == MSG_DELAY_REQ) &&
                               (ptp_msg_len_r == PTP_DELAY_REQ_LEN) &&
                               (ptp_domain_r == PTP_DOMAIN);

    task t_clear_parse_fields;
        begin
            dst_mac_r            <= 48'd0;
            src_mac_r            <= 48'd0;
            eth_type_r           <= 16'd0;
            ip_version_ihl_r     <= 8'd0;
            ip_protocol_r        <= 8'd0;
            ip_src_r             <= 32'd0;
            ip_dst_r             <= 32'd0;
            udp_src_port_r       <= 16'd0;
            udp_dst_port_r       <= 16'd0;
            ptp_msg_type_r       <= 4'd0;
            ptp_version_r        <= 8'd0;
            ptp_msg_len_r        <= 16'd0;
            ptp_domain_r         <= 8'd0;
            ptp_source_port_id_r <= 80'd0;
            ptp_sequence_id_r    <= 16'd0;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_frame_r                 <= 1'b0;
            byte_idx_r                 <= 16'd0;
            pending_ts_valid_r         <= 1'b0;
            pending_ts_ns_r            <= 64'd0;
            pending_ts_seconds_r       <= 48'd0;
            pending_ts_nanoseconds_r   <= 32'd0;
            frame_ts_valid_r           <= 1'b0;
            frame_ts_ns_r              <= 64'd0;
            frame_ts_seconds_r         <= 48'd0;
            frame_ts_nanoseconds_r     <= 32'd0;
            m_record_valid             <= 1'b0;
            m_sequence_id              <= 16'd0;
            m_source_port_identity     <= 80'd0;
            m_t4_ns                    <= 64'd0;
            m_t4_seconds               <= 48'd0;
            m_t4_nanoseconds           <= 32'd0;
            stat_delay_req_count       <= 32'd0;
            stat_delay_req_drop_count  <= 32'd0;
            stat_non_delay_ptp_count   <= 32'd0;
            stat_last_sequence_id      <= 16'd0;
            stat_last_t4_ns            <= 64'd0;
            stat_last_error            <= ERR_NONE;
            t_clear_parse_fields();
        end else begin
            m_record_valid <= 1'b0;

            if (!enable) begin
                in_frame_r         <= 1'b0;
                byte_idx_r         <= 16'd0;
                pending_ts_valid_r <= 1'b0;
                frame_ts_valid_r   <= 1'b0;
            end else begin
                if (s_frame_timestamp_valid) begin
                    pending_ts_valid_r       <= 1'b1;
                    pending_ts_ns_r          <= s_frame_timestamp_ns;
                    pending_ts_seconds_r     <= s_frame_timestamp_seconds;
                    pending_ts_nanoseconds_r <= s_frame_timestamp_nanoseconds;
                end

                if (s_axis_tvalid) begin
                    if (!in_frame_r) begin
                        in_frame_r             <= 1'b1;
                        byte_idx_r             <= 16'd0;
                        frame_ts_valid_r       <= pending_ts_valid_r;
                        frame_ts_ns_r          <= pending_ts_ns_r;
                        frame_ts_seconds_r     <= pending_ts_seconds_r;
                        frame_ts_nanoseconds_r <= pending_ts_nanoseconds_r;
                        pending_ts_valid_r     <= 1'b0;
                        t_clear_parse_fields();
                    end

                    case (byte_idx_r)
                        16'd0:  dst_mac_r[47:40]            <= s_axis_tdata;
                        16'd1:  dst_mac_r[39:32]            <= s_axis_tdata;
                        16'd2:  dst_mac_r[31:24]            <= s_axis_tdata;
                        16'd3:  dst_mac_r[23:16]            <= s_axis_tdata;
                        16'd4:  dst_mac_r[15:8]             <= s_axis_tdata;
                        16'd5:  dst_mac_r[7:0]              <= s_axis_tdata;
                        16'd6:  src_mac_r[47:40]            <= s_axis_tdata;
                        16'd7:  src_mac_r[39:32]            <= s_axis_tdata;
                        16'd8:  src_mac_r[31:24]            <= s_axis_tdata;
                        16'd9:  src_mac_r[23:16]            <= s_axis_tdata;
                        16'd10: src_mac_r[15:8]             <= s_axis_tdata;
                        16'd11: src_mac_r[7:0]              <= s_axis_tdata;
                        16'd12: eth_type_r[15:8]            <= s_axis_tdata;
                        16'd13: eth_type_r[7:0]             <= s_axis_tdata;
                        16'd14: ip_version_ihl_r            <= s_axis_tdata;
                        16'd23: ip_protocol_r               <= s_axis_tdata;
                        16'd26: ip_src_r[31:24]             <= s_axis_tdata;
                        16'd27: ip_src_r[23:16]             <= s_axis_tdata;
                        16'd28: ip_src_r[15:8]              <= s_axis_tdata;
                        16'd29: ip_src_r[7:0]               <= s_axis_tdata;
                        16'd30: ip_dst_r[31:24]             <= s_axis_tdata;
                        16'd31: ip_dst_r[23:16]             <= s_axis_tdata;
                        16'd32: ip_dst_r[15:8]              <= s_axis_tdata;
                        16'd33: ip_dst_r[7:0]               <= s_axis_tdata;
                        16'd34: udp_src_port_r[15:8]        <= s_axis_tdata;
                        16'd35: udp_src_port_r[7:0]         <= s_axis_tdata;
                        16'd36: udp_dst_port_r[15:8]        <= s_axis_tdata;
                        16'd37: udp_dst_port_r[7:0]         <= s_axis_tdata;
                        16'd42: ptp_msg_type_r              <= s_axis_tdata[3:0];
                        16'd43: ptp_version_r               <= s_axis_tdata;
                        16'd44: ptp_msg_len_r[15:8]         <= s_axis_tdata;
                        16'd45: ptp_msg_len_r[7:0]          <= s_axis_tdata;
                        16'd46: ptp_domain_r                <= s_axis_tdata;
                        16'd62: ptp_source_port_id_r[79:72] <= s_axis_tdata;
                        16'd63: ptp_source_port_id_r[71:64] <= s_axis_tdata;
                        16'd64: ptp_source_port_id_r[63:56] <= s_axis_tdata;
                        16'd65: ptp_source_port_id_r[55:48] <= s_axis_tdata;
                        16'd66: ptp_source_port_id_r[47:40] <= s_axis_tdata;
                        16'd67: ptp_source_port_id_r[39:32] <= s_axis_tdata;
                        16'd68: ptp_source_port_id_r[31:24] <= s_axis_tdata;
                        16'd69: ptp_source_port_id_r[23:16] <= s_axis_tdata;
                        16'd70: ptp_source_port_id_r[15:8]  <= s_axis_tdata;
                        16'd71: ptp_source_port_id_r[7:0]   <= s_axis_tdata;
                        16'd72: ptp_sequence_id_r[15:8]     <= s_axis_tdata;
                        16'd73: ptp_sequence_id_r[7:0]      <= s_axis_tdata;
                        default: begin end
                    endcase

                    if (s_axis_tlast) begin
                        if (delay_req_match_w) begin
                            if (m_record_ready) begin
                                m_record_valid         <= 1'b1;
                                m_sequence_id          <= ptp_sequence_id_r;
                                m_source_port_identity <= ptp_source_port_id_r;
                                m_t4_ns                <= frame_ts_ns_r;
                                m_t4_seconds           <= frame_ts_seconds_r;
                                m_t4_nanoseconds       <= frame_ts_nanoseconds_r;
                                stat_delay_req_count   <= stat_delay_req_count + 32'd1;
                                stat_last_sequence_id  <= ptp_sequence_id_r;
                                stat_last_t4_ns        <= frame_ts_ns_r;
                                stat_last_error        <= ERR_NONE;
                            end else begin
                                stat_delay_req_drop_count <= stat_delay_req_drop_count + 32'd1;
                                stat_last_error           <= ERR_RECORD_BUSY;
                            end
                        end else if (ptp_candidate_w) begin
                            stat_non_delay_ptp_count <= stat_non_delay_ptp_count + 32'd1;
                            if (!frame_ts_valid_r)
                                stat_last_error <= ERR_NO_TIMESTAMP;
                            else if (ptp_msg_type_r != MSG_DELAY_REQ)
                                stat_last_error <= ERR_NOT_DELAY_REQ;
                            else
                                stat_last_error <= ERR_BAD_FORMAT;
                        end

                        in_frame_r <= 1'b0;
                        byte_idx_r <= 16'd0;
                    end else begin
                        byte_idx_r <= byte_idx_r + 16'd1;
                    end
                end
            end
        end
    end

endmodule
`default_nettype wire
