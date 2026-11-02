`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: ptp_rx_filter
// -----------------------------------------------------------------------------
// Purpose:
//   Minimal PTP RX parser and normalizer for the compact closed loop.
//
// Design goals in this revision:
//   1) Keep the RX side strictly "minimal PTP only" and never back-pressure the
//      MAC-side RX byte stream.
//   2) Match only the messages the CPU slave state machine really needs:
//        - Sync       (event, UDP 319)
//        - Follow_Up  (general, UDP 320)
//        - Delay_Resp (general, UDP 320)
//   3) Keep the record delivered to the sys-domain mailbox deterministic:
//        - require complete core header fields before considering a frame
//        - require complete messageTimestamp for all accepted messages
//        - require complete requestingPortIdentity for Delay_Resp
//        - require a valid RX SFD timestamp only for Sync
//   4) Upgrade the output handshake from a one-cycle pulse to a held valid/ready
//      staging register, so a momentary sink stall does not drop a good frame.
//
// Integration model:
//   - This block runs in eth_rx_clk domain.
//   - It never back-pressures the RX byte stream.
//   - It contains one record staging register on the output side.
//   - If that staging register is still occupied when another accepted frame
//     completes, the new record is dropped locally and ERR_MBOX_FULL is raised.
//
// Record format (256 bits):
//   [255:252] msg_type
//   [251:236] sequence_id
//   [235]     has_rx_timestamp
//   [234]     has_msg_timestamp
//   [233]     has_requesting_port_id
//   [232]     reserved
//   [231:168] rx_timestamp_ns
//   [167:88]  msg_timestamp_80
//   [87:8]    requesting_port_identity_80
//   [7:0]     reserved for future flags/debug
//////////////////////////////////////////////////////////////////////////////////
module ptp_rx_filter(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         cfg_enable,
    input  wire         cfg_soft_reset,
    input  wire         cfg_clear_status,
    input  wire [47:0]  cfg_local_mac,
    input  wire [47:0]  cfg_host_mac,
    input  wire [31:0]  cfg_local_ip,
    input  wire [31:0]  cfg_host_ip,
    input  wire [7:0]   cfg_ptp_domain,

    input  wire [7:0]   s_axis_tdata,
    input  wire         s_axis_tvalid,
    input  wire         s_axis_tlast,

    input  wire [63:0]  ptp_frame_rxstamp_ns,
    input  wire         ptp_frame_rxstamp_valid,

    output reg          rec_valid,
    input  wire         rec_ready,
    output reg  [255:0] rec_data,

    output reg  [31:0]  stat_accept_count,
    output reg  [31:0]  stat_drop_count,
    output reg  [15:0]  stat_last_error
);

    localparam [15:0] ERR_NONE            = 16'h0000;
    localparam [15:0] ERR_MATCH_FAIL      = 16'h0001;
    localparam [15:0] ERR_UNSUPPORTED_MSG = 16'h0002;
    localparam [15:0] ERR_MBOX_FULL       = 16'h0003;
    localparam [15:0] ERR_MISSING_RXSTAMP = 16'h0004;

    localparam [3:0]  MSG_SYNC       = 4'h0;
    localparam [3:0]  MSG_FOLLOW_UP  = 4'h8;
    localparam [3:0]  MSG_DELAY_RESP = 4'h9;

    localparam [15:0] UDP_PORT_EVENT   = 16'd319;
    localparam [15:0] UDP_PORT_GENERAL = 16'd320;

    reg         in_frame_r;
    reg         frame_done_pending_r;
    reg [7:0]   byte_idx_r;

    reg         pending_rxstamp_valid_r;
    reg [63:0]  pending_rxstamp_ns_r;
    reg         frame_rxstamp_valid_r;
    reg [63:0]  frame_rxstamp_ns_r;

    reg [47:0]  dst_mac_r;
    reg [47:0]  src_mac_r;
    reg [15:0]  ethertype_r;
    reg [3:0]   ip_version_r;
    reg [3:0]   ip_ihl_r;
    reg [7:0]   ip_protocol_r;
    reg [31:0]  src_ip_r;
    reg [31:0]  dst_ip_r;
    reg [15:0]  udp_src_port_r;
    reg [15:0]  udp_dst_port_r;
    reg [3:0]   ptp_msg_type_r;
    reg [3:0]   ptp_version_r;
    reg [7:0]   ptp_domain_r;
    reg [15:0]  ptp_sequence_id_r;
    reg [79:0]  ptp_msg_timestamp_80_r;
    reg [79:0]  ptp_requesting_port_identity_r;

    reg         have_l2_r;
    reg         have_l3_l4_r;
    reg         have_ptp_core_r;
    reg         have_msg_ts_r;
    reg         have_req_pid_r;

    wire [7:0]  frame_byte_idx_w;
    wire        sfd_stamp_available_w;
    wire [63:0] sfd_stamp_value_w;

    wire        is_sync_w;
    wire        is_follow_up_w;
    wire        is_delay_resp_w;
    wire        is_supported_msg_w;
    wire        ptp_candidate_w;
    wire        addr_domain_match_w;
    wire        msg_body_complete_w;
    wire        msg_port_match_w;
    wire        accept_frame_w;
    wire        need_rxstamp_w;
    wire        rec_has_rx_ts_w;
    wire        rec_has_msg_ts_w;
    wire [79:0] rec_msg_ts_80_w;
    wire        out_fire_w;
    wire        out_slot_free_w;

    assign frame_byte_idx_w      = in_frame_r ? byte_idx_r : 8'd0;
    assign sfd_stamp_available_w = ptp_frame_rxstamp_valid | pending_rxstamp_valid_r;
    assign sfd_stamp_value_w     = ptp_frame_rxstamp_valid ?
                                   ptp_frame_rxstamp_ns : pending_rxstamp_ns_r;

    assign is_sync_w          = (ptp_msg_type_r == MSG_SYNC);
    assign is_follow_up_w     = (ptp_msg_type_r == MSG_FOLLOW_UP);
    assign is_delay_resp_w    = (ptp_msg_type_r == MSG_DELAY_RESP);
    assign is_supported_msg_w = is_sync_w | is_follow_up_w | is_delay_resp_w;

    // "PTP-looking" candidate: complete enough to know this is IPv4/UDP/PTPv2 on
    // the PTP well-known ports. Only such frames are counted as match failures.
    assign ptp_candidate_w = have_l2_r &&
                             have_l3_l4_r &&
                             have_ptp_core_r &&
                             (ethertype_r   == 16'h0800) &&
                             (ip_version_r  == 4'd4)     &&
                             (ip_ihl_r      == 4'd5)     &&
                             (ip_protocol_r == 8'd17)    &&
                             ((udp_src_port_r == UDP_PORT_EVENT)   ||
                              (udp_src_port_r == UDP_PORT_GENERAL) ||
                              (udp_dst_port_r == UDP_PORT_EVENT)   ||
                              (udp_dst_port_r == UDP_PORT_GENERAL)) &&
                             (ptp_version_r == 4'd2);

    assign addr_domain_match_w = cfg_enable &&
                                 ptp_candidate_w &&
                                 (dst_mac_r    == cfg_local_mac) &&
                                 (src_mac_r    == cfg_host_mac)  &&
                                 (dst_ip_r     == cfg_local_ip)  &&
                                 (src_ip_r     == cfg_host_ip)   &&
                                 (ptp_domain_r == cfg_ptp_domain);

    assign msg_body_complete_w = have_msg_ts_r &&
                                 (!is_delay_resp_w || have_req_pid_r);

    assign msg_port_match_w = (is_sync_w &&
                               (udp_src_port_r == UDP_PORT_EVENT) &&
                               (udp_dst_port_r == UDP_PORT_EVENT)) ||
                              ((is_follow_up_w || is_delay_resp_w) &&
                               (udp_src_port_r == UDP_PORT_GENERAL) &&
                               (udp_dst_port_r == UDP_PORT_GENERAL));

    assign accept_frame_w = addr_domain_match_w &&
                            is_supported_msg_w  &&
                            msg_body_complete_w &&
                            msg_port_match_w;

    assign need_rxstamp_w   = is_sync_w;
    assign rec_has_rx_ts_w  = need_rxstamp_w && frame_rxstamp_valid_r;
    assign rec_has_msg_ts_w = is_follow_up_w || is_delay_resp_w;
    assign rec_msg_ts_80_w  = rec_has_msg_ts_w ? ptp_msg_timestamp_80_r : 80'd0;
    assign out_fire_w       = rec_valid && rec_ready;
    assign out_slot_free_w  = !rec_valid || rec_ready;

    task t_clear_frame_fields;
        begin
            byte_idx_r                     <= 8'd0;
            frame_rxstamp_valid_r          <= 1'b0;
            frame_rxstamp_ns_r             <= 64'd0;
            dst_mac_r                      <= 48'd0;
            src_mac_r                      <= 48'd0;
            ethertype_r                    <= 16'd0;
            ip_version_r                   <= 4'd0;
            ip_ihl_r                       <= 4'd0;
            ip_protocol_r                  <= 8'd0;
            src_ip_r                       <= 32'd0;
            dst_ip_r                       <= 32'd0;
            udp_src_port_r                 <= 16'd0;
            udp_dst_port_r                 <= 16'd0;
            ptp_msg_type_r                 <= 4'd0;
            ptp_version_r                  <= 4'd0;
            ptp_domain_r                   <= 8'd0;
            ptp_sequence_id_r              <= 16'd0;
            ptp_msg_timestamp_80_r         <= 80'd0;
            ptp_requesting_port_identity_r <= 80'd0;
            have_l2_r                      <= 1'b0;
            have_l3_l4_r                   <= 1'b0;
            have_ptp_core_r                <= 1'b0;
            have_msg_ts_r                  <= 1'b0;
            have_req_pid_r                 <= 1'b0;
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_frame_r                     <= 1'b0;
            frame_done_pending_r           <= 1'b0;
            byte_idx_r                     <= 8'd0;
            pending_rxstamp_valid_r        <= 1'b0;
            pending_rxstamp_ns_r           <= 64'd0;
            frame_rxstamp_valid_r          <= 1'b0;
            frame_rxstamp_ns_r             <= 64'd0;
            dst_mac_r                      <= 48'd0;
            src_mac_r                      <= 48'd0;
            ethertype_r                    <= 16'd0;
            ip_version_r                   <= 4'd0;
            ip_ihl_r                       <= 4'd0;
            ip_protocol_r                  <= 8'd0;
            src_ip_r                       <= 32'd0;
            dst_ip_r                       <= 32'd0;
            udp_src_port_r                 <= 16'd0;
            udp_dst_port_r                 <= 16'd0;
            ptp_msg_type_r                 <= 4'd0;
            ptp_version_r                  <= 4'd0;
            ptp_domain_r                   <= 8'd0;
            ptp_sequence_id_r              <= 16'd0;
            ptp_msg_timestamp_80_r         <= 80'd0;
            ptp_requesting_port_identity_r <= 80'd0;
            have_l2_r                      <= 1'b0;
            have_l3_l4_r                   <= 1'b0;
            have_ptp_core_r                <= 1'b0;
            have_msg_ts_r                  <= 1'b0;
            have_req_pid_r                 <= 1'b0;
            rec_valid                      <= 1'b0;
            rec_data                       <= 256'd0;
            stat_accept_count              <= 32'd0;
            stat_drop_count                <= 32'd0;
            stat_last_error                <= ERR_NONE;
        end else begin
            if (ptp_frame_rxstamp_valid) begin
                pending_rxstamp_valid_r <= 1'b1;
                pending_rxstamp_ns_r    <= ptp_frame_rxstamp_ns;
            end

            if (cfg_soft_reset || cfg_clear_status || !cfg_enable) begin
                in_frame_r              <= 1'b0;
                frame_done_pending_r    <= 1'b0;
                pending_rxstamp_valid_r <= 1'b0;
                pending_rxstamp_ns_r    <= 64'd0;
                rec_valid               <= 1'b0;
                rec_data                <= 256'd0;
                t_clear_frame_fields();

                if (cfg_clear_status) begin
                    stat_accept_count <= 32'd0;
                    stat_drop_count   <= 32'd0;
                    stat_last_error   <= ERR_NONE;
                end
            end else begin
                if (out_fire_w) begin
                    rec_valid          <= 1'b0;
                    stat_accept_count  <= stat_accept_count + 32'd1;
                end

                if (frame_done_pending_r) begin
                    frame_done_pending_r <= 1'b0;

                    if (accept_frame_w) begin
                        if (need_rxstamp_w && !frame_rxstamp_valid_r) begin
                            stat_drop_count <= stat_drop_count + 32'd1;
                            stat_last_error <= ERR_MISSING_RXSTAMP;
                        end else if (out_slot_free_w) begin
                            rec_valid <= 1'b1;
                            rec_data  <= {
                                ptp_msg_type_r,
                                ptp_sequence_id_r,
                                rec_has_rx_ts_w,
                                rec_has_msg_ts_w,
                                is_delay_resp_w,
                                1'b0,
                                rec_has_rx_ts_w ? frame_rxstamp_ns_r : 64'd0,
                                rec_msg_ts_80_w,
                                is_delay_resp_w ?
                                    ptp_requesting_port_identity_r : 80'd0,
                                8'd0
                            };
                            stat_last_error   <= ERR_NONE;
                        end else begin
                            stat_drop_count <= stat_drop_count + 32'd1;
                            stat_last_error <= ERR_MBOX_FULL;
                        end
                    end else if (ptp_candidate_w) begin
                        stat_drop_count <= stat_drop_count + 32'd1;
                        if (!addr_domain_match_w)
                            stat_last_error <= ERR_MATCH_FAIL;
                        else if (!is_supported_msg_w)
                            stat_last_error <= ERR_UNSUPPORTED_MSG;
                        else
                            stat_last_error <= ERR_MATCH_FAIL;
                    end else begin
                        // Ignore unrelated non-PTP traffic.
                    end
                end

                if (s_axis_tvalid) begin
                    if (!in_frame_r) begin
                        in_frame_r              <= 1'b1;
                        byte_idx_r              <= 8'd0;
                        frame_rxstamp_valid_r   <= sfd_stamp_available_w;
                        frame_rxstamp_ns_r      <= sfd_stamp_value_w;
                        pending_rxstamp_valid_r <= 1'b0;
                        pending_rxstamp_ns_r    <= 64'd0;

                        dst_mac_r                      <= 48'd0;
                        src_mac_r                      <= 48'd0;
                        ethertype_r                    <= 16'd0;
                        ip_version_r                   <= 4'd0;
                        ip_ihl_r                       <= 4'd0;
                        ip_protocol_r                  <= 8'd0;
                        src_ip_r                       <= 32'd0;
                        dst_ip_r                       <= 32'd0;
                        udp_src_port_r                 <= 16'd0;
                        udp_dst_port_r                 <= 16'd0;
                        ptp_msg_type_r                 <= 4'd0;
                        ptp_version_r                  <= 4'd0;
                        ptp_domain_r                   <= 8'd0;
                        ptp_sequence_id_r              <= 16'd0;
                        ptp_msg_timestamp_80_r         <= 80'd0;
                        ptp_requesting_port_identity_r <= 80'd0;
                        have_l2_r                      <= 1'b0;
                        have_l3_l4_r                   <= 1'b0;
                        have_ptp_core_r                <= 1'b0;
                        have_msg_ts_r                  <= 1'b0;
                        have_req_pid_r                 <= 1'b0;
                    end

                    case (frame_byte_idx_w)
                        8'd0:  dst_mac_r[47:40] <= s_axis_tdata;
                        8'd1:  dst_mac_r[39:32] <= s_axis_tdata;
                        8'd2:  dst_mac_r[31:24] <= s_axis_tdata;
                        8'd3:  dst_mac_r[23:16] <= s_axis_tdata;
                        8'd4:  dst_mac_r[15:8]  <= s_axis_tdata;
                        8'd5:  dst_mac_r[7:0]   <= s_axis_tdata;
                        8'd6:  src_mac_r[47:40] <= s_axis_tdata;
                        8'd7:  src_mac_r[39:32] <= s_axis_tdata;
                        8'd8:  src_mac_r[31:24] <= s_axis_tdata;
                        8'd9:  src_mac_r[23:16] <= s_axis_tdata;
                        8'd10: src_mac_r[15:8]  <= s_axis_tdata;
                        8'd11: src_mac_r[7:0]   <= s_axis_tdata;
                        8'd12: ethertype_r[15:8] <= s_axis_tdata;
                        8'd13: begin
                            ethertype_r[7:0] <= s_axis_tdata;
                            have_l2_r        <= 1'b1;
                        end

                        8'd14: begin
                            ip_version_r <= s_axis_tdata[7:4];
                            ip_ihl_r     <= s_axis_tdata[3:0];
                        end
                        8'd23: ip_protocol_r <= s_axis_tdata;

                        8'd26: src_ip_r[31:24] <= s_axis_tdata;
                        8'd27: src_ip_r[23:16] <= s_axis_tdata;
                        8'd28: src_ip_r[15:8]  <= s_axis_tdata;
                        8'd29: src_ip_r[7:0]   <= s_axis_tdata;
                        8'd30: dst_ip_r[31:24] <= s_axis_tdata;
                        8'd31: dst_ip_r[23:16] <= s_axis_tdata;
                        8'd32: dst_ip_r[15:8]  <= s_axis_tdata;
                        8'd33: dst_ip_r[7:0]   <= s_axis_tdata;

                        8'd34: udp_src_port_r[15:8] <= s_axis_tdata;
                        8'd35: udp_src_port_r[7:0]  <= s_axis_tdata;
                        8'd36: udp_dst_port_r[15:8] <= s_axis_tdata;
                        8'd37: begin
                            udp_dst_port_r[7:0] <= s_axis_tdata;
                            have_l3_l4_r        <= 1'b1;
                        end

                        8'd42: ptp_msg_type_r <= s_axis_tdata[3:0];
                        8'd43: ptp_version_r  <= s_axis_tdata[3:0];
                        8'd46: ptp_domain_r   <= s_axis_tdata;
                        8'd72: ptp_sequence_id_r[15:8] <= s_axis_tdata;
                        8'd73: begin
                            ptp_sequence_id_r[7:0] <= s_axis_tdata;
                            have_ptp_core_r        <= 1'b1;
                        end

                        8'd76: ptp_msg_timestamp_80_r[79:72] <= s_axis_tdata;
                        8'd77: ptp_msg_timestamp_80_r[71:64] <= s_axis_tdata;
                        8'd78: ptp_msg_timestamp_80_r[63:56] <= s_axis_tdata;
                        8'd79: ptp_msg_timestamp_80_r[55:48] <= s_axis_tdata;
                        8'd80: ptp_msg_timestamp_80_r[47:40] <= s_axis_tdata;
                        8'd81: ptp_msg_timestamp_80_r[39:32] <= s_axis_tdata;
                        8'd82: ptp_msg_timestamp_80_r[31:24] <= s_axis_tdata;
                        8'd83: ptp_msg_timestamp_80_r[23:16] <= s_axis_tdata;
                        8'd84: ptp_msg_timestamp_80_r[15:8]  <= s_axis_tdata;
                        8'd85: begin
                            ptp_msg_timestamp_80_r[7:0] <= s_axis_tdata;
                            have_msg_ts_r               <= 1'b1;
                        end

                        8'd86: ptp_requesting_port_identity_r[79:72] <= s_axis_tdata;
                        8'd87: ptp_requesting_port_identity_r[71:64] <= s_axis_tdata;
                        8'd88: ptp_requesting_port_identity_r[63:56] <= s_axis_tdata;
                        8'd89: ptp_requesting_port_identity_r[55:48] <= s_axis_tdata;
                        8'd90: ptp_requesting_port_identity_r[47:40] <= s_axis_tdata;
                        8'd91: ptp_requesting_port_identity_r[39:32] <= s_axis_tdata;
                        8'd92: ptp_requesting_port_identity_r[31:24] <= s_axis_tdata;
                        8'd93: ptp_requesting_port_identity_r[23:16] <= s_axis_tdata;
                        8'd94: ptp_requesting_port_identity_r[15:8]  <= s_axis_tdata;
                        8'd95: begin
                            ptp_requesting_port_identity_r[7:0] <= s_axis_tdata;
                            have_req_pid_r                      <= 1'b1;
                        end

                        default: begin
                        end
                    endcase

                    if (s_axis_tlast) begin
                        in_frame_r           <= 1'b0;
                        frame_done_pending_r <= 1'b1;
                        byte_idx_r           <= 8'd0;
                    end else begin
                        byte_idx_r <= frame_byte_idx_w + 8'd1;
                    end
                end
            end
        end
    end

endmodule
