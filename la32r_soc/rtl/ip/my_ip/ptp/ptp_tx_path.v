`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: ptp_tx_path
// Description:
//   PTP 主动发送路径。
//
//   该模块把原 net_acq_ptp_subsystem 中与 eth_tx 域数据面相关的内容收成一层：
//     command capture / request staging
//         -> ptp_tx_engine
//         -> udp_tx_framer
//         -> ipv4_tx_framer
//         -> ptp_tx_* byte stream to tx_arbiter
//
//   行为保持与当前工程一致：
//     - 软件主动 TX 仍固定收口到 Delay_Req；
//     - 普通 UDP 数据面的 eth_cfg_dst_* 与 PTP peer 地址已经解耦，
//       该路径只消费由 net_acq_eth_cfg_manager 解析好的 eth_cfg_ptp_peer_*；
//     - 只有在需要时间戳时，且 txstamp bridge 源侧 ready，才允许真正放行；
//     - exec_ack pulse 的语义仍是“TX path 已经接收这条命令”，而不是“帧已发完”；
//     - 新增的 trigger_req / trigger_accept / trigger_while_busy 计数器只做
//       可观测性，不改变当前单-entry staging 的功能语义。
//////////////////////////////////////////////////////////////////////////////////
module ptp_tx_path(
    input  wire         eth_tx_clk,
    input  wire         eth_tx_resetn,

    // ------------------------------------------------------------------------
    // Active PTP configuration in eth_tx domain
    // ------------------------------------------------------------------------
    input  wire         tx_cfg_enable_eth,
    input  wire         eth_soft_reset_pulse_eth,
    input  wire         eth_clear_status_pulse_eth,
    input  wire         eth_ptp_tx_abort_pulse_eth,

    // ------------------------------------------------------------------------
    // Active ETH network configuration in eth_tx domain
    //   Local DUT identity remains shared with the normal UDP data plane.
    //   Only the destination peer is decoupled for compact PTP traffic.
    // ------------------------------------------------------------------------
    input  wire [47:0]  eth_cfg_local_mac_eth,
    input  wire [31:0]  eth_cfg_local_ip_eth,
    // Dedicated PTP peer address used by Delay_Req TX.
    // These signals are already resolved to legacy-or-split behavior by
    // net_acq_eth_cfg_manager, so this path never needs to inspect the mode bit.
    input  wire [47:0]  eth_cfg_ptp_peer_mac_eth,
    input  wire [31:0]  eth_cfg_ptp_peer_ip_eth,
    input  wire [7:0]   eth_cfg_ttl_eth,
    input  wire [7:0]   eth_cfg_dscp_ecn_eth,
    input  wire [15:0]  eth_cfg_ip_ident_seed_eth,

    // ------------------------------------------------------------------------
    // Decoded TX command from ptp_cfg_cmd_manager
    // ------------------------------------------------------------------------
    input  wire         tx_cmd_update_pulse_eth,
    input  wire         tx_cmd_need_timestamp_eth,
    input  wire [7:0]   tx_cmd_log_message_interval_eth,
    input  wire [15:0]  tx_cmd_flags_eth,
    input  wire [15:0]  tx_cmd_sequence_id_eth,
    input  wire [3:0]   tx_cmd_transport_specific_eth,
    input  wire [3:0]   tx_cmd_message_type_eth,
    input  wire [15:0]  tx_cmd_port_num_eth,
    input  wire [63:0]  tx_cmd_clock_id_eth,
    input  wire [7:0]   tx_cmd_domain_eth,

    // ------------------------------------------------------------------------
    // Timestamp bridge source readiness (single-outstanding model)
    // ------------------------------------------------------------------------
    input  wire         ptp_txstamp_src_ready_eth,

    // ------------------------------------------------------------------------
    // Output to the common tx_arbiter
    // ------------------------------------------------------------------------
    output wire [7:0]   ptp_tx_tdata,
    output wire         ptp_tx_tvalid,
    output wire         ptp_tx_tlast,
    input  wire         ptp_tx_tready,
    output wire [3:0]   ptp_tx_tuser,

    // ------------------------------------------------------------------------
    // Status / handshake back to ptp_cfg_cmd_manager
    // ------------------------------------------------------------------------
    output wire         ptp_path_busy_eth,
    output wire         ptp_tx_pending_any_eth,
    output wire         ptp_tx_cmd_exec_ack_pulse_eth,

    // ------------------------------------------------------------------------
    // Pre-scheduler observability extension (eth_tx domain)
    // ------------------------------------------------------------------------
    output wire [31:0]  ptp_tx_trigger_req_count_eth,
    output wire [31:0]  ptp_tx_trigger_accept_count_eth,
    output wire [31:0]  ptp_tx_trigger_while_busy_count_eth,
    output wire [15:0]  ptp_tx_last_error_eth
);

    localparam [3:0] C_PTP_MSG_DELAY_REQ = 4'h1;

    reg                 ptp_tx_req_pending_eth_r;
    reg                 ptp_tx_cmd_pending_eth_r;
    reg [31:0]          ptp_tx_trigger_req_count_eth_r;
    reg [31:0]          ptp_tx_trigger_accept_count_eth_r;
    reg [31:0]          ptp_tx_trigger_while_busy_count_eth_r;
    reg [7:0]           ptp_tx_domain_eth_r;
    reg [63:0]          ptp_tx_clock_id_eth_r;
    reg [15:0]          ptp_tx_port_num_eth_r;
    reg [3:0]           ptp_tx_message_type_eth_r;
    reg [3:0]           ptp_tx_transport_specific_eth_r;
    reg [15:0]          ptp_tx_sequence_id_eth_r;
    reg [15:0]          ptp_tx_flags_eth_r;
    reg [7:0]           ptp_tx_log_interval_eth_r;
    reg                 ptp_tx_need_timestamp_eth_r;

    wire                ptp_engine_cmd_ready_w;
    wire                ptp_engine_frame_ready_w;
    wire [15:0]         ptp_engine_frame_word_count_w;
    wire                ptp_engine_frame_pop_w;
    wire [15:0]         ptp_engine_word_data_w;
    wire                ptp_engine_word_valid_w;
    wire                ptp_engine_word_last_w;
    wire                ptp_engine_word_ready_w;
    wire                ptp_engine_need_timestamp_w;
    wire [1:0]          ptp_engine_frame_type_w;
    wire [15:0]         ptp_engine_active_sequence_id_w;
    wire                ptp_engine_busy_w;

    wire                ptp_udp_frame_ready_w;
    wire [15:0]         ptp_udp_length_w;
    wire                ptp_udp_frame_pop_w;
    wire [7:0]          ptp_udp_byte_data_w;
    wire                ptp_udp_byte_valid_w;
    wire                ptp_udp_byte_last_w;
    wire                ptp_udp_byte_ready_w;
    wire                ptp_udp_busy_w;

    wire [7:0]          ptp_ipv4_m_tx_data_w;
    wire                ptp_ipv4_m_tx_valid_w;
    wire                ptp_ipv4_m_tx_last_w;
    wire                ptp_ipv4_m_tx_ready_w;
    wire                ptp_ipv4_m_tx_start_w;
    wire                ptp_ipv4_busy_w;

    wire [15:0]         ptp_udp_port_eth_w;
    wire                ptp_tx_pipeline_busy_eth_w;
    wire                ptp_tx_cmd_handshake_eth_w;
    wire                ptp_tx_trigger_capture_eth_w;
    wire                ptp_tx_trigger_accept_eth_w;
    wire                ptp_tx_trigger_while_busy_eth_w;
    wire [31:0]         ptp_engine_stat_frame_accept_count_w;
    wire [31:0]         ptp_engine_stat_frame_drop_count_w;
    wire [15:0]         ptp_engine_stat_last_error_w;
    wire [31:0]         ptp_udp_stat_datagram_count_w;
    wire [15:0]         ptp_udp_stat_last_udp_length_w;
    wire [15:0]         ptp_udp_stat_last_error_w;
    wire [31:0]         ptp_ipv4_stat_tx_frame_count_w;
    wire [31:0]         ptp_ipv4_stat_tx_byte_count_w;
    wire [31:0]         ptp_ipv4_stat_backpressure_cycles_w;
    wire [15:0]         ptp_ipv4_stat_last_ip_total_length_w;
    wire [15:0]         ptp_ipv4_stat_last_udp_length_w;
    wire [15:0]         ptp_ipv4_stat_last_error_w;
    wire [15:0]         ptp_tx_last_error_eth_w;
    wire                unused_keep_w;

    assign ptp_udp_port_eth_w =
        (ptp_engine_frame_type_w == 2'b01) ? 16'd319 : 16'd320;

    assign ptp_tx_pipeline_busy_eth_w =
        ptp_tx_cmd_pending_eth_r |
        ptp_engine_busy_w |
        ptp_udp_busy_w |
        ptp_ipv4_busy_w;

    assign ptp_tx_cmd_handshake_eth_w =
        ptp_tx_cmd_pending_eth_r && ptp_engine_cmd_ready_w;

    assign ptp_tx_cmd_exec_ack_pulse_eth = ptp_tx_cmd_handshake_eth_w;
    assign ptp_tx_trigger_capture_eth_w  = tx_cmd_update_pulse_eth;

    // "while busy" 的含义是：软件在当前单-entry staging 尚未完全排空时，
    // 又送来了一次新的 trigger。这里先只做计数，不改变现有覆盖式写入行为。
    assign ptp_tx_trigger_accept_eth_w   =
        ptp_tx_req_pending_eth_r &&
        !ptp_tx_pipeline_busy_eth_w &&
        (!ptp_tx_need_timestamp_eth_r || ptp_txstamp_src_ready_eth);
    assign ptp_tx_trigger_while_busy_eth_w =
        tx_cmd_update_pulse_eth &&
        (ptp_tx_req_pending_eth_r || ptp_tx_cmd_pending_eth_r ||
         ptp_engine_busy_w || ptp_udp_busy_w || ptp_ipv4_busy_w);

    assign ptp_tx_pending_any_eth =
        ptp_tx_req_pending_eth_r |
        ptp_tx_cmd_pending_eth_r;

    assign ptp_path_busy_eth =
        ptp_tx_req_pending_eth_r |
        ptp_tx_pipeline_busy_eth_w;

    assign ptp_tx_last_error_eth_w = (ptp_engine_stat_last_error_w != 16'd0) ? ptp_engine_stat_last_error_w :
                                     (ptp_udp_stat_last_error_w    != 16'd0) ? ptp_udp_stat_last_error_w    :
                                                                                 ptp_ipv4_stat_last_error_w;

    assign ptp_tx_trigger_req_count_eth        = ptp_tx_trigger_req_count_eth_r;
    assign ptp_tx_trigger_accept_count_eth     = ptp_tx_trigger_accept_count_eth_r;
    assign ptp_tx_trigger_while_busy_count_eth = ptp_tx_trigger_while_busy_count_eth_r;
    assign ptp_tx_last_error_eth               = ptp_tx_last_error_eth_w;

    assign ptp_ipv4_m_tx_ready_w = ptp_tx_tready;

    assign ptp_tx_tdata  = ptp_ipv4_m_tx_data_w;
    assign ptp_tx_tvalid = ptp_ipv4_m_tx_valid_w;
    assign ptp_tx_tlast  = ptp_ipv4_m_tx_last_w;
    assign ptp_tx_tuser  = {ptp_engine_frame_type_w, ptp_engine_need_timestamp_w, 1'b0};

    always @(posedge eth_tx_clk or negedge eth_tx_resetn) begin
        if (!eth_tx_resetn) begin
            ptp_tx_req_pending_eth_r        <= 1'b0;
            ptp_tx_cmd_pending_eth_r        <= 1'b0;
            ptp_tx_trigger_req_count_eth_r  <= 32'd0;
            ptp_tx_trigger_accept_count_eth_r <= 32'd0;
            ptp_tx_trigger_while_busy_count_eth_r <= 32'd0;
            ptp_tx_need_timestamp_eth_r     <= 1'b0;
            ptp_tx_domain_eth_r             <= 8'd0;
            ptp_tx_clock_id_eth_r           <= 64'd0;
            ptp_tx_port_num_eth_r           <= 16'd0;
            ptp_tx_message_type_eth_r       <= 4'd0;
            ptp_tx_transport_specific_eth_r <= 4'd0;
            ptp_tx_sequence_id_eth_r        <= 16'd0;
            ptp_tx_flags_eth_r              <= 16'd0;
            ptp_tx_log_interval_eth_r       <= 8'd0;
        end else begin
            if (eth_soft_reset_pulse_eth || eth_clear_status_pulse_eth ||
                eth_ptp_tx_abort_pulse_eth || !tx_cfg_enable_eth) begin
                ptp_tx_req_pending_eth_r            <= 1'b0;
                ptp_tx_cmd_pending_eth_r            <= 1'b0;
                ptp_tx_trigger_req_count_eth_r      <= 32'd0;
                ptp_tx_trigger_accept_count_eth_r   <= 32'd0;
                ptp_tx_trigger_while_busy_count_eth_r <= 32'd0;
                ptp_tx_need_timestamp_eth_r         <= 1'b0;
            end else begin
                if (ptp_tx_trigger_capture_eth_w) begin
                    ptp_tx_trigger_req_count_eth_r  <= ptp_tx_trigger_req_count_eth_r + 32'd1;
                    if (ptp_tx_trigger_while_busy_eth_w)
                        ptp_tx_trigger_while_busy_count_eth_r <= ptp_tx_trigger_while_busy_count_eth_r + 32'd1;
                    ptp_tx_need_timestamp_eth_r     <= tx_cmd_need_timestamp_eth;
                    ptp_tx_domain_eth_r             <= tx_cmd_domain_eth;
                    ptp_tx_clock_id_eth_r           <= tx_cmd_clock_id_eth;
                    ptp_tx_port_num_eth_r           <= tx_cmd_port_num_eth;
                    ptp_tx_message_type_eth_r       <= C_PTP_MSG_DELAY_REQ;
                    ptp_tx_transport_specific_eth_r <= tx_cmd_transport_specific_eth;
                    ptp_tx_sequence_id_eth_r        <= tx_cmd_sequence_id_eth;
                    ptp_tx_flags_eth_r              <= tx_cmd_flags_eth;
                    ptp_tx_log_interval_eth_r       <= tx_cmd_log_message_interval_eth;
                    ptp_tx_req_pending_eth_r        <= 1'b1;
                end

                if (ptp_tx_trigger_accept_eth_w) begin
                    ptp_tx_req_pending_eth_r        <= 1'b0;
                    ptp_tx_cmd_pending_eth_r        <= 1'b1;
                    ptp_tx_trigger_accept_count_eth_r <= ptp_tx_trigger_accept_count_eth_r + 32'd1;
                end

                if (ptp_tx_cmd_handshake_eth_w)
                    ptp_tx_cmd_pending_eth_r <= 1'b0;
            end
        end
    end

    ptp_tx_engine #(
        .P_WORD_W    (16),
        .P_MAX_WORDS (32)
    ) u_ptp_tx_engine (
        .clk                      (eth_tx_clk),
        .rst_n                    (eth_tx_resetn),
        .cfg_enable               (tx_cfg_enable_eth),
        .cfg_soft_reset           (eth_soft_reset_pulse_eth),
        .cfg_clear_status         (eth_clear_status_pulse_eth),
        .cfg_domain               (ptp_tx_domain_eth_r),
        .cfg_clock_id             (ptp_tx_clock_id_eth_r),
        .cfg_port_num             (ptp_tx_port_num_eth_r),
        .cmd_valid                (ptp_tx_cmd_pending_eth_r),
        .cmd_ready                (ptp_engine_cmd_ready_w),
        .cmd_message_type         (ptp_tx_message_type_eth_r),
        .cmd_transport_specific   (ptp_tx_transport_specific_eth_r),
        .cmd_need_timestamp       (ptp_tx_need_timestamp_eth_r),
        .cmd_sequence_id          (ptp_tx_sequence_id_eth_r),
        .cmd_flags                (ptp_tx_flags_eth_r),
        .cmd_correction_field     (64'd0),
        .cmd_log_message_interval (ptp_tx_log_interval_eth_r),
        .cmd_timestamp_80         (80'd0),
        .cmd_request_clock_id     (ptp_tx_clock_id_eth_r),
        .cmd_request_port_num     (ptp_tx_port_num_eth_r),
        .m_frame_ready            (ptp_engine_frame_ready_w),
        .m_frame_word_count       (ptp_engine_frame_word_count_w),
        .m_frame_pop              (ptp_engine_frame_pop_w),
        .m_word_data              (ptp_engine_word_data_w),
        .m_word_valid             (ptp_engine_word_valid_w),
        .m_word_last              (ptp_engine_word_last_w),
        .m_word_ready             (ptp_engine_word_ready_w),
        .m_need_timestamp         (ptp_engine_need_timestamp_w),
        .m_frame_type             (ptp_engine_frame_type_w),
        .m_active_sequence_id     (ptp_engine_active_sequence_id_w),
        .busy                     (ptp_engine_busy_w),
        .stat_frame_accept_count  (ptp_engine_stat_frame_accept_count_w),
        .stat_frame_drop_count    (ptp_engine_stat_frame_drop_count_w),
        .stat_last_error          (ptp_engine_stat_last_error_w)
    );

    udp_tx_framer #(
        .P_WORD_W (16)
    ) u_ptp_udp_tx_framer (
        .clk                (eth_tx_clk),
        .rst_n              (eth_tx_resetn),
        .cfg_enable         (tx_cfg_enable_eth),
        .cfg_soft_reset     (eth_soft_reset_pulse_eth),
        .cfg_clear_status   (eth_clear_status_pulse_eth),
        .cfg_udp_src_port   (ptp_udp_port_eth_w),
        .cfg_udp_dst_port   (ptp_udp_port_eth_w),
        .s_word_data        (ptp_engine_word_data_w),
        .s_word_valid       (ptp_engine_word_valid_w),
        .s_word_last        (ptp_engine_word_last_w),
        .s_word_ready       (ptp_engine_word_ready_w),
        .s_frame_ready      (ptp_engine_frame_ready_w),
        .s_frame_word_count (ptp_engine_frame_word_count_w),
        .s_frame_pop        (ptp_engine_frame_pop_w),
        .m_frame_ready      (ptp_udp_frame_ready_w),
        .m_udp_length       (ptp_udp_length_w),
        .m_frame_pop        (ptp_udp_frame_pop_w),
        .m_byte_data        (ptp_udp_byte_data_w),
        .m_byte_valid       (ptp_udp_byte_valid_w),
        .m_byte_last        (ptp_udp_byte_last_w),
        .m_byte_ready       (ptp_udp_byte_ready_w),
        .busy               (ptp_udp_busy_w),
        .stat_datagram_count(ptp_udp_stat_datagram_count_w),
        .stat_last_udp_length(ptp_udp_stat_last_udp_length_w),
        .stat_last_error    (ptp_udp_stat_last_error_w)
    );

    ipv4_tx_framer #(
        .P_MAC_W (8)
    ) u_ptp_ipv4_tx_framer (
        .clk                       (eth_tx_clk),
        .rst_n                     (eth_tx_resetn),
        .cfg_enable                (tx_cfg_enable_eth),
        .cfg_soft_reset            (eth_soft_reset_pulse_eth),
        .cfg_clear_status          (eth_clear_status_pulse_eth),
        .cfg_dst_mac               (eth_cfg_ptp_peer_mac_eth),
        .cfg_src_mac               (eth_cfg_local_mac_eth),
        .cfg_src_ip                (eth_cfg_local_ip_eth),
        .cfg_dst_ip                (eth_cfg_ptp_peer_ip_eth),
        .cfg_ip_ttl                (eth_cfg_ttl_eth),
        .cfg_ip_dscp_ecn           (eth_cfg_dscp_ecn_eth),
        .cfg_ip_ident_seed         (eth_cfg_ip_ident_seed_eth),
        .s_frame_ready             (ptp_udp_frame_ready_w),
        .s_udp_length              (ptp_udp_length_w),
        .s_frame_pop               (ptp_udp_frame_pop_w),
        .s_byte_data               (ptp_udp_byte_data_w),
        .s_byte_valid              (ptp_udp_byte_valid_w),
        .s_byte_last               (ptp_udp_byte_last_w),
        .s_byte_ready              (ptp_udp_byte_ready_w),
        .m_tx_data                 (ptp_ipv4_m_tx_data_w),
        .m_tx_valid                (ptp_ipv4_m_tx_valid_w),
        .m_tx_last                 (ptp_ipv4_m_tx_last_w),
        .m_tx_ready                (ptp_ipv4_m_tx_ready_w),
        .m_tx_start                (ptp_ipv4_m_tx_start_w),
        .busy                      (ptp_ipv4_busy_w),
        .stat_tx_frame_count       (ptp_ipv4_stat_tx_frame_count_w),
        .stat_tx_byte_count        (ptp_ipv4_stat_tx_byte_count_w),
        .stat_backpressure_cycles  (ptp_ipv4_stat_backpressure_cycles_w),
        .stat_last_ip_total_length (ptp_ipv4_stat_last_ip_total_length_w),
        .stat_last_udp_length      (ptp_ipv4_stat_last_udp_length_w),
        .stat_last_error           (ptp_ipv4_stat_last_error_w)
    );

    assign unused_keep_w = tx_cmd_message_type_eth[0] |
                           ptp_ipv4_m_tx_start_w |
                           ptp_engine_active_sequence_id_w[0] |
                           ptp_engine_stat_frame_accept_count_w[0] |
                           ptp_engine_stat_frame_drop_count_w[0] |
                           ptp_udp_stat_datagram_count_w[0] |
                           ptp_udp_stat_last_udp_length_w[0] |
                           ptp_ipv4_stat_tx_frame_count_w[0] |
                           ptp_ipv4_stat_tx_byte_count_w[0] |
                           ptp_ipv4_stat_backpressure_cycles_w[0] |
                           ptp_ipv4_stat_last_ip_total_length_w[0] |
                           ptp_ipv4_stat_last_udp_length_w[0];

endmodule
