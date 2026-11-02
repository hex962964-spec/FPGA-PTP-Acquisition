`timescale 1ns / 1ps

/*
 * -----------------------------------------------------------------------------
 * ipv4_tx_framer
 * -----------------------------------------------------------------------------
 * 作用：
 *   1) 接收 udp_tx_framer 输出的 UDP datagram byte stream。
 *   2) 在其前面插入 Ethernet Header + IPv4 Header。
 *   3) 输出最终发往 MAC 侧的字节流接口 m_tx_*。
 *
 * 当前版本的封装关系：
 *   [Dst MAC][Src MAC][0x0800]
 *   [IPv4 Header (20 bytes, no options)]
 *   [UDP Header + UDP Payload]
 *
 * 设计说明：
 *   - EtherType 固定为 16'h0800 (IPv4)
 *   - IPv4 Header 长度固定 20 bytes，IHL=5
 *   - Flags/Fragment Offset 固定为 16'h4000（DF=1, offset=0）
 *   - Protocol 固定为 8'h11 (UDP)
 *   - IPv4 checksum uses five short arithmetic stages after packet capture.
 *     It finishes while the first Ethernet/IP header bytes are transmitted;
 *     checksum bytes are not selected until header indices 24 and 25.
 *   - 当前模块不生成 Ethernet FCS，默认留给更下层 MAC / PHY 处理
 * -----------------------------------------------------------------------------
 */
module ipv4_tx_framer #(
    parameter integer P_MAC_W = 8
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ------------------------------------------------------------------------
    // 全局控制
    // ------------------------------------------------------------------------
    input  wire                     cfg_enable,
    input  wire                     cfg_soft_reset,
    input  wire                     cfg_clear_status,

    // ------------------------------------------------------------------------
    // L2/L3 静态配置
    // ------------------------------------------------------------------------
    input  wire [47:0]              cfg_dst_mac,
    input  wire [47:0]              cfg_src_mac,
    input  wire [31:0]              cfg_src_ip,
    input  wire [31:0]              cfg_dst_ip,
    input  wire [7:0]               cfg_ip_ttl,
    input  wire [7:0]               cfg_ip_dscp_ecn,
    input  wire [15:0]              cfg_ip_ident_seed,

    // ------------------------------------------------------------------------
    // 来自 udp_tx_framer 的 UDP datagram byte stream
    // s_frame_ready : 当前有完整 UDP datagram 可发送
    // s_udp_length  : UDP datagram 长度（bytes，含 8-byte UDP header）
    // s_frame_pop   : 本模块拉高 1 个周期，通知 udp_tx_framer 开始发送 datagram
    // s_byte_*      : UDP datagram 字节流
    // ------------------------------------------------------------------------
    input  wire                     s_frame_ready,
    input  wire [15:0]              s_udp_length,
    output reg                      s_frame_pop,

    input  wire [7:0]               s_byte_data,
    input  wire                     s_byte_valid,
    input  wire                     s_byte_last,
    output wire                     s_byte_ready,

    // ------------------------------------------------------------------------
    // 输出到更下层 MAC wrapper / GMII adapter 的字节流
    // ------------------------------------------------------------------------
    output wire [P_MAC_W-1:0]       m_tx_data,
    output wire                     m_tx_valid,
    output wire                     m_tx_last,
    input  wire                     m_tx_ready,
    output wire                     m_tx_start,

    // ------------------------------------------------------------------------
    // 状态 / 统计
    // ------------------------------------------------------------------------
    output wire                     busy,
    output reg  [31:0]              stat_tx_frame_count,
    output reg  [31:0]              stat_tx_byte_count,
    output reg  [31:0]              stat_backpressure_cycles,
    output reg  [15:0]              stat_last_ip_total_length,
    output reg  [15:0]              stat_last_udp_length,
    output reg  [15:0]              stat_last_error
);

    localparam [1:0] ST_IDLE        = 2'd0;
    localparam [1:0] ST_SEND_HDR    = 2'd1;
    localparam [1:0] ST_SEND_PAYLOAD= 2'd2;

    localparam [15:0] ERR_NONE = 16'd0;

    reg [1:0]  state_r;
    reg [5:0]  hdr_idx_r;              // 0..33，覆盖 Ethernet(14) + IPv4(20)

    reg [47:0] dst_mac_r;
    reg [47:0] src_mac_r;
    reg [31:0] src_ip_r;
    reg [31:0] dst_ip_r;
    reg [7:0]  ip_ttl_r;
    reg [7:0]  ip_dscp_ecn_r;
    reg [15:0] ip_ident_r;
    reg [15:0] udp_length_r;
    reg [15:0] ip_total_length_r;
    reg [15:0] ip_header_checksum_r;

    // The packet fields above remain stable until its final byte.  Five
    // arithmetic stages fit inside the 24 header bytes preceding the checksum,
    // even with no downstream stalls.  No packet-start latency is added.
    reg [2:0]  checksum_stage_r;
    reg [16:0] checksum_pair0_r;
    reg [16:0] checksum_pair1_r;
    reg [16:0] checksum_pair2_r;
    reg [16:0] checksum_pair3_r;
    reg [17:0] checksum_half0_r;
    reg [17:0] checksum_half1_r;
    reg [18:0] checksum_eight_words_r;
    reg [19:0] checksum_total_r;
    wire [16:0] checksum_fold_w;
    wire [16:0] checksum_fold_final_w;

    assign checksum_fold_w = {1'b0, checksum_total_r[15:0]} +
                             {13'd0, checksum_total_r[19:16]};
    assign checksum_fold_final_w = {1'b0, checksum_fold_w[15:0]} +
                                   {16'd0, checksum_fold_w[16]};

    reg [7:0]  hdr_byte_c;

    wire       m_fire;
    wire [15:0] next_ip_ident;

    assign next_ip_ident = cfg_ip_ident_seed + stat_tx_frame_count[15:0];

    assign m_tx_valid = (state_r == ST_SEND_HDR) ? 1'b1 :
                        ((state_r == ST_SEND_PAYLOAD) ? s_byte_valid : 1'b0);

    assign m_tx_data  = (state_r == ST_SEND_HDR) ? hdr_byte_c : s_byte_data;
    assign m_tx_last  = (state_r == ST_SEND_PAYLOAD) ? s_byte_last : 1'b0;
    assign m_tx_start = (state_r == ST_SEND_HDR) && (hdr_idx_r == 6'd0) && m_tx_valid && m_tx_ready;

    // 只有真正进入 payload 阶段后，才允许从 udp_tx_framer 取 byte
    assign s_byte_ready = (state_r == ST_SEND_PAYLOAD) && m_tx_ready;

    assign m_fire = m_tx_valid && m_tx_ready;
    assign busy   = (state_r != ST_IDLE);

    // ------------------------------------------------------------------------
    // IPv4 头校验和
    // Header 10 个 16-bit word：
    //   0: {8'h45, dscp_ecn}
    //   1: total_length
    //   2: identification
    //   3: 16'h4000
    //   4: {ttl, 8'h11}
    //   5: checksum (计算时填 0)
    //   6: src_ip[31:16]
    //   7: src_ip[15:0]
    //   8: dst_ip[31:16]
    //   9: dst_ip[15:0]
    // ------------------------------------------------------------------------
    // These data registers need no reset: checksum_stage_r is reset/aborted,
    // and every value is overwritten before checksum_total_r can be consumed.
    // Keeping them off the asynchronous reset tree also avoids extra fanout.
    always @(posedge clk) begin
        case (checksum_stage_r)
            3'd1: begin
                checksum_pair0_r <= {1'b0, 8'h45, ip_dscp_ecn_r} +
                                    {1'b0, ip_total_length_r};
                checksum_pair1_r <= {1'b0, ip_ident_r} + 17'h04000;
                checksum_pair2_r <= {1'b0, ip_ttl_r, 8'h11} +
                                    {1'b0, src_ip_r[31:16]};
                checksum_pair3_r <= {1'b0, src_ip_r[15:0]} +
                                    {1'b0, dst_ip_r[31:16]};
            end
            3'd2: begin
                checksum_half0_r <= {1'b0, checksum_pair0_r} +
                                    {1'b0, checksum_pair1_r};
                checksum_half1_r <= {1'b0, checksum_pair2_r} +
                                    {1'b0, checksum_pair3_r};
            end
            3'd3:
                checksum_eight_words_r <= {1'b0, checksum_half0_r} +
                                          {1'b0, checksum_half1_r};
            3'd4:
                checksum_total_r <= {1'b0, checksum_eight_words_r} +
                                    {4'd0, dst_ip_r[15:0]};
            default: begin end
        endcase
    end

    // ------------------------------------------------------------------------
    // Ethernet + IPv4 头字节选择
    // 0..13  : Ethernet header
    // 14..33 : IPv4 header
    // ------------------------------------------------------------------------
    always @(*) begin
        case (hdr_idx_r)
            // Ethernet header: dst_mac(6) + src_mac(6) + EtherType(2)
            6'd0:  hdr_byte_c = dst_mac_r[47:40];
            6'd1:  hdr_byte_c = dst_mac_r[39:32];
            6'd2:  hdr_byte_c = dst_mac_r[31:24];
            6'd3:  hdr_byte_c = dst_mac_r[23:16];
            6'd4:  hdr_byte_c = dst_mac_r[15:8];
            6'd5:  hdr_byte_c = dst_mac_r[7:0];
            6'd6:  hdr_byte_c = src_mac_r[47:40];
            6'd7:  hdr_byte_c = src_mac_r[39:32];
            6'd8:  hdr_byte_c = src_mac_r[31:24];
            6'd9:  hdr_byte_c = src_mac_r[23:16];
            6'd10: hdr_byte_c = src_mac_r[15:8];
            6'd11: hdr_byte_c = src_mac_r[7:0];
            6'd12: hdr_byte_c = 8'h08;
            6'd13: hdr_byte_c = 8'h00;

            // IPv4 header (20 bytes)
            6'd14: hdr_byte_c = 8'h45;                    // Version=4, IHL=5
            6'd15: hdr_byte_c = ip_dscp_ecn_r;
            6'd16: hdr_byte_c = ip_total_length_r[15:8];
            6'd17: hdr_byte_c = ip_total_length_r[7:0];
            6'd18: hdr_byte_c = ip_ident_r[15:8];
            6'd19: hdr_byte_c = ip_ident_r[7:0];
            6'd20: hdr_byte_c = 8'h40;                    // DF=1, Fragment Offset[12:8]=0
            6'd21: hdr_byte_c = 8'h00;
            6'd22: hdr_byte_c = ip_ttl_r;
            6'd23: hdr_byte_c = 8'h11;                    // UDP
            6'd24: hdr_byte_c = ip_header_checksum_r[15:8];
            6'd25: hdr_byte_c = ip_header_checksum_r[7:0];
            6'd26: hdr_byte_c = src_ip_r[31:24];
            6'd27: hdr_byte_c = src_ip_r[23:16];
            6'd28: hdr_byte_c = src_ip_r[15:8];
            6'd29: hdr_byte_c = src_ip_r[7:0];
            6'd30: hdr_byte_c = dst_ip_r[31:24];
            6'd31: hdr_byte_c = dst_ip_r[23:16];
            6'd32: hdr_byte_c = dst_ip_r[15:8];
            default: hdr_byte_c = dst_ip_r[7:0];
        endcase
    end

    // ------------------------------------------------------------------------
    // 主状态机
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r                  <= ST_IDLE;
            hdr_idx_r                <= 6'd0;
            dst_mac_r                <= 48'd0;
            src_mac_r                <= 48'd0;
            src_ip_r                 <= 32'd0;
            dst_ip_r                 <= 32'd0;
            ip_ttl_r                 <= 8'd0;
            ip_dscp_ecn_r            <= 8'd0;
            ip_ident_r               <= 16'd0;
            udp_length_r             <= 16'd0;
            ip_total_length_r        <= 16'd0;
            ip_header_checksum_r     <= 16'd0;
            checksum_stage_r         <= 3'd0;
            s_frame_pop              <= 1'b0;
            stat_tx_frame_count      <= 32'd0;
            stat_tx_byte_count       <= 32'd0;
            stat_backpressure_cycles <= 32'd0;
            stat_last_ip_total_length<= 16'd0;
            stat_last_udp_length     <= 16'd0;
            stat_last_error          <= ERR_NONE;
        end else begin
            s_frame_pop <= 1'b0;

            if (cfg_soft_reset) begin
                state_r   <= ST_IDLE;
                hdr_idx_r <= 6'd0;
                checksum_stage_r <= 3'd0;
            end else begin
                if (cfg_clear_status) begin
                    stat_tx_frame_count       <= 32'd0;
                    stat_tx_byte_count        <= 32'd0;
                    stat_backpressure_cycles  <= 32'd0;
                    stat_last_ip_total_length <= 16'd0;
                    stat_last_udp_length      <= 16'd0;
                    stat_last_error           <= ERR_NONE;
                end

                // backpressure 统计：只要本模块正在发，但下游没 ready，就记周期数
                if (m_tx_valid && !m_tx_ready) begin
                    stat_backpressure_cycles <= stat_backpressure_cycles + 32'd1;
                end

                // 通用：实际发出 1 个 byte 就加总字节计数
                if (m_fire) begin
                    stat_tx_byte_count <= stat_tx_byte_count + 32'd1;
                end

                case (checksum_stage_r)
                    3'd1, 3'd2, 3'd3, 3'd4:
                        checksum_stage_r <= checksum_stage_r + 3'd1;
                    3'd5: begin
                        ip_header_checksum_r <= ~checksum_fold_final_w[15:0];
                        checksum_stage_r <= 3'd0;
                    end
                    default: checksum_stage_r <= 3'd0;
                endcase

                case (state_r)
                    // --------------------------------------------------------
                    // 等待 udp_tx_framer 提供完整 datagram
                    // --------------------------------------------------------
                    ST_IDLE: begin
                        hdr_idx_r <= 6'd0;

                        if (cfg_enable && s_frame_ready) begin
                            dst_mac_r            <= cfg_dst_mac;
                            src_mac_r            <= cfg_src_mac;
                            src_ip_r             <= cfg_src_ip;
                            dst_ip_r             <= cfg_dst_ip;
                            ip_ttl_r             <= cfg_ip_ttl;
                            ip_dscp_ecn_r        <= cfg_ip_dscp_ecn;
                            ip_ident_r           <= next_ip_ident;
                            udp_length_r         <= s_udp_length;
                            ip_total_length_r    <= s_udp_length + 16'd20;
                            checksum_stage_r      <= 3'd1;

                            stat_last_udp_length      <= s_udp_length;
                            stat_last_ip_total_length <= s_udp_length + 16'd20;

                            s_frame_pop <= 1'b1;      // 通知 udp_tx_framer 开始提供 byte stream
                            state_r     <= ST_SEND_HDR;
                        end
                    end

                    // --------------------------------------------------------
                    // 发送 Ethernet + IPv4 Header (34 bytes)
                    // --------------------------------------------------------
                    ST_SEND_HDR: begin
                        if (m_fire) begin
                            if (hdr_idx_r == 6'd33) begin
                                hdr_idx_r <= 6'd0;
                                state_r   <= ST_SEND_PAYLOAD;
                            end else begin
                                hdr_idx_r <= hdr_idx_r + 6'd1;
                            end
                        end
                    end

                    // --------------------------------------------------------
                    // 发送 UDP datagram bytes
                    // --------------------------------------------------------
                    ST_SEND_PAYLOAD: begin
                        if (m_fire && s_byte_last) begin
                            state_r             <= ST_IDLE;
                            stat_tx_frame_count <= stat_tx_frame_count + 32'd1;
                        end
                    end

                    default: begin
                        state_r <= ST_IDLE;
                    end
                endcase
            end
        end
    end

endmodule
