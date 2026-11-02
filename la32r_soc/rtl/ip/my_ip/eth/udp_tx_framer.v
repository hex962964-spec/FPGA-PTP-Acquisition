`timescale 1ns / 1ps

/*
 * -----------------------------------------------------------------------------
 * udp_tx_framer
 * -----------------------------------------------------------------------------
 * 作用：
 *   1) 接收 net_acq_pkt_ingress_adapter 已缓存好的 16-bit word 帧。
 *   2) 根据 frame_word_count 计算 UDP Length。
 *   3) 在 payload 前面插入 8-byte UDP Header：
 *        Source Port / Destination Port / Length / Checksum
 *   4) 将 16-bit payload word 转成 8-bit byte stream（高字节先发）。
 *
 * 设计说明：
 *   - 当前版本的 UDP checksum 固定填 16'h0000（即不计算 UDP 校验和）。
 *   - 采用“先知道整帧长度，再开始发送”的策略，便于后级 ipv4_tx_framer
 *     在发包前就知道 IP total length。
 *   - 对 ingress 的 frame_pop 由本模块在收到下游 m_frame_pop 后产生；
 *     ingress adapter 随后进入 DRAIN，把 payload word 吐给本模块。
 *   - 内部使用 2-word 小缓存，避免 16-bit -> 8-bit 转换时在每个 word 边界插气泡。
 * -----------------------------------------------------------------------------
 */
module udp_tx_framer #(
    parameter integer P_WORD_W = 16
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
    // UDP 静态配置
    // ------------------------------------------------------------------------
    input  wire [15:0]              cfg_udp_src_port,
    input  wire [15:0]              cfg_udp_dst_port,

    // ------------------------------------------------------------------------
    // 来自 ingress adapter 的 word 级帧输出
    // s_frame_ready      : 有完整 payload 帧可用
    // s_frame_word_count : 该帧包含多少个 16-bit word
    // s_frame_pop        : 本模块拉高 1 个周期，通知 ingress 开始 DRAIN
    // s_word_*           : ingress DRAIN 期间逐 word 输出 payload
    // ------------------------------------------------------------------------
    input  wire [P_WORD_W-1:0]      s_word_data,
    input  wire                     s_word_valid,
    input  wire                     s_word_last,
    output wire                     s_word_ready,

    input  wire                     s_frame_ready,
    input  wire [15:0]              s_frame_word_count,
    output reg                      s_frame_pop,

    // ------------------------------------------------------------------------
    // 输出给 ipv4_tx_framer 的 UDP datagram byte stream
    // m_frame_ready : 当前有一整个 UDP datagram 可开始发送
    // m_udp_length  : UDP header + payload 的总长度（bytes）
    // m_frame_pop   : 由下游拉高 1 个周期，表示“开始消费当前 UDP datagram”
    // m_byte_*      : UDP datagram 的字节流（先 8-byte UDP header，再 payload bytes）
    // ------------------------------------------------------------------------
    output wire                     m_frame_ready,
    output wire [15:0]              m_udp_length,
    input  wire                     m_frame_pop,

    output wire [7:0]               m_byte_data,
    output wire                     m_byte_valid,
    output wire                     m_byte_last,
    input  wire                     m_byte_ready,

    // ------------------------------------------------------------------------
    // 状态 / 统计
    // ------------------------------------------------------------------------
    output wire                     busy,
    output reg  [31:0]              stat_datagram_count,
    output reg  [15:0]              stat_last_udp_length,
    output reg  [15:0]              stat_last_error
);

    localparam [1:0] ST_IDLE         = 2'd0;
    localparam [1:0] ST_SEND_UDP_HDR = 2'd1;
    localparam [1:0] ST_SEND_PAYLOAD = 2'd2;

    localparam [15:0] ERR_NONE = 16'd0;

    reg [1:0]  state_r;
    reg [3:0]  udp_hdr_idx_r;
    reg [15:0] active_udp_length_r;

    // ------------------------------------------------------------------------
    // 2-word 小缓存：
    //   q0 作为当前输出 word，q1 作为预取 word。
    //   byte_sel_r=0 -> 发 q0 的高字节；byte_sel_r=1 -> 发 q0 的低字节。
    // ------------------------------------------------------------------------
    reg [P_WORD_W-1:0] q0_data_r;
    reg                q0_last_r;
    reg [P_WORD_W-1:0] q1_data_r;
    reg                q1_last_r;
    reg [1:0]          q_count_r;
    reg                byte_sel_r;

    wire [15:0] calc_payload_length;
    wire [15:0] calc_udp_length;
    wire        s_fire;
    wire        m_fire;
    wire        payload_word_done;
    reg  [7:0]  udp_hdr_byte_c;

    assign calc_payload_length = {s_frame_word_count[14:0], 1'b0};  // word_count * 2
    assign calc_udp_length     = calc_payload_length + 16'd8;

    assign m_frame_ready = cfg_enable && (state_r == ST_IDLE) && s_frame_ready;
    assign m_udp_length  = (state_r == ST_IDLE) ? calc_udp_length : active_udp_length_r;
    assign busy          = (state_r != ST_IDLE);

    // 只要内部 2-word 小缓存未满，就继续向 ingress 要 word。
    assign s_word_ready = ((state_r == ST_SEND_UDP_HDR) || (state_r == ST_SEND_PAYLOAD)) &&
                          (q_count_r < 2);

    // 当前输出字节流：
    //   - UDP 头阶段：输出 8-byte UDP Header
    //   - Payload 阶段：输出 q0 的高/低字节
    assign m_byte_valid = (state_r == ST_SEND_UDP_HDR) ? 1'b1 :
                          ((state_r == ST_SEND_PAYLOAD) && (q_count_r != 0));

    assign m_byte_data  = (state_r == ST_SEND_UDP_HDR) ? udp_hdr_byte_c :
                          (byte_sel_r ? q0_data_r[7:0] : q0_data_r[15:8]);

    // m_byte_last 只在 UDP datagram 最后一个 payload byte 拉高
    assign m_byte_last  = (state_r == ST_SEND_PAYLOAD) &&
                          (q_count_r != 0) && q0_last_r && byte_sel_r;

    assign s_fire            = s_word_valid && s_word_ready;
    assign m_fire            = m_byte_valid && m_byte_ready;
    assign payload_word_done = (state_r == ST_SEND_PAYLOAD) && m_fire && byte_sel_r;

    // ------------------------------------------------------------------------
    // UDP Header 字节选择
    // byte[0:1] = Source Port
    // byte[2:3] = Destination Port
    // byte[4:5] = UDP Length
    // byte[6:7] = UDP Checksum (当前固定为 0)
    // ------------------------------------------------------------------------
    always @(*) begin
        case (udp_hdr_idx_r)
            4'd0:    udp_hdr_byte_c = cfg_udp_src_port[15:8];
            4'd1:    udp_hdr_byte_c = cfg_udp_src_port[7:0];
            4'd2:    udp_hdr_byte_c = cfg_udp_dst_port[15:8];
            4'd3:    udp_hdr_byte_c = cfg_udp_dst_port[7:0];
            4'd4:    udp_hdr_byte_c = active_udp_length_r[15:8];
            4'd5:    udp_hdr_byte_c = active_udp_length_r[7:0];
            4'd6:    udp_hdr_byte_c = 8'h00;
            default: udp_hdr_byte_c = 8'h00;
        endcase
    end

    // ------------------------------------------------------------------------
    // 主状态机
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin : UDP_TX_MAIN
        reg [P_WORD_W-1:0] new_word_data_v;
        reg                new_word_last_v;
        reg                have_new_word_v;
        reg                dequeue_word_v;
        reg                enqueue_word_v;
        reg                finish_frame_v;
        reg [1:0]          next_q_count_v;
        reg [P_WORD_W-1:0] next_q0_data_v;
        reg                next_q0_last_v;
        reg [P_WORD_W-1:0] next_q1_data_v;
        reg                next_q1_last_v;

        if (!rst_n) begin
            state_r              <= ST_IDLE;
            udp_hdr_idx_r        <= 4'd0;
            active_udp_length_r  <= 16'd0;
            q0_data_r            <= {P_WORD_W{1'b0}};
            q0_last_r            <= 1'b0;
            q1_data_r            <= {P_WORD_W{1'b0}};
            q1_last_r            <= 1'b0;
            q_count_r            <= 2'd0;
            byte_sel_r           <= 1'b0;
            s_frame_pop          <= 1'b0;
            stat_datagram_count  <= 32'd0;
            stat_last_udp_length <= 16'd0;
            stat_last_error      <= ERR_NONE;
        end else begin
            // 默认脉冲信号拉低
            s_frame_pop <= 1'b0;

            if (cfg_soft_reset) begin
                state_r             <= ST_IDLE;
                udp_hdr_idx_r       <= 4'd0;
                active_udp_length_r <= 16'd0;
                q0_data_r           <= {P_WORD_W{1'b0}};
                q0_last_r           <= 1'b0;
                q1_data_r           <= {P_WORD_W{1'b0}};
                q1_last_r           <= 1'b0;
                q_count_r           <= 2'd0;
                byte_sel_r          <= 1'b0;
            end else begin
                if (cfg_clear_status) begin
                    stat_datagram_count  <= 32'd0;
                    stat_last_udp_length <= 16'd0;
                    stat_last_error      <= ERR_NONE;
                end

                // ------------------------------------------------------------
                // 通用的 2-word 小缓存 enqueue/dequeue 处理
                // ------------------------------------------------------------
                new_word_data_v  = s_word_data;
                new_word_last_v  = s_word_last;
                have_new_word_v  = s_fire;
                dequeue_word_v   = payload_word_done;
                finish_frame_v   = payload_word_done && q0_last_r;
                enqueue_word_v   = have_new_word_v;

                next_q_count_v   = q_count_r;
                next_q0_data_v   = q0_data_r;
                next_q0_last_v   = q0_last_r;
                next_q1_data_v   = q1_data_r;
                next_q1_last_v   = q1_last_r;

                case ({enqueue_word_v, dequeue_word_v})
                    2'b00: begin
                        // no-op
                    end

                    2'b01: begin
                        // 只出队一个完整 word
                        case (q_count_r)
                            2'd0: begin
                                next_q_count_v = 2'd0;
                            end
                            2'd1: begin
                                next_q_count_v = 2'd0;
                            end
                            default: begin
                                next_q0_data_v = q1_data_r;
                                next_q0_last_v = q1_last_r;
                                next_q_count_v = 2'd1;
                            end
                        endcase
                    end

                    2'b10: begin
                        // 只入队一个新 word
                        case (q_count_r)
                            2'd0: begin
                                next_q0_data_v = new_word_data_v;
                                next_q0_last_v = new_word_last_v;
                                next_q_count_v = 2'd1;
                            end
                            2'd1: begin
                                next_q1_data_v = new_word_data_v;
                                next_q1_last_v = new_word_last_v;
                                next_q_count_v = 2'd2;
                            end
                            default: begin
                                // q_count_r==2 时不会出现 enqueue，因为 s_word_ready=0
                                next_q_count_v = q_count_r;
                            end
                        endcase
                    end

                    default: begin
                        // 同时入队和出队一个 word
                        case (q_count_r)
                            2'd0: begin
                                next_q0_data_v = new_word_data_v;
                                next_q0_last_v = new_word_last_v;
                                next_q_count_v = 2'd1;
                            end
                            2'd1: begin
                                // q0 完成后，用新 word 直接顶替 q0，保证不插气泡
                                next_q0_data_v = new_word_data_v;
                                next_q0_last_v = new_word_last_v;
                                next_q_count_v = 2'd1;
                            end
                            default: begin
                                // q0 完成，q1 前移到 q0，同时新 word 填入 q1
                                next_q0_data_v = q1_data_r;
                                next_q0_last_v = q1_last_r;
                                next_q1_data_v = new_word_data_v;
                                next_q1_last_v = new_word_last_v;
                                next_q_count_v = 2'd2;
                            end
                        endcase
                    end
                endcase

                // 默认更新缓存
                q0_data_r  <= next_q0_data_v;
                q0_last_r  <= next_q0_last_v;
                q1_data_r  <= next_q1_data_v;
                q1_last_r  <= next_q1_last_v;
                q_count_r  <= next_q_count_v;

                case (state_r)
                    // --------------------------------------------------------
                    // 等待 ingress 提供“完整一帧可用”
                    // --------------------------------------------------------
                    ST_IDLE: begin
                        udp_hdr_idx_r <= 4'd0;
                        byte_sel_r    <= 1'b0;
                        q_count_r     <= 2'd0;

                        if (m_frame_ready && m_frame_pop) begin
                            active_udp_length_r <= calc_udp_length;
                            s_frame_pop         <= 1'b1;      // 通知 ingress 开始 DRAIN
                            state_r             <= ST_SEND_UDP_HDR;
                        end
                    end

                    // --------------------------------------------------------
                    // 输出 8-byte UDP Header
                    // 同时允许预取 payload word 到 2-word 小缓存
                    // --------------------------------------------------------
                    ST_SEND_UDP_HDR: begin
                        byte_sel_r <= 1'b0;

                        if (m_fire) begin
                            if (udp_hdr_idx_r == 4'd7) begin
                                udp_hdr_idx_r <= 4'd0;
                                state_r       <= ST_SEND_PAYLOAD;
                            end else begin
                                udp_hdr_idx_r <= udp_hdr_idx_r + 4'd1;
                            end
                        end
                    end

                    // --------------------------------------------------------
                    // 输出 payload bytes（高字节先发，低字节后发）
                    // --------------------------------------------------------
                    ST_SEND_PAYLOAD: begin
                        if (m_fire) begin
                            if (!byte_sel_r) begin
                                // 刚发完高字节，下一拍发低字节
                                byte_sel_r <= 1'b1;
                            end else begin
                                // 刚发完低字节，下一拍切回下一个 word 的高字节
                                byte_sel_r <= 1'b0;

                                if (finish_frame_v) begin
                                    state_r              <= ST_IDLE;
                                    stat_datagram_count  <= stat_datagram_count + 32'd1;
                                    stat_last_udp_length <= active_udp_length_r;
                                end
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

endmodule
