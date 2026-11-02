`timescale 1ns / 1ps

/*
 * -----------------------------------------------------------------------------
 * net_acq_pkt_ingress_adapter
 * -----------------------------------------------------------------------------
 * 作用：
 *   1) 作为 Ethernet/UDP 后端最前面的“入口接收模块”，消费上游 packet_formatter
 *      输出的 16-bit pkt_* 包流。
 *   2) 采用“单帧 store-and-forward”策略：先完整接收一整帧，再交给后级 framer。
 *   3) 提供 frame_ready/frame_pop 握手，便于后级在拿到完整帧长度后再开始发送。
 *   4) 当收到超长帧（超过 P_MAX_WORDS 且 pkt_last 仍未到）时，不会卡死链路；
 *      模块会进入 DROP 状态，持续丢弃输入，直到看到 pkt_last 完成重同步。
 *
 * 设计假设：
 *   - 一个 net_acq packet 对应一帧后端 payload。
 *   - 当前 packet_formatter 输出的是 16-bit word stream：
 *       s_pkt_data  : 当前 word 数据
 *       s_pkt_valid : 当前 word 有效
 *       s_pkt_last  : 当前 word 为整帧最后一个 word
 *       s_pkt_ready : 本模块是否还能接收当前 word
 *   - 本模块只做“帧缓存与边界管理”，不做 UDP/IP/Ethernet 头封装。
 *
 * 后续建议：
 *   - m_word_* 送给 udp_tx_framer / udp_payload_packer。
 *   - frame_word_count 可直接用于计算：udp_payload_bytes = frame_word_count * 2。
 * -----------------------------------------------------------------------------
 */
module net_acq_pkt_ingress_adapter #(
    parameter integer P_WORD_W    = 16,
    parameter integer P_MAX_WORDS = 272   // 10-word header + up to 256 payload words + margin
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ------------------------------------------------------------------------
    // 全局控制
    // cfg_enable      : 允许接收上游 pkt 流；拉低后 s_pkt_ready=0
    // cfg_soft_reset  : 同步脉冲，清空当前帧/状态机；若中途有帧，会记一次 drop
    // cfg_clear_status: 只清统计/错误码，不清 buffer 和状态机
    // ------------------------------------------------------------------------
    input  wire                     cfg_enable,
    input  wire                     cfg_soft_reset,
    input  wire                     cfg_clear_status,

    // ------------------------------------------------------------------------
    // 上游 packet stream 输入（来自 packet_formatter）
    // ------------------------------------------------------------------------
    input  wire [P_WORD_W-1:0]      s_pkt_data,
    input  wire                     s_pkt_valid,
    input  wire                     s_pkt_last,
    output wire                     s_pkt_ready,

    // ------------------------------------------------------------------------
    // 下游帧数据输出（送给后级 udp/ip/eth framer）
    // m_word_*      : 已缓存完整帧后，按 word 顺序吐出
    // frame_ready   : 当前已有一整帧可供下游读取
    // frame_word_count : 当前整帧 word 数（一个 word = 16bit）
    // frame_pop     : 下游拉高一个周期，表示“开始消费当前整帧”
    // ------------------------------------------------------------------------
    output wire [P_WORD_W-1:0]      m_word_data,
    output wire                     m_word_valid,
    output wire                     m_word_last,
    input  wire                     m_word_ready,

    output wire                     frame_ready,
    output reg  [15:0]              frame_word_count,
    input  wire                     frame_pop,

    // ------------------------------------------------------------------------
    // 状态 / 统计输出
    // busy                  : 非空闲状态
    // drop_mode             : 当前正处于“丢弃直到 pkt_last”的重同步状态
    // stat_frame_accept_count : 成功完整接收的帧数
    // stat_frame_drop_count   : 被丢弃的帧数（超长/soft_reset 打断）
    // stat_last_error         : 最近一次错误码
    // ------------------------------------------------------------------------
    output wire                     busy,
    output wire                     drop_mode,
    output reg  [31:0]              stat_frame_accept_count,
    output reg  [31:0]              stat_frame_drop_count,
    output reg  [15:0]              stat_last_error
);

    // ------------------------------------------------------------------------
    // clog2 helper（Verilog-2001 兼容）
    // ------------------------------------------------------------------------
    function integer f_clog2;
        input integer value;
        integer v;
        integer i;
        begin
            v = value - 1;
            for (i = 0; v > 0; i = i + 1) begin
                v = v >> 1;
            end
            f_clog2 = i;
        end
    endfunction

    localparam integer C_PTR_W = (P_MAX_WORDS <= 2) ? 1 : f_clog2(P_MAX_WORDS);

    // ------------------------------------------------------------------------
    // 状态机定义
    // ST_IDLE  : 空闲，可开始接收一帧
    // ST_RECV  : 正在接收一帧
    // ST_READY : 一整帧已接收完成，等待下游 frame_pop
    // ST_DRAIN : 正在按 word 吐出当前整帧
    // ST_DROP  : 当前帧异常过长，持续丢弃直到看到 pkt_last 以重同步
    // ------------------------------------------------------------------------
    localparam [2:0] ST_IDLE  = 3'd0;
    localparam [2:0] ST_RECV  = 3'd1;
    localparam [2:0] ST_READY = 3'd2;
    localparam [2:0] ST_DRAIN = 3'd3;
    localparam [2:0] ST_DROP  = 3'd4;

    // ------------------------------------------------------------------------
    // 错误码定义
    // ------------------------------------------------------------------------
    localparam [15:0] ERR_NONE             = 16'd0;
    localparam [15:0] ERR_FRAME_TOO_LONG   = 16'd1;
    localparam [15:0] ERR_SOFT_RESET_ABORT = 16'd2;

    reg [2:0] state_r;

    // 单帧 buffer：存储当前完整帧的 word 与对应的 last 标志
    reg [P_WORD_W-1:0] frame_mem      [0:P_MAX_WORDS-1];
    reg                frame_last_mem [0:P_MAX_WORDS-1];

    // wr_count_r：当前接收帧已写入多少个 word（也可作为下一个写地址）
    // rd_count_r：当前输出帧正在读第几个 word
    reg [15:0] wr_count_r;
    reg [15:0] rd_count_r;

    wire s_fire;
    wire m_fire;

    assign s_fire = s_pkt_valid && s_pkt_ready;
    assign m_fire = m_word_valid && m_word_ready;

    // ------------------------------------------------------------------------
    // ready/valid/状态输出
    //
    // s_pkt_ready 的来源：
    //   - 这就是本模块“是否还能继续接上游 word”的判断
    //   - 也就是说，整个 Ethernet backend 对外的 s_pkt_ready，最终就应该
    //     来自这里（通常由 top 直接 assign 出去）
    // ------------------------------------------------------------------------
    assign s_pkt_ready = cfg_enable &&
                         ((state_r == ST_IDLE) ||
                          (state_r == ST_RECV) ||
                          (state_r == ST_DROP));

    assign m_word_valid = (state_r == ST_DRAIN);
    assign m_word_data  = (state_r == ST_DRAIN) ? frame_mem[rd_count_r[C_PTR_W-1:0]]
                                                : {P_WORD_W{1'b0}};
    assign m_word_last  = (state_r == ST_DRAIN) ? frame_last_mem[rd_count_r[C_PTR_W-1:0]]
                                                : 1'b0;

    assign frame_ready = (state_r == ST_READY);
    assign busy       = (state_r != ST_IDLE);
    assign drop_mode  = (state_r == ST_DROP);

    // ------------------------------------------------------------------------
    // 主状态机
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r                  <= ST_IDLE;
            wr_count_r               <= 16'd0;
            rd_count_r               <= 16'd0;
            frame_word_count         <= 16'd0;
            stat_frame_accept_count  <= 32'd0;
            stat_frame_drop_count    <= 32'd0;
            stat_last_error          <= ERR_NONE;
        end else begin
            // 默认：保持当前值

            // 1) soft_reset 优先级最高：清空当前数据面状态，但不清统计；
            //    若中途有帧，则记一次 drop。
            if (cfg_soft_reset) begin
                if (state_r != ST_IDLE) begin
                    stat_frame_drop_count <= stat_frame_drop_count + 32'd1;
                    stat_last_error       <= ERR_SOFT_RESET_ABORT;
                end

                state_r          <= ST_IDLE;
                wr_count_r       <= 16'd0;
                rd_count_r       <= 16'd0;
                frame_word_count <= 16'd0;
            end else begin
                // 2) clear_status：只清统计和错误码，不影响当前 buffer / 状态机
                if (cfg_clear_status) begin
                    stat_frame_accept_count <= 32'd0;
                    stat_frame_drop_count   <= 32'd0;
                    stat_last_error         <= ERR_NONE;
                end

                case (state_r)
                    // --------------------------------------------------------
                    // 空闲：等待首个 word
                    // --------------------------------------------------------
                    ST_IDLE: begin
                        if (s_fire) begin
                            frame_mem[wr_count_r[C_PTR_W-1:0]]      <= s_pkt_data;
                            frame_last_mem[wr_count_r[C_PTR_W-1:0]] <= s_pkt_last;

                            // 单 word 成帧
                            if (s_pkt_last) begin
                                frame_word_count        <= 16'd1;
                                stat_frame_accept_count <= stat_frame_accept_count + 32'd1;
                                rd_count_r              <= 16'd0;
                                wr_count_r              <= 16'd0;
                                state_r                 <= ST_READY;
                            end else begin
                                wr_count_r <= 16'd1;
                                state_r    <= ST_RECV;
                            end
                        end
                    end

                    // --------------------------------------------------------
                    // 接收中：持续缓存 word，直到 pkt_last
                    // --------------------------------------------------------
                    ST_RECV: begin
                        if (s_fire) begin
                            frame_mem[wr_count_r[C_PTR_W-1:0]]      <= s_pkt_data;
                            frame_last_mem[wr_count_r[C_PTR_W-1:0]] <= s_pkt_last;

                            if (s_pkt_last) begin
                                // 当前 word 是整帧最后一个 word
                                frame_word_count        <= wr_count_r + 16'd1;
                                stat_frame_accept_count <= stat_frame_accept_count + 32'd1;
                                rd_count_r              <= 16'd0;
                                wr_count_r              <= 16'd0;
                                state_r                 <= ST_READY;
                            end else if (wr_count_r == (P_MAX_WORDS - 1)) begin
                                // buffer 已经被写满，但 pkt_last 仍未到 => 当前帧过长
                                // 进入 DROP，持续丢弃直到看到 pkt_last，避免卡死链路
                                stat_frame_drop_count <= stat_frame_drop_count + 32'd1;
                                stat_last_error       <= ERR_FRAME_TOO_LONG;
                                frame_word_count      <= 16'd0;
                                wr_count_r            <= 16'd0;
                                rd_count_r            <= 16'd0;
                                state_r               <= ST_DROP;
                            end else begin
                                wr_count_r <= wr_count_r + 16'd1;
                            end
                        end
                    end

                    // --------------------------------------------------------
                    // 一整帧已到齐，等待下游 frame_pop
                    // --------------------------------------------------------
                    ST_READY: begin
                        if (frame_pop) begin
                            rd_count_r <= 16'd0;
                            state_r    <= ST_DRAIN;
                        end
                    end

                    // --------------------------------------------------------
                    // 向下游逐 word 输出当前整帧
                    // --------------------------------------------------------
                    ST_DRAIN: begin
                        if (m_fire) begin
                            if (frame_last_mem[rd_count_r[C_PTR_W-1:0]]) begin
                                // 当前 word 就是本帧最后一个 word，整帧输出完成
                                state_r          <= ST_IDLE;
                                wr_count_r       <= 16'd0;
                                rd_count_r       <= 16'd0;
                                frame_word_count <= 16'd0;
                            end else begin
                                rd_count_r <= rd_count_r + 16'd1;
                            end
                        end
                    end

                    // --------------------------------------------------------
                    // 异常丢弃：持续吞掉输入直到看到 pkt_last，完成重同步
                    // --------------------------------------------------------
                    ST_DROP: begin
                        if (s_fire && s_pkt_last) begin
                            state_r    <= ST_IDLE;
                            wr_count_r <= 16'd0;
                            rd_count_r <= 16'd0;
                        end
                    end

                    default: begin
                        state_r    <= ST_IDLE;
                        wr_count_r <= 16'd0;
                        rd_count_r <= 16'd0;
                    end
                endcase
            end
        end
    end

endmodule
