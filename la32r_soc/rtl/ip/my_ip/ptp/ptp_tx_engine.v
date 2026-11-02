`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: ptp_tx_engine
// -----------------------------------------------------------------------------
// 作用：
//   作为“PTP payload 生成器”，根据上层命令构造一整帧 PTPv2 payload，
//   然后以 16-bit word stream 的形式输出给后级 udp_tx_framer。
//
// 当前版本定位：
//   - 不直接负责 UDP/IP/Ethernet 头，只负责 PTP payload。
//   - 使用“单帧 store-and-forward”模型：先整帧写入本地 frame_mem，再输出。
//   - 首版重点支持：Sync / Delay_Req / Follow_Up / Delay_Resp。
//   - 其余 messageType 统一按 unsupported 处理，便于后续逐步扩展。
//
// 与现有工程的衔接方式：
//   ptp_tx_engine
//       -> udp_tx_framer(ptp path)
//       -> ipv4_tx_framer(ptp path)
//       -> tx_arbiter
//       -> top_phy
//
// 字段约定：
//   - cmd_timestamp_80 使用“PTP Timestamp raw field”布局：
//       [79:32] = secondsField[47:0]
//       [31:0]  = nanosecondsField[31:0]
//   - m_frame_type：
//       2'b01 = ptp_event
//       2'b10 = ptp_general
//   - m_need_timestamp：由上层命令决定。通常 Delay_Req / Sync 需要打 TX 戳。
//////////////////////////////////////////////////////////////////////////////////
module ptp_tx_engine #(
    parameter integer P_WORD_W    = 16,
    parameter integer P_MAX_WORDS = 32
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
    // 静态 PTP 配置（当前来自 wrapper / CSR）
    // ------------------------------------------------------------------------
    input  wire [7:0]               cfg_domain,
    input  wire [63:0]              cfg_clock_id,
    input  wire [15:0]              cfg_port_num,

    // ------------------------------------------------------------------------
    // 命令输入
    // cmd_valid/cmd_ready 握手成功后，本模块会把该命令构造成一整帧 PTP payload
    // ------------------------------------------------------------------------
    input  wire                     cmd_valid,
    output wire                     cmd_ready,
    input  wire [3:0]               cmd_message_type,
    input  wire [3:0]               cmd_transport_specific,
    input  wire                     cmd_need_timestamp,
    input  wire [15:0]              cmd_sequence_id,
    input  wire [15:0]              cmd_flags,
    input  wire [63:0]              cmd_correction_field,
    input  wire [7:0]               cmd_log_message_interval,
    input  wire [79:0]              cmd_timestamp_80,
    input  wire [63:0]              cmd_request_clock_id,
    input  wire [15:0]              cmd_request_port_num,

    // ------------------------------------------------------------------------
    // 输出给 udp_tx_framer 的“帧就绪 + word 流”接口
    // ------------------------------------------------------------------------
    output wire                     m_frame_ready,
    output reg  [15:0]              m_frame_word_count,
    input  wire                     m_frame_pop,

    output wire [P_WORD_W-1:0]      m_word_data,
    output wire                     m_word_valid,
    output wire                     m_word_last,
    input  wire                     m_word_ready,

    // 与当前 frame 绑定的元信息（给下游仲裁 / top_phy 用）
    output reg                      m_need_timestamp,
    output reg  [1:0]               m_frame_type,
    output reg  [15:0]              m_active_sequence_id,

    // ------------------------------------------------------------------------
    // 状态 / 统计
    // ------------------------------------------------------------------------
    output wire                     busy,
    output reg  [31:0]              stat_frame_accept_count,
    output reg  [31:0]              stat_frame_drop_count,
    output reg  [15:0]              stat_last_error
);

    localparam [1:0] ST_IDLE  = 2'd0;
    localparam [1:0] ST_READY = 2'd1;
    localparam [1:0] ST_DRAIN = 2'd2;

    localparam [15:0] ERR_NONE             = 16'h0000;
    localparam [15:0] ERR_UNSUPPORTED_MSG  = 16'h0001;
    localparam [15:0] ERR_FRAME_TOO_LONG   = 16'h0002;

    // 当前首版支持的 messageType
    localparam [3:0] MSG_SYNC       = 4'h0;
    localparam [3:0] MSG_DELAY_REQ  = 4'h1;
    localparam [3:0] MSG_FOLLOW_UP  = 4'h8;
    localparam [3:0] MSG_DELAY_RESP = 4'h9;

    reg [1:0] state_r;
    reg [15:0] rd_count_r;

    reg [P_WORD_W-1:0] frame_mem [0:P_MAX_WORDS-1];

    integer i;

    function [7:0] f_control_field;
        input [3:0] msg_type;
        begin
            case (msg_type)
                MSG_SYNC:       f_control_field = 8'h00;
                MSG_DELAY_REQ:  f_control_field = 8'h01;
                MSG_FOLLOW_UP:  f_control_field = 8'h02;
                MSG_DELAY_RESP: f_control_field = 8'h03;
                default:        f_control_field = 8'h05;
            endcase
        end
    endfunction

    function [15:0] f_message_length_bytes;
        input [3:0] msg_type;
        begin
            case (msg_type)
                MSG_SYNC,
                MSG_DELAY_REQ,
                MSG_FOLLOW_UP:  f_message_length_bytes = 16'd44; // 34-byte common header + 10-byte timestamp field
                MSG_DELAY_RESP: f_message_length_bytes = 16'd54; // 34-byte common header + 10-byte receiveTimestamp + 10-byte requestingPortIdentity
                default:        f_message_length_bytes = 16'd0;
            endcase
        end
    endfunction

    function [15:0] f_message_length_words;
        input [3:0] msg_type;
        begin
            case (msg_type)
                MSG_SYNC,
                MSG_DELAY_REQ,
                MSG_FOLLOW_UP:  f_message_length_words = 16'd22;
                MSG_DELAY_RESP: f_message_length_words = 16'd27;
                default:        f_message_length_words = 16'd0;
            endcase
        end
    endfunction

    function [1:0] f_frame_type;
        input [3:0] msg_type;
        begin
            case (msg_type)
                MSG_SYNC,
                MSG_DELAY_REQ:  f_frame_type = 2'b01; // ptp_event
                MSG_FOLLOW_UP,
                MSG_DELAY_RESP: f_frame_type = 2'b10; // ptp_general
                default:        f_frame_type = 2'b00;
            endcase
        end
    endfunction

    function f_message_supported;
        input [3:0] msg_type;
        begin
            case (msg_type)
                MSG_SYNC,
                MSG_DELAY_REQ,
                MSG_FOLLOW_UP,
                MSG_DELAY_RESP: f_message_supported = 1'b1;
                default:        f_message_supported = 1'b0;
            endcase
        end
    endfunction

    task t_fill_timestamp_5w;
        input integer base_idx;
        input [79:0] ts80;
        begin
            frame_mem[base_idx + 0] <= ts80[79:64];
            frame_mem[base_idx + 1] <= ts80[63:48];
            frame_mem[base_idx + 2] <= ts80[47:32];
            frame_mem[base_idx + 3] <= ts80[31:16];
            frame_mem[base_idx + 4] <= ts80[15:0];
        end
    endtask

    task t_fill_port_identity_5w;
        input integer base_idx;
        input [63:0]  clock_id;
        input [15:0]  port_num;
        begin
            frame_mem[base_idx + 0] <= clock_id[63:48];
            frame_mem[base_idx + 1] <= clock_id[47:32];
            frame_mem[base_idx + 2] <= clock_id[31:16];
            frame_mem[base_idx + 3] <= clock_id[15:0];
            frame_mem[base_idx + 4] <= port_num;
        end
    endtask

    task t_fill_common_header;
        input [3:0]  message_type;
        input [3:0]  transport_specific;
        input [15:0] flags;
        input [63:0] correction_field;
        input [15:0] sequence_id;
        input [7:0]  log_message_interval;
        reg [15:0]   msg_len_b;
        begin
            msg_len_b = f_message_length_bytes(message_type);

            // Word 0..16 = 34-byte common header
            frame_mem[0]  <= {{transport_specific, message_type}, 8'h02}; // byte0={tspec,msgType}, byte1={reserved[7:4],versionPTP=2}
            frame_mem[1]  <= msg_len_b;
            frame_mem[2]  <= {cfg_domain, 8'h00};
            frame_mem[3]  <= flags;
            frame_mem[4]  <= correction_field[63:48];
            frame_mem[5]  <= correction_field[47:32];
            frame_mem[6]  <= correction_field[31:16];
            frame_mem[7]  <= correction_field[15:0];
            frame_mem[8]  <= 16'h0000; // reserved3[31:16]
            frame_mem[9]  <= 16'h0000; // reserved3[15:0]
            frame_mem[10] <= cfg_clock_id[63:48];
            frame_mem[11] <= cfg_clock_id[47:32];
            frame_mem[12] <= cfg_clock_id[31:16];
            frame_mem[13] <= cfg_clock_id[15:0];
            frame_mem[14] <= cfg_port_num;
            frame_mem[15] <= sequence_id;
            frame_mem[16] <= {f_control_field(message_type), log_message_interval};
        end
    endtask

    wire cmd_fire;
    assign cmd_ready     = cfg_enable && (state_r == ST_IDLE);
    assign cmd_fire      = cmd_valid && cmd_ready;
    assign m_frame_ready = (state_r == ST_READY);
    assign m_word_valid  = (state_r == ST_DRAIN);
    assign m_word_data   = (state_r == ST_DRAIN) ? frame_mem[rd_count_r] : {P_WORD_W{1'b0}};
    assign m_word_last   = (state_r == ST_DRAIN) && (rd_count_r == (m_frame_word_count - 16'd1));
    assign busy          = (state_r != ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r                  <= ST_IDLE;
            rd_count_r               <= 16'd0;
            m_frame_word_count       <= 16'd0;
            m_need_timestamp         <= 1'b0;
            m_frame_type             <= 2'b00;
            m_active_sequence_id     <= 16'd0;
            stat_frame_accept_count  <= 32'd0;
            stat_frame_drop_count    <= 32'd0;
            stat_last_error          <= ERR_NONE;
            for (i = 0; i < P_MAX_WORDS; i = i + 1)
                frame_mem[i] <= {P_WORD_W{1'b0}};
        end else begin
            if (cfg_soft_reset || !cfg_enable) begin
                state_r            <= ST_IDLE;
                rd_count_r         <= 16'd0;
                m_frame_word_count <= 16'd0;
                m_need_timestamp   <= 1'b0;
                m_frame_type       <= 2'b00;
            end else begin
                if (cfg_clear_status) begin
                    stat_frame_accept_count <= 32'd0;
                    stat_frame_drop_count   <= 32'd0;
                    stat_last_error         <= ERR_NONE;
                end

                case (state_r)
                    ST_IDLE: begin
                        rd_count_r <= 16'd0;
                        if (cmd_fire) begin
                            for (i = 0; i < P_MAX_WORDS; i = i + 1)
                                frame_mem[i] <= {P_WORD_W{1'b0}};

                            if (!f_message_supported(cmd_message_type)) begin
                                stat_frame_drop_count <= stat_frame_drop_count + 32'd1;
                                stat_last_error       <= ERR_UNSUPPORTED_MSG;
                            end else if (f_message_length_words(cmd_message_type) > P_MAX_WORDS) begin
                                stat_frame_drop_count <= stat_frame_drop_count + 32'd1;
                                stat_last_error       <= ERR_FRAME_TOO_LONG;
                            end else begin
                                t_fill_common_header(
                                    cmd_message_type,
                                    cmd_transport_specific,
                                    cmd_flags,
                                    cmd_correction_field,
                                    cmd_sequence_id,
                                    cmd_log_message_interval
                                );

                                case (cmd_message_type)
                                    MSG_SYNC,
                                    MSG_DELAY_REQ,
                                    MSG_FOLLOW_UP: begin
                                        t_fill_timestamp_5w(17, cmd_timestamp_80);
                                    end

                                    MSG_DELAY_RESP: begin
                                        t_fill_timestamp_5w(17, cmd_timestamp_80);
                                        t_fill_port_identity_5w(22, cmd_request_clock_id, cmd_request_port_num);
                                    end

                                    default: begin
                                        // unreachable due to support check
                                    end
                                endcase

                                m_frame_word_count      <= f_message_length_words(cmd_message_type);
                                m_need_timestamp        <= cmd_need_timestamp;
                                m_frame_type            <= f_frame_type(cmd_message_type);
                                m_active_sequence_id    <= cmd_sequence_id;
                                stat_frame_accept_count <= stat_frame_accept_count + 32'd1;
                                stat_last_error         <= ERR_NONE;
                                state_r                 <= ST_READY;
                            end
                        end
                    end

                    ST_READY: begin
                        if (m_frame_pop) begin
                            rd_count_r <= 16'd0;
                            state_r    <= ST_DRAIN;
                        end
                    end

                    ST_DRAIN: begin
                        if (m_word_valid && m_word_ready) begin
                            if (rd_count_r == (m_frame_word_count - 16'd1)) begin
                                rd_count_r <= 16'd0;
                                state_r    <= ST_IDLE;
                            end else begin
                                rd_count_r <= rd_count_r + 16'd1;
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
