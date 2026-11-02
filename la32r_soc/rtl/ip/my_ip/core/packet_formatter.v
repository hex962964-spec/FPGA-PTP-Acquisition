`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/03/29
// Design Name:
// Module Name: packet_formatter
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
//   包格式化模块：
//   1) 先从 packet_info_fifo 读取一条包描述符
//   2) 输出固定 header
//   3) 再从 sample_fifo 连续读取对应数量的 payload 样本
//   4) 输出统一 packet stream：pkt_data / pkt_valid / pkt_last / pkt_ready
//
//   当前 header 采用 16bit 字宽，固定 10 个字：
//     word0 : 16'h55AA              magic
//     word1 : 16'h0001              version / flags(reserved)
//     word2 : seq_id[15:0]
//     word3 : seq_id[31:16]
//     word4 : channel_id[15:0]
//     word5 : sample_count[15:0]
//     word6 : timestamp[15:0]
//     word7 : timestamp[31:16]
//     word8 : timestamp[47:32]
//     word9 : timestamp[63:48]
//
//   本次修订目标：
//   - 保持“整包缓存后发送（store-and-forward）”的总体架构不变
//   - 修复原版本中 payload 发送断续的问题，使同一包在 pkt_data 处尽量连续输出
//   - 仍然兼容同步 FIFO，但将 rd_en 改为“组合地产生”，从而把 formatter 视角下的
//     读数等待从原来的多拍状态链路压缩到 1 个时钟周期
//
//   当前实现策略：
//   - 新增 clear_status 输入：仅清 sticky error_flag，不打断当前发送状态机
//
//
//   - packet_info_fifo / sample_fifo 仍然看作同步 FIFO
//   - packet_info_rd_en / sample_fifo_rd_en 不再是时序脉冲寄存器，而是根据当前状态
//     与握手条件组合地产生
//   - 在 header 末尾预取 payload：
//       * 发送 word8 时预取第 1 个 payload
//       * 发送 word9 时预取第 2 个 payload（若存在）
//   - 进入 payload 阶段后：
//       * 当前 payload 被下游接收的同一个时钟沿，装载下一个已经预取好的 payload
//       * 同时再去预取“下下个” payload（若存在）
//   - 因此在 pkt_ready=1 的情况下，可实现：
//       header 连续发送，payload 也连续发送
//
//   重要说明：
//   - 本模块的目标是“包一旦开始发送，就连续发完”
//   - 包与包之间仍会保留 descriptor 读取与状态切换带来的少量空拍
//   - 若未来还要进一步压缩包间空拍，可再继续做 packet_info_fifo 的前瞻预取
//
// Dependencies:
//
// Revision:
// Revision 0.03 - Payload streaming refined, keep store-and-forward architecture
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module packet_formatter(
        clk_in,
        resetn,
        enable,
        clear_status,

        sample_fifo_rd_en,
        sample_fifo_rd_data,
        sample_fifo_empty,

        packet_info_rd_en,
        packet_info_rd_data,
        packet_info_empty,

        pkt_data,
        pkt_valid,
        pkt_last,
        pkt_ready,

        busy,
        error_flag,
        packet_out_done,
        packet_out_timestamp
    );

    input               clk_in;
    input               resetn;
    input               enable;
    input               clear_status; // 1拍脉冲：仅清 sticky error_flag，不复位当前发送状态机

    // 注意：rd_en 仍通过模块端口输出，但本次修订改成“组合地产生”，
    // 使同步 FIFO 可以在当前时钟沿直接执行读操作，下一拍 formatter 即可使用 fifo_dout。
    output  reg         sample_fifo_rd_en;
    input       [15:0]  sample_fifo_rd_data;
    input               sample_fifo_empty;

    output  reg         packet_info_rd_en;
    input       [129:0] packet_info_rd_data;
    input               packet_info_empty;

    output  reg [15:0]  pkt_data;
    output  reg         pkt_valid;
    output  reg         pkt_last;
    input               pkt_ready;

    output              busy;
    output  reg         error_flag;
    output  reg         packet_out_done;
    output  reg [63:0]  packet_out_timestamp;

    // ------------------------------------------------------------------------
    // 状态定义
    //
    // ST_IDLE        : 等待有完整包 descriptor 到来
    // ST_INFO_REQ    : 对 packet_info_fifo 发起同步读请求
    // ST_INFO_LOAD   : 锁存 descriptor，并准备发送 header
    // ST_HEADER_SEND : 连续发送固定 header，并在末尾预取 payload
    // ST_PAYLOAD_SEND: 连续发送 payload，并边发边预取后续 payload
    // ------------------------------------------------------------------------
    localparam ST_IDLE          = 3'd0;
    localparam ST_INFO_REQ      = 3'd1;
    localparam ST_INFO_LOAD     = 3'd2;
    localparam ST_HEADER_SEND   = 3'd3;
    localparam ST_PAYLOAD_SEND  = 3'd4;

    localparam HEADER_WORD_NUM  = 4'd10;

    reg [2:0]  state;
    reg [3:0]  header_word_index;

    // payload_count_left 的语义：
    //   当前正在 pkt_data 上等待发送的 payload 以及其后所有未发送 payload 的总数。
    // 例：
    //   = 4  表示当前字是 payload0，后面还剩 3 个 payload
    //   = 1  表示当前字已经是本包最后一个 payload
    reg [15:0] payload_count_left;

    reg [31:0] current_seq_id;
    reg [15:0] current_channel_id;
    reg [15:0] current_sample_count;
    reg [63:0] current_timestamp;
    reg [1:0]  current_time_quality;

    assign busy = (state != ST_IDLE);

    // ------------------------------------------------------------------------
    // 根据当前包信息构造 header word
    // ------------------------------------------------------------------------
    function [15:0] build_header_word;
        input [3:0]  idx;
        input [1:0]  time_quality;
        input [31:0] seq_id;
        input [15:0] channel_id;
        input [15:0] sample_count;
        input [63:0] timestamp;
        begin
            case (idx)
                4'd0: build_header_word = 16'h55AA;
                4'd1: build_header_word = {6'd0, time_quality, 8'h02};
                4'd2: build_header_word = seq_id[15:0];
                4'd3: build_header_word = seq_id[31:16];
                4'd4: build_header_word = channel_id;
                4'd5: build_header_word = sample_count;
                4'd6: build_header_word = timestamp[15:0];
                4'd7: build_header_word = timestamp[31:16];
                4'd8: build_header_word = timestamp[47:32];
                4'd9: build_header_word = timestamp[63:48];
                default: build_header_word = 16'h0000;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------------
    // 组合读使能：
    // 1) ST_INFO_REQ      ：当前拍对 descriptor FIFO 发起读请求
    // 2) ST_HEADER_SEND   ：在 header 末尾预取 payload
    // 3) ST_PAYLOAD_SEND  ：当前 payload 被接收时，继续预取“后续一个” payload
    //
    // 这样同步 FIFO 会在“当前时钟沿”完成读指针前进与 fifo_dout 更新，
    // formatter 在下一拍就能拿到新数据，从而把同一包内部的 payload 做成连续发送。
    // ------------------------------------------------------------------------
    always @(*) begin
        sample_fifo_rd_en = 1'b0;
        packet_info_rd_en = 1'b0;

        case (state)
            ST_INFO_REQ: begin
                packet_info_rd_en = 1'b1;
            end

            ST_HEADER_SEND: begin
                if (pkt_valid && pkt_ready) begin
                    // 发送 header word8 时预取第 1 个 payload。
                    if ((header_word_index == (HEADER_WORD_NUM - 2)) &&
                        (current_sample_count != 16'd0))
                        sample_fifo_rd_en = 1'b1;

                    // 发送 header word9 时预取第 2 个 payload（若存在）。
                    else if ((header_word_index == (HEADER_WORD_NUM - 1)) &&
                             (current_sample_count > 16'd1))
                        sample_fifo_rd_en = 1'b1;
                end
            end

            ST_PAYLOAD_SEND: begin
                if (pkt_valid && pkt_ready && (payload_count_left > 16'd2)) begin
                    // 当前 payload 被接收时，如果后面还至少有两个 payload，
                    // 则预取“下下个” payload，保证下一拍仍可连续装载。
                    sample_fifo_rd_en = 1'b1;
                end
            end

            default: begin
                sample_fifo_rd_en = 1'b0;
                packet_info_rd_en = 1'b0;
            end
        endcase
    end

    // ------------------------------------------------------------------------
    // 主状态机
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn)
    if (!resetn) begin
        state                <= ST_IDLE;
        header_word_index    <= 4'd0;
        payload_count_left   <= 16'd0;

        current_seq_id       <= 32'd0;
        current_channel_id   <= 16'd0;
        current_sample_count <= 16'd0;
        current_timestamp    <= 64'd0;
        current_time_quality <= 2'd0;

        pkt_data             <= 16'd0;
        pkt_valid            <= 1'b0;
        pkt_last             <= 1'b0;

        error_flag           <= 1'b0;
        packet_out_done      <= 1'b0;
        packet_out_timestamp <= 64'd0;
    end else begin
        // 默认拉低单周期完成脉冲
        packet_out_done <= 1'b0;

        // clear_status 只清 sticky error_flag，不影响当前状态机与包发送上下文。
        // 这样软件可以在不停机时独立清 STATUS.error。
        if (clear_status)
            error_flag <= 1'b0;

        if (!enable) begin
            // enable=0 语义：停止发送并清空 formatter 内部状态
            state                <= ST_IDLE;
            header_word_index    <= 4'd0;
            payload_count_left   <= 16'd0;

            current_seq_id       <= 32'd0;
            current_channel_id   <= 16'd0;
            current_sample_count <= 16'd0;
            current_timestamp    <= 64'd0;
            current_time_quality <= 2'd0;

            pkt_data             <= 16'd0;
            pkt_valid            <= 1'b0;
            pkt_last             <= 1'b0;

            error_flag           <= 1'b0;
            packet_out_timestamp <= 64'd0;
        end else begin
            case (state)
                // ------------------------------------------------------------
                // 空闲态：等待 packet_info_fifo 中存在完整包 descriptor
                // ------------------------------------------------------------
                ST_IDLE: begin
                    pkt_valid <= 1'b0;
                    pkt_last  <= 1'b0;

                    if (!packet_info_empty)
                        state <= ST_INFO_REQ;
                end

                // ------------------------------------------------------------
                // 对 descriptor FIFO 发起同步读请求
                // 当前拍 packet_info_rd_en=1，下一拍即可在 packet_info_rd_data 看到结果
                // ------------------------------------------------------------
                ST_INFO_REQ: begin
                    state <= ST_INFO_LOAD;
                end

                // ------------------------------------------------------------
                // 锁存 descriptor，并装载 header 第 0 个字
                // ------------------------------------------------------------
                ST_INFO_LOAD: begin
                    current_time_quality <= packet_info_rd_data[129:128];
                    current_timestamp    <= packet_info_rd_data[127:64];
                    current_sample_count <= packet_info_rd_data[63:48];
                    current_channel_id   <= packet_info_rd_data[47:32];
                    current_seq_id       <= packet_info_rd_data[31:0];

                    header_word_index  <= 4'd0;
                    payload_count_left <= 16'd0;

                    pkt_data  <= build_header_word(4'd0,
                                                   packet_info_rd_data[129:128],
                                                   packet_info_rd_data[31:0],
                                                   packet_info_rd_data[47:32],
                                                   packet_info_rd_data[63:48],
                                                   packet_info_rd_data[127:64]);
                    pkt_valid <= 1'b1;
                    pkt_last  <= 1'b0;
                    state     <= ST_HEADER_SEND;
                end

                // ------------------------------------------------------------
                // 连续发送固定 header
                // 同时在 header 末尾两拍预取 payload
                // ------------------------------------------------------------
                ST_HEADER_SEND: begin
                    if (pkt_valid && pkt_ready) begin
                        // 保护性检查：若理论上应当能预取 payload，但 sample_fifo 已空，
                        // 则说明 descriptor 与 payload 发生了错位。
                        if ((header_word_index == (HEADER_WORD_NUM - 2)) &&
                            (current_sample_count != 16'd0) &&
                            sample_fifo_empty) begin
                            error_flag <= 1'b1;
                            pkt_valid  <= 1'b0;
                            pkt_last   <= 1'b0;
                            state      <= ST_IDLE;
                        end
                        else if ((header_word_index == (HEADER_WORD_NUM - 1)) &&
                                 (current_sample_count > 16'd1) &&
                                 sample_fifo_empty) begin
                            error_flag <= 1'b1;
                            pkt_valid  <= 1'b0;
                            pkt_last   <= 1'b0;
                            state      <= ST_IDLE;
                        end
                        else if (header_word_index == (HEADER_WORD_NUM - 1)) begin
                            // header 已发完
                            if (current_sample_count == 16'd0) begin
                                // 空 payload 包：直接结束本包
                                pkt_valid            <= 1'b0;
                                pkt_last             <= 1'b0;
                                payload_count_left   <= 16'd0;
                                packet_out_done      <= 1'b1;
                                packet_out_timestamp <= current_timestamp;
                                state                <= ST_IDLE;
                            end else begin
                                // 第 1 个 payload 已经在前一拍预取完成，
                                // 此处直接装载到 pkt_data，实现 header->payload 无断拍过渡。
                                payload_count_left <= current_sample_count;
                                pkt_data           <= sample_fifo_rd_data;
                                pkt_valid          <= 1'b1;
                                pkt_last           <= (current_sample_count == 16'd1);
                                state              <= ST_PAYLOAD_SEND;
                            end
                        end else begin
                            // header 尚未发完，继续发送下一个 header word
                            header_word_index <= header_word_index + 4'd1;
                            pkt_data          <= build_header_word(header_word_index + 4'd1,
                                                                  current_time_quality,
                                                                  current_seq_id,
                                                                  current_channel_id,
                                                                  current_sample_count,
                                                                  current_timestamp);
                            pkt_valid         <= 1'b1;
                            pkt_last          <= 1'b0;
                        end
                    end
                end

                // ------------------------------------------------------------
                // 连续发送 payload
                // 说明：
                //   - 当前 pkt_data 上的 payload 被接收后，下一拍直接装载 sample_fifo_rd_data
                //   - 若后面还至少有两个 payload，则当前拍同步预取“下下个” payload
                //   - 因此在 pkt_ready 连续有效时，payload 可以做到一拍一个字连续输出
                // ------------------------------------------------------------
                ST_PAYLOAD_SEND: begin
                    if (pkt_valid && pkt_ready) begin
                        // 若还需要继续预取，但 FIFO 却已空，则说明 descriptor / payload 失配。
                        if ((payload_count_left > 16'd2) && sample_fifo_empty) begin
                            error_flag         <= 1'b1;
                            pkt_valid          <= 1'b0;
                            pkt_last           <= 1'b0;
                            payload_count_left <= 16'd0;
                            state              <= ST_IDLE;
                        end
                        else if (payload_count_left == 16'd1) begin
                            // 当前 payload 已经是本包最后一个字，发送完成
                            pkt_valid            <= 1'b0;
                            pkt_last             <= 1'b0;
                            payload_count_left   <= 16'd0;
                            packet_out_done      <= 1'b1;
                            packet_out_timestamp <= current_timestamp;
                            state                <= ST_IDLE;
                        end else begin
                            // 当前 payload 发送完成后，下一拍直接切换到已经预取好的下一个 payload。
                            payload_count_left <= payload_count_left - 16'd1;
                            pkt_data           <= sample_fifo_rd_data;
                            pkt_valid          <= 1'b1;

                            // 当旧 payload_count_left == 2 时，说明当前字后面只剩最后一个 payload；
                            // 因此下一拍装载的这个 payload 需要拉高 pkt_last。
                            pkt_last           <= (payload_count_left == 16'd2);
                            state              <= ST_PAYLOAD_SEND;
                        end
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
