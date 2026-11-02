`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// top_phy_axis_mac_tx
// -----------------------------------------------------------------------------
// 作用：
//   1) 接收来自 net_acq 后端的 8-bit AXIS-like 字节流（不含 preamble/SFD/FCS）
//   2) 插入 7x55 + D5 preamble/SFD
//   3) 对 MAC 帧正文计算 Ethernet CRC32，并在末尾追加 4-byte FCS
//   4) 保证最小帧长（不含 FCS 至少 60 bytes）
//   5) 插入 12-byte IFG
//
// 当前版本定位：
//   - transmit-only
//   - 假设上游在一帧内能持续提供数据（当 tready 恒高时不插空）
//   - tuser[0] = SOF, tuser[1] = need_timestamp, tuser[3:2] = frame_type
//   - 只有 PTP event 帧才允许打 TX 时间戳
//   - 时间戳锁存点就是 SFD 发出的那个拍
//////////////////////////////////////////////////////////////////////////////////
module top_phy_axis_mac_tx #(
    parameter integer MIN_FRAME_BYTES = 60,
    parameter integer IFG_BYTES       = 12
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         tx_enable,

    input  wire [7:0]   s_axis_tdata,
    input  wire         s_axis_tvalid,
    input  wire         s_axis_tlast,
    output wire         s_axis_tready,
    input  wire [3:0]   s_axis_tuser,
    input  wire [63:0]  ptp_tod_time_ns,
    input  wire         ptp_tod_time_valid,

    output reg  [7:0]   gmii_txd,
    output reg          gmii_tx_en,
    output reg          gmii_tx_er,
    output reg  [63:0]  ptp_txstamp_ns,
    output reg          ptp_txstamp_valid,

    output wire         tx_busy,
    output reg  [31:0]  stat_frame_count,
    output reg  [31:0]  stat_mac_byte_count,
    output reg  [15:0]  stat_last_error
);

    localparam [2:0] ST_IDLE     = 3'd0;
    localparam [2:0] ST_PREAMBLE = 3'd1;
    localparam [2:0] ST_DATA     = 3'd2;
    localparam [2:0] ST_PAD      = 3'd3;
    localparam [2:0] ST_FCS      = 3'd4;
    localparam [2:0] ST_IFG      = 3'd5;
    localparam [2:0] ST_ABORT    = 3'd6;

    localparam [15:0] ERR_NONE           = 16'h0000;
    localparam [15:0] ERR_DATA_UNDERFLOW = 16'h0001;

    localparam [1:0] FRAME_TYPE_DATA      = 2'b00;
    localparam [1:0] FRAME_TYPE_PTP_EVENT = 2'b01;

    reg  [2:0]   state_r;
    reg  [2:0]   preamble_idx_r;
    reg  [3:0]   ifg_cnt_r;
    reg  [1:0]   fcs_idx_r;
    reg  [15:0]  frame_len_r;
    reg  [15:0]  pad_remaining_r;
    reg  [31:0]  crc_r;
    reg  [31:0]  fcs_r;

    reg  [7:0]   buf_data_r;
    reg          buf_last_r;
    reg          buf_valid_r;
    reg          frame_need_timestamp_r;
    reg  [1:0]   frame_type_r;

    wire [31:0]  crc_after_data_w;
    wire [31:0]  crc_after_pad_w;
    wire         s_axis_sof_w;
    wire         s_axis_fire_w;
    wire         frame_timestamp_this_frame_w;

    assign tx_busy = (state_r != ST_IDLE);
    assign s_axis_sof_w = s_axis_tuser[0];
    assign frame_timestamp_this_frame_w = frame_need_timestamp_r &&
                                          (frame_type_r == FRAME_TYPE_PTP_EVENT) &&
                                          ptp_tod_time_valid;

    // 约束：
    //   - IDLE：只在 SOF 到达时接首字节并缓存
    //   - PREAMBLE/PAD/FCS/IFG/ABORT：不接收上游
    //   - DATA：仅当当前缓存字节不是本帧最后 1 byte 时，才在同拍接收下一个字节
    //           这样可避免“发送上一帧最后 1 byte 的同一拍，又错误握手下一帧首字节”
    assign s_axis_tready = tx_enable &&
                           (((state_r == ST_IDLE) && s_axis_sof_w) ||
                            ((state_r == ST_DATA) && !buf_last_r));
    assign s_axis_fire_w = s_axis_tvalid && s_axis_tready;

    assign crc_after_data_w = crc32_next_d8(crc_r, buf_data_r);
    assign crc_after_pad_w  = crc32_next_d8(crc_r, 8'h00);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r                    <= ST_IDLE;
            preamble_idx_r             <= 3'd0;
            ifg_cnt_r                  <= 4'd0;
            fcs_idx_r                  <= 2'd0;
            frame_len_r                <= 16'd0;
            pad_remaining_r            <= 16'd0;
            crc_r                      <= 32'hFFFF_FFFF;
            fcs_r                      <= 32'd0;
            buf_data_r                 <= 8'd0;
            buf_last_r                 <= 1'b0;
            buf_valid_r                <= 1'b0;
            frame_need_timestamp_r     <= 1'b0;
            frame_type_r               <= FRAME_TYPE_DATA;
            gmii_txd                   <= 8'd0;
            gmii_tx_en                 <= 1'b0;
            gmii_tx_er                 <= 1'b0;
            ptp_txstamp_ns             <= 64'd0;
            ptp_txstamp_valid          <= 1'b0;
            stat_frame_count           <= 32'd0;
            stat_mac_byte_count        <= 32'd0;
            stat_last_error            <= ERR_NONE;
        end else begin
            // 默认空闲输出；在具体状态里覆盖。
            gmii_txd          <= 8'd0;
            gmii_tx_en        <= 1'b0;
            gmii_tx_er        <= 1'b0;
            ptp_txstamp_valid <= 1'b0;

            if (!tx_enable) begin
                state_r                    <= ST_IDLE;
                preamble_idx_r             <= 3'd0;
                ifg_cnt_r                  <= 4'd0;
                fcs_idx_r                  <= 2'd0;
                frame_len_r                <= 16'd0;
                pad_remaining_r            <= 16'd0;
                crc_r                      <= 32'hFFFF_FFFF;
                fcs_r                      <= 32'd0;
                buf_data_r                 <= 8'd0;
                buf_last_r                 <= 1'b0;
                buf_valid_r                <= 1'b0;
                frame_need_timestamp_r     <= 1'b0;
                frame_type_r               <= FRAME_TYPE_DATA;
                end else begin
                case (state_r)
                    ST_IDLE: begin
                        crc_r       <= 32'hFFFF_FFFF;
                        frame_len_r <= 16'd0;
                        if (s_axis_fire_w) begin
                            buf_data_r                 <= s_axis_tdata;
                            buf_last_r                 <= s_axis_tlast;
                            buf_valid_r                <= 1'b1;
                            frame_need_timestamp_r     <= s_axis_tuser[1];
                            frame_type_r               <= s_axis_tuser[3:2];
                            preamble_idx_r             <= 3'd0;
                            stat_last_error            <= ERR_NONE;
                            state_r                    <= ST_PREAMBLE;
                        end
                    end

                    ST_PREAMBLE: begin
                        gmii_tx_en <= 1'b1;
                        gmii_txd   <= (preamble_idx_r == 3'd7) ? 8'hD5 : 8'h55;
                        if (preamble_idx_r == 3'd7) begin
                            // 只有 PTP event 帧才打 TX timestamp，
                            // 且锁存点就是 SFD 发出的这个拍。
                            if (frame_timestamp_this_frame_w) begin
                                ptp_txstamp_ns    <= ptp_tod_time_ns;
                                ptp_txstamp_valid <= 1'b1;
                            end
                            state_r <= ST_DATA;
                        end
                        preamble_idx_r <= preamble_idx_r + 3'd1;
                    end

                    ST_DATA: begin
                        gmii_tx_en <= 1'b1;
                        gmii_txd   <= buf_data_r;
                        crc_r      <= crc_after_data_w;
                        frame_len_r <= frame_len_r + 16'd1;
                        stat_mac_byte_count <= stat_mac_byte_count + 32'd1;

                        if (buf_last_r) begin
                            buf_valid_r <= 1'b0;
                            if ((frame_len_r + 16'd1) < MIN_FRAME_BYTES[15:0]) begin
                                pad_remaining_r <= MIN_FRAME_BYTES[15:0] - (frame_len_r + 16'd1);
                                state_r         <= ST_PAD;
                            end else begin
                                fcs_r            <= ~crc_after_data_w;
                                fcs_idx_r        <= 2'd0;
                                stat_frame_count <= stat_frame_count + 32'd1;
                                state_r          <= ST_FCS;
                            end
                        end else if (s_axis_fire_w) begin
                            buf_data_r  <= s_axis_tdata;
                            buf_last_r  <= s_axis_tlast;
                            buf_valid_r <= 1'b1;
                        end else begin
                            // 理论上 net_acq 后端在一帧内若 tready 恒高，应连续吐数。
                            // 若这里发生 underflow，则直接中止本帧并进入 IFG。
                            buf_valid_r     <= 1'b0;
                            stat_last_error <= ERR_DATA_UNDERFLOW;
                            state_r         <= ST_ABORT;
                        end
                    end

                    ST_PAD: begin
                        gmii_tx_en <= 1'b1;
                        gmii_txd   <= 8'h00;
                        crc_r      <= crc_after_pad_w;
                        frame_len_r <= frame_len_r + 16'd1;
                        stat_mac_byte_count <= stat_mac_byte_count + 32'd1;

                        if (pad_remaining_r == 16'd1) begin
                            fcs_r            <= ~crc_after_pad_w;
                            fcs_idx_r        <= 2'd0;
                            pad_remaining_r  <= 16'd0;
                            stat_frame_count <= stat_frame_count + 32'd1;
                            state_r          <= ST_FCS;
                        end else begin
                            pad_remaining_r <= pad_remaining_r - 16'd1;
                        end
                    end

                    ST_FCS: begin
                        gmii_tx_en <= 1'b1;
                        case (fcs_idx_r)
                            2'd0: gmii_txd <= fcs_r[7:0];
                            2'd1: gmii_txd <= fcs_r[15:8];
                            2'd2: gmii_txd <= fcs_r[23:16];
                            default: gmii_txd <= fcs_r[31:24];
                        endcase

                        if (fcs_idx_r == 2'd3) begin
                            ifg_cnt_r <= (IFG_BYTES > 0) ? (IFG_BYTES - 1) : 0;
                            state_r   <= ST_IFG;
                        end else begin
                            fcs_idx_r <= fcs_idx_r + 2'd1;
                        end
                    end

                    ST_ABORT: begin
                        gmii_tx_en <= 1'b0;
                        gmii_tx_er <= 1'b0;
                        ifg_cnt_r  <= (IFG_BYTES > 0) ? (IFG_BYTES - 1) : 0;
                        state_r    <= ST_IFG;
                    end

                    ST_IFG: begin
                        gmii_tx_en <= 1'b0;
                        gmii_tx_er <= 1'b0;
                        if (ifg_cnt_r == 4'd0) begin
                            state_r <= ST_IDLE;
                        end else begin
                            ifg_cnt_r <= ifg_cnt_r - 4'd1;
                        end
                    end

                    default: begin
                        state_r <= ST_IDLE;
                    end
                endcase
            end
        end
    end

    function [31:0] crc32_next_d8;
        input [31:0] crc_in;
        input [7:0]  data_in;
        integer i;
        reg [31:0] crc_tmp;
        reg [7:0]  data_tmp;
        begin
            crc_tmp  = crc_in;
            data_tmp = data_in;
            for (i = 0; i < 8; i = i + 1) begin
                if ((crc_tmp[0] ^ data_tmp[0]) == 1'b1) begin
                    crc_tmp = (crc_tmp >> 1) ^ 32'hEDB8_8320;
                end else begin
                    crc_tmp = (crc_tmp >> 1);
                end
                data_tmp = data_tmp >> 1;
            end
            crc32_next_d8 = crc_tmp;
        end
    endfunction

endmodule
