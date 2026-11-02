`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: ptp_rx_async_fifo
// -----------------------------------------------------------------------------
// Purpose:
//   Small asynchronous FIFO used by the rewritten PTP RX path.
//
// Placement:
//   ptp_rx_filter (eth_rx_clk) -> ptp_rx_async_fifo -> ptp_rx_mailbox (sys_clk)
//
// Features:
//   - Gray-pointer async FIFO
//   - Generic payload width
//   - Sticky overflow indication on the write side, mirrored into the read side
//   - Ready/valid interfaces on both ends
//
// Notes:
//   - P_DEPTH must be a power of two.
//   - The overflow sticky bit is intended to feed the software-visible mailbox
//     overflow status. It indicates that at least one parsed RX record was lost
//     because the FIFO had no free entry.
//
// CDC / timing 收口说明：
//   - Gray 指针同步链与 overflow 跨域同步链都显式标注 ASYNC_REG；
//   - 不改变 FIFO 功能与 mailbox 侧可见语义。
//////////////////////////////////////////////////////////////////////////////////
module ptp_rx_async_fifo #(
    parameter integer P_DATA_W = 256,
    parameter integer P_DEPTH  = 8
)(
    input  wire                 wr_clk,
    input  wire                 wr_resetn,
    input  wire                 wr_clear,
    input  wire [P_DATA_W-1:0]  s_data,
    input  wire                 s_valid,
    output wire                 s_ready,
    output wire                 wr_full,
    output reg                  wr_overflow_sticky,

    input  wire                 rd_clk,
    input  wire                 rd_resetn,
    input  wire                 rd_clear,
    output wire [P_DATA_W-1:0]  m_data,
    output wire                 m_valid,
    input  wire                 m_ready,
    output wire                 rd_empty,
    output reg                  rd_overflow_sticky
);

    function integer f_clog2;
        input integer value;
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1)
                value = value >> 1;
            f_clog2 = i;
        end
    endfunction

    localparam integer C_ADDR_W = f_clog2(P_DEPTH);
    localparam integer C_PTR_W  = C_ADDR_W + 1;

    // Synchronous read port keeps mailbox data entirely in rd_clk.
    (* ram_style = "block" *) reg [P_DATA_W-1:0] mem [0:P_DEPTH-1];
    reg [P_DATA_W-1:0] rd_data_r;

    reg [C_PTR_W-1:0] wr_bin_r;
    reg [C_PTR_W-1:0] wr_gray_r;
    reg [C_PTR_W-1:0] rd_bin_r;
    reg [C_PTR_W-1:0] rd_gray_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [C_PTR_W-1:0] rd_gray_sync1_wr_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [C_PTR_W-1:0] rd_gray_sync2_wr_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [C_PTR_W-1:0] wr_gray_sync1_rd_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [C_PTR_W-1:0] wr_gray_sync2_rd_r;

    reg               wr_full_r;
    reg               rd_empty_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg wr_overflow_sync1_rd_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg wr_overflow_sync2_rd_r;

    wire               do_write;
    wire               do_read;

    wire [C_PTR_W-1:0] wr_bin_next;
    wire [C_PTR_W-1:0] wr_gray_next;
    wire [C_PTR_W-1:0] rd_bin_next;
    wire [C_PTR_W-1:0] rd_gray_next;

    wire               wr_full_next;
    wire               rd_empty_next;

    assign s_ready  = !wr_full_r;
    assign wr_full  = wr_full_r;
    assign m_valid  = !rd_empty_r;
    assign rd_empty = rd_empty_r;

    assign do_write = s_valid && s_ready;
    assign do_read  = m_valid && m_ready;

    assign wr_bin_next  = wr_bin_r + {{(C_PTR_W-1){1'b0}}, do_write};
    assign wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
    assign rd_bin_next  = rd_bin_r + {{(C_PTR_W-1){1'b0}}, do_read};
    assign rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;

    assign wr_full_next = (wr_gray_next == {
        ~rd_gray_sync2_wr_r[C_PTR_W-1:C_PTR_W-2],
         rd_gray_sync2_wr_r[C_PTR_W-3:0]
    });

    assign rd_empty_next = (rd_gray_next == wr_gray_sync2_rd_r);

    assign m_data = rd_data_r;

    // Pointer publication is unchanged: space is released only on m_valid &&
    // m_ready. The next selected word is prefetched on the same consuming edge.
    always @(posedge wr_clk) begin
        if (wr_resetn && !wr_clear && do_write)
            mem[wr_bin_r[C_ADDR_W-1:0]] <= s_data;
    end

    always @(posedge rd_clk) begin
        if (rd_resetn && !rd_clear && !rd_empty_next)
            rd_data_r <= mem[rd_bin_next[C_ADDR_W-1:0]];
    end

    always @(posedge wr_clk or negedge wr_resetn) begin
        if (!wr_resetn) begin
            wr_bin_r           <= {C_PTR_W{1'b0}};
            wr_gray_r          <= {C_PTR_W{1'b0}};
            rd_gray_sync1_wr_r <= {C_PTR_W{1'b0}};
            rd_gray_sync2_wr_r <= {C_PTR_W{1'b0}};
            wr_full_r          <= 1'b0;
            wr_overflow_sticky <= 1'b0;
        end else begin
            rd_gray_sync1_wr_r <= rd_gray_r;
            rd_gray_sync2_wr_r <= rd_gray_sync1_wr_r;

            if (wr_clear) begin
                wr_bin_r           <= {C_PTR_W{1'b0}};
                wr_gray_r          <= {C_PTR_W{1'b0}};
                wr_full_r          <= 1'b0;
                wr_overflow_sticky <= 1'b0;
            end else begin
                if (s_valid && !s_ready)
                    wr_overflow_sticky <= 1'b1;

                wr_bin_r  <= wr_bin_next;
                wr_gray_r <= wr_gray_next;
                wr_full_r <= wr_full_next;
            end
        end
    end

    always @(posedge rd_clk or negedge rd_resetn) begin
        if (!rd_resetn) begin
            rd_bin_r             <= {C_PTR_W{1'b0}};
            rd_gray_r            <= {C_PTR_W{1'b0}};
            wr_gray_sync1_rd_r   <= {C_PTR_W{1'b0}};
            wr_gray_sync2_rd_r   <= {C_PTR_W{1'b0}};
            rd_empty_r           <= 1'b1;
            wr_overflow_sync1_rd_r <= 1'b0;
            wr_overflow_sync2_rd_r <= 1'b0;
            rd_overflow_sticky   <= 1'b0;
        end else begin
            wr_gray_sync1_rd_r     <= wr_gray_r;
            wr_gray_sync2_rd_r     <= wr_gray_sync1_rd_r;
            wr_overflow_sync1_rd_r <= wr_overflow_sticky;
            wr_overflow_sync2_rd_r <= wr_overflow_sync1_rd_r;

            if (rd_clear) begin
                rd_bin_r            <= {C_PTR_W{1'b0}};
                rd_gray_r           <= {C_PTR_W{1'b0}};
                rd_empty_r          <= 1'b1;
                rd_overflow_sticky  <= 1'b0;
            end else begin
                if (wr_overflow_sync2_rd_r)
                    rd_overflow_sticky <= 1'b1;

                rd_bin_r   <= rd_bin_next;
                rd_gray_r  <= rd_gray_next;
                rd_empty_r <= rd_empty_next;
            end
        end
    end

endmodule
