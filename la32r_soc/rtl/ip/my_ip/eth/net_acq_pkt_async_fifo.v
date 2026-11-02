`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_pkt_async_fifo
// Description:
//   sys_clk(50MHz) -> eth_tx_clk(125MHz) 的包流异步 FIFO。
//
//   功能定位：
//     - 位于 packet_formatter 之后、net_acq_eth_backend_top 之前；
//     - 传输对象为 16-bit pkt_data + 1-bit pkt_last；
//     - 写侧 backpressure 由 FIFO full 决定；
//     - 读侧按标准 ready/valid 消费。
//
//   说明：
//     - 当前实现采用 Gray pointer 异步 FIFO；
//     - 默认深度 1024（建议值，满足现阶段系统带宽缓冲需求）；
//     - 为保持逻辑简单，这里要求 P_DEPTH 为 2 的幂；
//     - 同步双时钟RAM读端预取，避免存储体组合读MUX直接跨域进入消费逻辑。
//
//   CDC / timing 收口说明：
//     - Gray 指针同步链显式标注 ASYNC_REG / SHREG_EXTRACT；
//     - 不改变 FIFO 行为和外部 ready/valid 合同。
//////////////////////////////////////////////////////////////////////////////////

module net_acq_pkt_async_fifo #(
    parameter integer P_WORD_W = 16,
    parameter integer P_DEPTH  = 1024
)(
    // ------------------------------------------------------------------------
    // write side : sys_clk
    // ------------------------------------------------------------------------
    input  wire                 wr_clk,
    input  wire                 wr_resetn,
    input  wire                 wr_clear,
    input  wire [P_WORD_W-1:0]  s_pkt_data,
    input  wire                 s_pkt_valid,
    input  wire                 s_pkt_last,
    output wire                 s_pkt_ready,
    output wire                 wr_empty,
    output wire                 wr_full,

    // ------------------------------------------------------------------------
    // read side : eth_tx_clk
    // ------------------------------------------------------------------------
    input  wire                 rd_clk,
    input  wire                 rd_resetn,
    input  wire                 rd_clear,
    output wire [P_WORD_W-1:0]  m_pkt_data,
    output wire                 m_pkt_valid,
    output wire                 m_pkt_last,
    input  wire                 m_pkt_ready,
    output wire                 rd_empty
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

    // ------------------------------------------------------------------------
    // 存储体：{pkt_last, pkt_data}
    // ------------------------------------------------------------------------
    // True dual-clock RAM. No payload reset: rd_empty_r qualifies every output.
    (* ram_style = "block" *) reg [P_WORD_W:0] mem [0:P_DEPTH-1];
    reg [P_WORD_W:0] rd_data_r;

    // ------------------------------------------------------------------------
    // 指针（binary + gray）
    // ------------------------------------------------------------------------
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

    wire               do_write;
    wire               do_read;

    wire [C_PTR_W-1:0] wr_bin_next;
    wire [C_PTR_W-1:0] wr_gray_next;
    wire [C_PTR_W-1:0] rd_bin_next;
    wire [C_PTR_W-1:0] rd_gray_next;

    wire               wr_full_next;
    wire               rd_empty_next;

    assign s_pkt_ready = !wr_full_r;
    assign wr_full     = wr_full_r;
    assign wr_empty    = (wr_gray_r == rd_gray_sync2_wr_r);

    assign m_pkt_valid = !rd_empty_r;
    assign rd_empty    = rd_empty_r;

    assign do_write = s_pkt_valid && s_pkt_ready;
    assign do_read  = m_pkt_valid && m_pkt_ready;

    assign wr_bin_next  = wr_bin_r + {{(C_PTR_W-1){1'b0}}, do_write};
    assign wr_gray_next = (wr_bin_next >> 1) ^ wr_bin_next;
    assign rd_bin_next  = rd_bin_r + {{(C_PTR_W-1){1'b0}}, do_read};
    assign rd_gray_next = (rd_bin_next >> 1) ^ rd_bin_next;

    assign wr_full_next = (wr_gray_next == {
        ~rd_gray_sync2_wr_r[C_PTR_W-1:C_PTR_W-2],
         rd_gray_sync2_wr_r[C_PTR_W-3:0]
    });

    assign rd_empty_next = (rd_gray_next == wr_gray_sync2_rd_r);

    // Prefetch the next unconsumed word on the read clock. The RAM output and
    // rd_empty_r update on the same edge, preserving FWFT ready/valid behavior.
    // A stalled word retains its address and value; a consumed word selects the
    // next address without a bubble when another synchronized entry is present.
    assign {m_pkt_last, m_pkt_data} = rd_data_r;

    always @(posedge wr_clk) begin
        if (wr_resetn && !wr_clear && do_write)
            mem[wr_bin_r[C_ADDR_W-1:0]] <= {s_pkt_last, s_pkt_data};
    end

    always @(posedge rd_clk) begin
        if (rd_resetn && !rd_clear && !rd_empty_next)
            rd_data_r <= mem[rd_bin_next[C_ADDR_W-1:0]];
    end

    // ------------------------------------------------------------------------
    // 写时钟域：同步读指针 Gray、写存储体、推进写指针
    // ------------------------------------------------------------------------
    always @(posedge wr_clk or negedge wr_resetn) begin
        if (!wr_resetn) begin
            wr_bin_r           <= {C_PTR_W{1'b0}};
            wr_gray_r          <= {C_PTR_W{1'b0}};
            rd_gray_sync1_wr_r <= {C_PTR_W{1'b0}};
            rd_gray_sync2_wr_r <= {C_PTR_W{1'b0}};
            wr_full_r          <= 1'b0;
        end else begin
            rd_gray_sync1_wr_r <= rd_gray_r;
            rd_gray_sync2_wr_r <= rd_gray_sync1_wr_r;

            if (wr_clear) begin
                wr_bin_r  <= {C_PTR_W{1'b0}};
                wr_gray_r <= {C_PTR_W{1'b0}};
                wr_full_r <= 1'b0;
            end else begin
                wr_bin_r  <= wr_bin_next;
                wr_gray_r <= wr_gray_next;
                wr_full_r <= wr_full_next;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 读时钟域：同步写指针 Gray、推进读指针
    // ------------------------------------------------------------------------
    always @(posedge rd_clk or negedge rd_resetn) begin
        if (!rd_resetn) begin
            rd_bin_r           <= {C_PTR_W{1'b0}};
            rd_gray_r          <= {C_PTR_W{1'b0}};
            wr_gray_sync1_rd_r <= {C_PTR_W{1'b0}};
            wr_gray_sync2_rd_r <= {C_PTR_W{1'b0}};
            rd_empty_r         <= 1'b1;
        end else begin
            wr_gray_sync1_rd_r <= wr_gray_r;
            wr_gray_sync2_rd_r <= wr_gray_sync1_rd_r;

            if (rd_clear) begin
                rd_bin_r   <= {C_PTR_W{1'b0}};
                rd_gray_r  <= {C_PTR_W{1'b0}};
                rd_empty_r <= 1'b1;
            end else begin
                rd_bin_r   <= rd_bin_next;
                rd_gray_r  <= rd_gray_next;
                rd_empty_r <= rd_empty_next;
            end
        end
    end

endmodule
