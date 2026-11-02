`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// rgmii_tx_if
// -----------------------------------------------------------------------------
// 将 8-bit GMII TX 数据转换成 4-bit DDR RGMII TX 接口。
//
// RGMII 发送约定：
//   - 上升沿发送 data[3:0]
//   - 下降沿发送 data[7:4]
//   - tx_ctl 上升沿 = tx_en
//   - tx_ctl 下降沿 = tx_en ^ tx_er
//
// 关键时序策略：
//   - rgmii_txd / rgmii_tx_ctl 使用 clk 打到 ODDR
//   - rgmii_txc 使用 clk_shift 打到 ODDR
//
// SIMULATION=1 时使用行为模型，便于纯 RTL 仿真；
// SIMULATION=0 时使用 Xilinx ODDR，便于直接在 Vivado 上板。
//////////////////////////////////////////////////////////////////////////////////
module rgmii_tx_if #(
    parameter SIMULATION = 1'b0
)(
    input  wire         clk,
    input  wire         clk_shift,
    input  wire         rst_n,
    input  wire [7:0]   gmii_txd,
    input  wire         gmii_tx_en,
    input  wire         gmii_tx_er,
    output wire         rgmii_txc,
    output wire [3:0]   rgmii_txd,
    output wire         rgmii_tx_ctl
);

generate
if (SIMULATION) begin : g_sim
    reg [3:0]  rgmii_txd_r;
    reg        rgmii_tx_ctl_r;

    assign rgmii_txc = rst_n ? clk_shift : 1'b0;

    always @(posedge clk or negedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rgmii_txd_r    <= 4'd0;
            rgmii_tx_ctl_r <= 1'b0;
        end else if (clk) begin
            rgmii_txd_r    <= gmii_txd[3:0];
            rgmii_tx_ctl_r <= gmii_tx_en;
        end else begin
            rgmii_txd_r    <= gmii_txd[7:4];
            rgmii_tx_ctl_r <= gmii_tx_en ^ gmii_tx_er;
        end
    end

    assign rgmii_txd    = rgmii_txd_r;
    assign rgmii_tx_ctl = rgmii_tx_ctl_r;
end else begin : g_xilinx
    ODDR #(
        .DDR_CLK_EDGE("SAME_EDGE"),
        .INIT(1'b0),
        .SRTYPE("SYNC")
    ) u_oddr_txc (
        .Q (rgmii_txc),
        .C (clk_shift),
        .CE(1'b1),
        .D1(1'b1),
        .D2(1'b0),
        .R (~rst_n),
        .S (1'b0)
    );

    genvar i;
    for (i = 0; i < 4; i = i + 1) begin : g_txd
        ODDR #(
            .DDR_CLK_EDGE("SAME_EDGE"),
            .INIT(1'b0),
            .SRTYPE("SYNC")
        ) u_oddr_txd (
            .Q (rgmii_txd[i]),
            .C (clk),
            .CE(1'b1),
            .D1(gmii_txd[i]),
            .D2(gmii_txd[i+4]),
            .R (~rst_n),
            .S (1'b0)
        );
    end

    ODDR #(
        .DDR_CLK_EDGE("SAME_EDGE"),
        .INIT(1'b0),
        .SRTYPE("SYNC")
    ) u_oddr_txctl (
        .Q (rgmii_tx_ctl),
        .C (clk),
        .CE(1'b1),
        .D1(gmii_tx_en),
        .D2(gmii_tx_en ^ gmii_tx_er),
        .R (~rst_n),
        .S (1'b0)
    );
end
endgenerate

endmodule
