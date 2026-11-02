`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_rgmii_tx_if
// -----------------------------------------------------------------------------
// Purpose:
//   Convert an internal 8-bit GMII-like TX bus into a 4-bit DDR RGMII TX port.
//
// Required clocking, matching the user's board plan:
//   - clk       : 125 MHz eth_tx_clk, used by TXD/TX_CTL ODDR primitives.
//   - clk_90    : 125 MHz eth_tx_clk_90, 90-degree phase-shifted, used by TXC.
//
// RGMII coding:
//   rising edge  TXD[3:0] = gmii_txd[3:0], TX_CTL = gmii_tx_en
//   falling edge TXD[3:0] = gmii_txd[7:4], TX_CTL = gmii_tx_en ^ gmii_tx_er
//
// Xilinx primitives:
//   ODDR is used for TXD, TX_CTL and TXC when SIMULATION=0.
//////////////////////////////////////////////////////////////////////////////////
module observer_rgmii_tx_if #(
    parameter SIMULATION = 1'b0
)(
    input  wire       clk,
    input  wire       clk_90,
    input  wire       rst_n,

    input  wire [7:0] gmii_txd,
    input  wire       gmii_tx_en,
    input  wire       gmii_tx_er,

    output wire       rgmii_txc,
    output wire [3:0] rgmii_txd,
    output wire       rgmii_tx_ctl
);

    generate
    if (SIMULATION) begin : g_sim
        reg [3:0] txd_r;
        reg       ctl_r;
        assign rgmii_txc = rst_n ? clk_90 : 1'b0;

        always @(posedge clk or negedge clk or negedge rst_n) begin
            if (!rst_n) begin
                txd_r <= 4'd0;
                ctl_r <= 1'b0;
            end else if (clk) begin
                txd_r <= gmii_txd[3:0];
                ctl_r <= gmii_tx_en;
            end else begin
                txd_r <= gmii_txd[7:4];
                ctl_r <= gmii_tx_en ^ gmii_tx_er;
            end
        end

        assign rgmii_txd    = txd_r;
        assign rgmii_tx_ctl = ctl_r;
    end else begin : g_xilinx
        ODDR #(
            .DDR_CLK_EDGE("SAME_EDGE"),
            .INIT(1'b0),
            .SRTYPE("SYNC")
        ) u_oddr_txc (
            .Q (rgmii_txc),
            .C (clk_90),
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
        ) u_oddr_tx_ctl (
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
`default_nettype wire
