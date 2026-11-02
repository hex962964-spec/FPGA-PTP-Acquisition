`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_rgmii_rx_if
// -----------------------------------------------------------------------------
// Purpose:
//   Convert PHY-side RGMII DDR RX signals into an internal 8-bit GMII-like bus.
//
// Required clocking, matching the user's board plan:
//   - clk_90 is clk_eth_rx.eth_rx_clk_90, generated from the incoming RGMII RXC.
//   - Xilinx IDDR primitives sample RXD/RX_CTL using the 90-degree shifted RXC.
//
// RGMII coding:
//   rising edge  RXD[3:0] = low nibble,  RX_CTL = RX_DV
//   falling edge RXD[3:0] = high nibble, RX_CTL = RX_DV ^ RX_ER
//
// IDDR SAME_EDGE note:
//   Q2 is the previous falling-edge sample retimed to the current rising edge.
//   Therefore the byte aligned to this cycle is {Q2, previous Q1}.
//////////////////////////////////////////////////////////////////////////////////
module observer_rgmii_rx_if #(
    parameter SIMULATION = 1'b0
)(
    input  wire       clk_90,
    input  wire       rst_n,

    input  wire [3:0] rgmii_rxd,
    input  wire       rgmii_rx_ctl,

    output reg  [7:0] gmii_rxd,
    output reg        gmii_rx_dv,
    output reg        gmii_rx_er
);

    generate
    if (SIMULATION) begin : g_sim
        // Simulation helper: assume incoming testbench already provides an
        // 8-bit-like stream on rxd[3:0] over two half-cycles of clk_90.
        reg [3:0] rise_d;
        reg       ctl_rise_d;
        always @(posedge clk_90 or negedge rst_n) begin
            if (!rst_n) begin
                rise_d      <= 4'd0;
                ctl_rise_d  <= 1'b0;
                gmii_rxd    <= 8'd0;
                gmii_rx_dv  <= 1'b0;
                gmii_rx_er  <= 1'b0;
            end else begin
                rise_d      <= rgmii_rxd;
                ctl_rise_d  <= rgmii_rx_ctl;
                gmii_rxd    <= {rgmii_rxd, rise_d};
                gmii_rx_dv  <= ctl_rise_d;
                gmii_rx_er  <= 1'b0;
            end
        end
    end else begin : g_xilinx
        wire [3:0] rxd_rise_w;
        wire [3:0] rxd_fall_w;
        wire       ctl_rise_w;
        wire       ctl_fall_w;

        reg [3:0] rxd_rise_d;
        reg       ctl_rise_d;

        genvar i;
        for (i = 0; i < 4; i = i + 1) begin : g_iddr_rxd
            IDDR #(
                .DDR_CLK_EDGE("SAME_EDGE"),
                .INIT_Q1(1'b0),
                .INIT_Q2(1'b0),
                .SRTYPE("ASYNC")
            ) u_iddr_rxd (
                .Q1(rxd_rise_w[i]),
                .Q2(rxd_fall_w[i]),
                .C (clk_90),
                .CE(1'b1),
                .D (rgmii_rxd[i]),
                .R (~rst_n),
                .S (1'b0)
            );
        end

        IDDR #(
            .DDR_CLK_EDGE("SAME_EDGE"),
            .INIT_Q1(1'b0),
            .INIT_Q2(1'b0),
            .SRTYPE("ASYNC")
        ) u_iddr_rx_ctl (
            .Q1(ctl_rise_w),
            .Q2(ctl_fall_w),
            .C (clk_90),
            .CE(1'b1),
            .D (rgmii_rx_ctl),
            .R (~rst_n),
            .S (1'b0)
        );

        always @(posedge clk_90 or negedge rst_n) begin
            if (!rst_n) begin
                rxd_rise_d <= 4'd0;
                ctl_rise_d <= 1'b0;
                gmii_rxd   <= 8'd0;
                gmii_rx_dv <= 1'b0;
                gmii_rx_er <= 1'b0;
            end else begin
                gmii_rxd   <= {rxd_fall_w, rxd_rise_d};
                gmii_rx_dv <= ctl_rise_d;
                gmii_rx_er <= ctl_rise_d ^ ctl_fall_w;

                rxd_rise_d <= rxd_rise_w;
                ctl_rise_d <= ctl_rise_w;
            end
        end
    end
    endgenerate

endmodule
`default_nettype wire
