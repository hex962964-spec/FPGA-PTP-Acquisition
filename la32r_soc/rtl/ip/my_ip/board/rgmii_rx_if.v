`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: rgmii_rx_if
// -----------------------------------------------------------------------------
// Purpose:
//   Convert PHY-side RGMII DDR RX signals into an internal 8-bit GMII-like bus.
//
// Key points:
//   1) Xilinx IDDR primitives are used in SAME_EDGE mode as requested.
//   2) RGMII carries low nibble on the rising edge and high nibble on the
//      falling edge.
//   3) RX_CTL carries RX_DV on the rising edge and RX_DV XOR RX_ER on the
//      falling edge.
//   4) In IDDR SAME_EDGE mode, Q2 is the previous falling-edge sample retimed
//      onto the current rising edge. Therefore the byte must be assembled as:
//           { current Q2(high nibble), previous-cycle Q1(low nibble) }
//      instead of directly concatenating current-cycle Q1/Q2.
//
// Notes:
//   - This module targets Xilinx flows (Vivado) where IDDR is a built-in
//     primitive.
//   - If you later need stronger board-level timing closure, you can add IODELAY
//     or clock constraints around this block without changing its interface.
//////////////////////////////////////////////////////////////////////////////////
module rgmii_rx_if(
    input  wire        rgmii_rxc,
    input  wire        rst_n,
    input  wire [3:0]  rgmii_rxd,
    input  wire        rgmii_rx_ctl,

    output reg  [7:0]  gmii_rxd,
    output reg         gmii_rx_dv,
    output reg         gmii_rx_er
);

    wire [3:0] rxd_rise_w;
    wire [3:0] rxd_fall_w;
    wire       ctl_rise_w;
    wire       ctl_fall_w;

    reg  [3:0] rxd_rise_d;
    reg        ctl_rise_d;

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_iddr_rxd
            IDDR #(
                .DDR_CLK_EDGE("SAME_EDGE"),
                .INIT_Q1(1'b0),
                .INIT_Q2(1'b0),
                .SRTYPE("ASYNC")
            ) u_iddr_rxd (
                .Q1(rxd_rise_w[i]),
                .Q2(rxd_fall_w[i]),
                .C(rgmii_rxc),
                .CE(1'b1),
                .D(rgmii_rxd[i]),
                .R(~rst_n),
                .S(1'b0)
            );
        end
    endgenerate

    IDDR #(
        .DDR_CLK_EDGE("SAME_EDGE"),
        .INIT_Q1(1'b0),
        .INIT_Q2(1'b0),
        .SRTYPE("ASYNC")
    ) u_iddr_rx_ctl (
        .Q1(ctl_rise_w),
        .Q2(ctl_fall_w),
        .C(rgmii_rxc),
        .CE(1'b1),
        .D(rgmii_rx_ctl),
        .R(~rst_n),
        .S(1'b0)
    );

    always @(posedge rgmii_rxc or negedge rst_n) begin
        if (!rst_n) begin
            rxd_rise_d <= 4'd0;
            ctl_rise_d <= 1'b0;
            gmii_rxd   <= 8'd0;
            gmii_rx_dv <= 1'b0;
            gmii_rx_er <= 1'b0;
        end else begin
            // IDDR SAME_EDGE:
            //   rxd_rise_w / ctl_rise_w are samples from the current rising edge
            //   rxd_fall_w / ctl_fall_w are samples from the previous falling edge
            // So the valid byte on this cycle is:
            //   { previous falling-edge high nibble, previous rising-edge low nibble }
            gmii_rxd   <= {rxd_fall_w, rxd_rise_d};
            gmii_rx_dv <= ctl_rise_d;
            gmii_rx_er <= ctl_rise_d ^ ctl_fall_w;

            rxd_rise_d <= rxd_rise_w;
            ctl_rise_d <= ctl_rise_w;
        end
    end

endmodule
