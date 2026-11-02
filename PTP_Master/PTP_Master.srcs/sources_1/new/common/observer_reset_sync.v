`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_reset_sync
// -----------------------------------------------------------------------------
// Purpose:
//   Asynchronous assert, synchronous release reset synchronizer.
//   The external reset of this observer project is active low.  Every generated
//   clock domain must use one instance of this module before driving local RTL.
//
// Reset policy:
//   arst_n = 0  -> rst_n = 0 immediately
//   arst_n = 1  -> rst_n is released after two clk cycles
//////////////////////////////////////////////////////////////////////////////////
module observer_reset_sync(
    input  wire clk,
    input  wire arst_n,
    output wire rst_n
);

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg sync0_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg sync1_r;

    always @(posedge clk or negedge arst_n) begin
        if (!arst_n) begin
            sync0_r <= 1'b0;
            sync1_r <= 1'b0;
        end else begin
            sync0_r <= 1'b1;
            sync1_r <= sync0_r;
        end
    end

    assign rst_n = sync1_r;

endmodule
`default_nettype wire
