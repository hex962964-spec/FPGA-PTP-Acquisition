`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_tod_counter
// -----------------------------------------------------------------------------
// Purpose:
//   Hardware time-of-day counter for the Observer FPGA.
//
// Current master-TX version:
//   - Runs from a 125 MHz Ethernet clock by default.
//   - Maintains both a flat nanosecond counter and the native PTP timestamp
//     representation: secondsField[47:0] + nanosecondsField[31:0].
//   - The extra seconds/nanoseconds outputs avoid synthesizing a 64-bit divide
//     when the PTP Follow_Up frame needs the Sync SFD timestamp.
//
// Reset policy:
//   All local RTL uses active-low reset. clear_pulse synchronously resets the
//   local ToD to zero without resetting the surrounding pipeline.
//////////////////////////////////////////////////////////////////////////////////
module observer_tod_counter #(
    parameter [31:0] STEP_NS       = 32'd8,
    parameter [31:0] ONE_SECOND_NS = 32'd1000000000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        clear_pulse,

    output reg  [63:0] tod_ns,
    output reg  [47:0] tod_seconds,
    output reg  [31:0] tod_nanoseconds,
    output reg         tod_valid,
    output reg         pps_pulse
);

    wire [32:0] next_nsec_sum_w;
    wire        next_nsec_wrap_w;
    wire [32:0] next_nsec_sub_w;
    wire [31:0] next_nsec_w;

    assign next_nsec_sum_w  = {1'b0, tod_nanoseconds} + {1'b0, STEP_NS};
    assign next_nsec_wrap_w = (next_nsec_sum_w >= {1'b0, ONE_SECOND_NS});
    assign next_nsec_sub_w  = next_nsec_sum_w - {1'b0, ONE_SECOND_NS};
    assign next_nsec_w      = next_nsec_wrap_w ? next_nsec_sub_w[31:0] : next_nsec_sum_w[31:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tod_ns          <= 64'd0;
            tod_seconds     <= 48'd0;
            tod_nanoseconds <= 32'd0;
            tod_valid       <= 1'b0;
            pps_pulse       <= 1'b0;
        end else begin
            pps_pulse <= 1'b0;

            if (clear_pulse) begin
                tod_ns          <= 64'd0;
                tod_seconds     <= 48'd0;
                tod_nanoseconds <= 32'd0;
                tod_valid       <= enable;
            end else if (enable) begin
                tod_valid       <= 1'b1;
                tod_ns          <= tod_ns + {32'd0, STEP_NS};
                tod_nanoseconds <= next_nsec_w;

                if (next_nsec_wrap_w) begin
                    tod_seconds <= tod_seconds + 48'd1;
                    pps_pulse   <= 1'b1;
                end
            end else begin
                tod_valid <= 1'b0;
            end
        end
    end

endmodule
`default_nettype wire
