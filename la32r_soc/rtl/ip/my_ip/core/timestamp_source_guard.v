`timescale 1ns / 1ps
`default_nettype none
// Do not silently change a PTP-nanosecond stream back to legacy clock ticks.
// Once external time has been selected, any invalidity while acquisition is
// enabled latches a fault and blocks NEW sample accepts until cfg_enable=0.
// This forces the existing stop/flush/restart path rather than resuming a packet
// across a time epoch. The legacy-only mode is unchanged until first ext_valid.
module timestamp_source_guard (
    input wire clk, input wire resetn, input wire cfg_enable,
    input wire [63:0] legacy_time,
    input wire [63:0] ext_time_ns,
    input wire ext_time_valid,
    output wire [63:0] selected_time,
    output wire accept_time_valid,
    output reg time_fault,
    output reg external_selected
);
    assign selected_time = (external_selected || ext_time_valid) ? ext_time_ns : legacy_time;
    assign accept_time_valid = !time_fault &&
        ((!external_selected && !ext_time_valid) ||
         (external_selected && ext_time_valid));
    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            external_selected <= 1'b0;
            time_fault <= 1'b0;
        end else begin
            if (ext_time_valid) external_selected <= 1'b1;
            if (!cfg_enable) time_fault <= 1'b0;
            else if ((external_selected && !ext_time_valid) ||
                     (!external_selected && ext_time_valid))
                time_fault <= 1'b1;
        end
    end
endmodule
`default_nettype wire
