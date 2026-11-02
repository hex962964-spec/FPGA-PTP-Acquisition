`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_status_cdc_bridge
// -----------------------------------------------------------------------------
// Purpose:
//   Snapshot a multi-bit status bus from a source clock domain into sys_clk.
//
// CDC pattern:
//   sys_clk request pulse -> request toggle -> source domain captures status bus
//   into a hold bank -> source toggles acknowledge -> sys_clk samples hold bank.
//
// Usage notes:
//   - src_status_bus may change continuously in the source domain.
//   - src_hold_bus is only updated when a new request is received, and is then
//     held stable until the next request.  sys_clk samples it only after the ACK
//     toggle has been synchronized back, so the multi-bit crossing is protected
//     by the req/ack protocol rather than treated as a free-running bus.
//////////////////////////////////////////////////////////////////////////////////
module observer_status_cdc_bridge #(
    parameter integer WIDTH = 32
)(
    input  wire             sys_clk,
    input  wire             sys_rst_n,
    input  wire             sys_req_pulse,
    output reg  [WIDTH-1:0] sys_status_bus,
    output reg              sys_valid_pulse,

    input  wire             src_clk,
    input  wire             src_rst_n,
    input  wire [WIDTH-1:0] src_status_bus
);

    reg sys_req_toggle_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg src_req_sync0_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg src_req_sync1_r;
    reg src_req_seen_r;
    reg [WIDTH-1:0] src_hold_bus_r;
    reg src_ack_toggle_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg sys_ack_sync0_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg sys_ack_sync1_r;
    reg sys_ack_seen_r;

    // Request launcher in sys_clk.
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            sys_req_toggle_r <= 1'b0;
        end else if (sys_req_pulse) begin
            sys_req_toggle_r <= ~sys_req_toggle_r;
        end
    end

    // Request receiver and source-domain hold bank.
    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            src_req_sync0_r <= 1'b0;
            src_req_sync1_r <= 1'b0;
            src_req_seen_r  <= 1'b0;
            src_hold_bus_r  <= {WIDTH{1'b0}};
            src_ack_toggle_r <= 1'b0;
        end else begin
            src_req_sync0_r <= sys_req_toggle_r;
            src_req_sync1_r <= src_req_sync0_r;

            if (src_req_sync1_r != src_req_seen_r) begin
                src_req_seen_r   <= src_req_sync1_r;
                src_hold_bus_r   <= src_status_bus;
                src_ack_toggle_r <= ~src_ack_toggle_r;
            end
        end
    end

    // ACK receiver and stable-bus sample in sys_clk.
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            sys_ack_sync0_r <= 1'b0;
            sys_ack_sync1_r <= 1'b0;
            sys_ack_seen_r  <= 1'b0;
            sys_status_bus  <= {WIDTH{1'b0}};
            sys_valid_pulse <= 1'b0;
        end else begin
            sys_valid_pulse <= 1'b0;
            sys_ack_sync0_r <= src_ack_toggle_r;
            sys_ack_sync1_r <= sys_ack_sync0_r;

            if (sys_ack_sync1_r != sys_ack_seen_r) begin
                sys_ack_seen_r  <= sys_ack_sync1_r;
                sys_status_bus  <= src_hold_bus_r;
                sys_valid_pulse <= 1'b1;
            end
        end
    end

endmodule
`default_nettype wire
