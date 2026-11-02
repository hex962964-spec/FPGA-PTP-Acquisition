`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_ptp_delay_req_cdc
// -----------------------------------------------------------------------------
// Single-record req/ack CDC bridge for Delay_Req records from eth_rx_clk_90 to
// eth_tx_clk.  The source pulse is captured into a stable hold bank; the
// destination presents a level-valid record until the Delay_Resp generator
// accepts it.
//////////////////////////////////////////////////////////////////////////////////
module observer_ptp_delay_req_cdc #(
    parameter integer WIDTH = 240
)(
    input  wire             src_clk,
    input  wire             src_rst_n,
    input  wire             src_valid_pulse,
    output wire             src_ready,
    input  wire [WIDTH-1:0] src_record_bus,
    output reg  [31:0]      src_drop_count,

    input  wire             dst_clk,
    input  wire             dst_rst_n,
    output reg              dst_valid_pulse,
    input  wire             dst_ready,
    output reg  [WIDTH-1:0] dst_record_bus
);

    reg              src_pending_r;
    reg              src_req_toggle_r;
    reg [WIDTH-1:0]  src_hold_bus_r;
    reg              src_ack_seen_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0] src_ack_sync_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0] dst_req_sync_r;

    reg              dst_req_seen_r;
    reg              dst_ack_toggle_r;

    assign src_ready = !src_pending_r;

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            src_pending_r    <= 1'b0;
            src_req_toggle_r <= 1'b0;
            src_hold_bus_r   <= {WIDTH{1'b0}};
            src_ack_seen_r   <= 1'b0;
            src_ack_sync_r   <= 2'b00;
            src_drop_count   <= 32'd0;
        end else begin
            src_ack_sync_r <= {src_ack_sync_r[0], dst_ack_toggle_r};

            if (src_pending_r && (src_ack_sync_r[1] != src_ack_seen_r)) begin
                src_pending_r  <= 1'b0;
                src_ack_seen_r <= src_ack_sync_r[1];
            end

            if (src_valid_pulse) begin
                if (!src_pending_r) begin
                    src_hold_bus_r   <= src_record_bus;
                    src_pending_r    <= 1'b1;
                    src_req_toggle_r <= ~src_req_toggle_r;
                end else begin
                    src_drop_count <= src_drop_count + 32'd1;
                end
            end
        end
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            dst_req_sync_r   <= 2'b00;
            dst_req_seen_r   <= 1'b0;
            dst_ack_toggle_r <= 1'b0;
            dst_valid_pulse  <= 1'b0;
            dst_record_bus   <= {WIDTH{1'b0}};
        end else begin
            dst_req_sync_r <= {dst_req_sync_r[0], src_req_toggle_r};

            if (!dst_valid_pulse && (dst_req_sync_r[1] != dst_req_seen_r)) begin
                dst_record_bus  <= src_hold_bus_r;
                dst_valid_pulse <= 1'b1;
                dst_req_seen_r  <= dst_req_sync_r[1];
            end

            if (dst_valid_pulse && dst_ready) begin
                dst_valid_pulse  <= 1'b0;
                dst_ack_toggle_r <= dst_req_seen_r;
            end
        end
    end

endmodule
`default_nettype wire
