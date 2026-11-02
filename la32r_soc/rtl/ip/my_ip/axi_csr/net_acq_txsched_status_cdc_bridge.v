`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_txsched_status_cdc_bridge
// Description:
//   Bring TSN-lite v1 scheduler observability from eth_tx_clk back to sys_clk
//   using an atomic snapshot bus plus req/ack toggle handshake.
//////////////////////////////////////////////////////////////////////////////////
module net_acq_txsched_status_cdc_bridge(
    input  wire         eth_clk,
    input  wire         eth_resetn,
    input  wire [31:0]  eth_txs_status,
    input  wire [31:0]  eth_txs_held_count,
    input  wire [31:0]  eth_txs_pass_count,
    input  wire [31:0]  eth_txs_late_count,
    input  wire [31:0]  eth_txs_slot_miss_count,
    input  wire [63:0]  eth_txs_last_plan_ns,
    input  wire [63:0]  eth_txs_last_actual_ns,
    input  wire [31:0]  eth_txs_last_delta_ns,

    input  wire         sys_clk,
    input  wire         sys_resetn,

    output reg  [31:0]  sys_txs_status,
    output reg  [31:0]  sys_txs_held_count,
    output reg  [31:0]  sys_txs_pass_count,
    output reg  [31:0]  sys_txs_late_count,
    output reg  [31:0]  sys_txs_slot_miss_count,
    output reg  [63:0]  sys_txs_last_plan_ns,
    output reg  [63:0]  sys_txs_last_actual_ns,
    output reg  [31:0]  sys_txs_last_delta_ns,
    output reg          sys_txs_update_pulse
);

    localparam integer P_STATUS_W = 320;

    wire [P_STATUS_W-1:0] status_bus_eth_w;
    assign status_bus_eth_w = {
        eth_txs_status,
        eth_txs_held_count,
        eth_txs_pass_count,
        eth_txs_late_count,
        eth_txs_slot_miss_count,
        eth_txs_last_plan_ns,
        eth_txs_last_actual_ns,
        eth_txs_last_delta_ns
    };

    reg [P_STATUS_W-1:0] status_shadow_eth_r;
    reg [P_STATUS_W-1:0] status_last_sent_eth_r;
    reg                  status_req_toggle_eth_r;
    reg                  status_pending_eth_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg status_ack_toggle_sync1_eth_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg status_ack_toggle_sync2_eth_r;
    reg                                                      status_ack_toggle_seen_eth_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg status_req_toggle_sync1_sys_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg status_req_toggle_sync2_sys_r;
    reg                                                      status_req_toggle_seen_sys_r;
    reg                                                      status_ack_toggle_sys_r;

    always @(posedge eth_clk or negedge eth_resetn) begin
        if (!eth_resetn) begin
            status_shadow_eth_r           <= {P_STATUS_W{1'b0}};
            status_last_sent_eth_r        <= {P_STATUS_W{1'b0}};
            status_req_toggle_eth_r       <= 1'b0;
            status_pending_eth_r          <= 1'b0;
            status_ack_toggle_sync1_eth_r <= 1'b0;
            status_ack_toggle_sync2_eth_r <= 1'b0;
            status_ack_toggle_seen_eth_r  <= 1'b0;
        end else begin
            status_ack_toggle_sync1_eth_r <= status_ack_toggle_sys_r;
            status_ack_toggle_sync2_eth_r <= status_ack_toggle_sync1_eth_r;

            if (status_ack_toggle_sync2_eth_r != status_ack_toggle_seen_eth_r) begin
                status_ack_toggle_seen_eth_r <= status_ack_toggle_sync2_eth_r;
                status_pending_eth_r         <= 1'b0;
            end

            if (!status_pending_eth_r && (status_bus_eth_w != status_last_sent_eth_r)) begin
                status_shadow_eth_r     <= status_bus_eth_w;
                status_last_sent_eth_r  <= status_bus_eth_w;
                status_req_toggle_eth_r <= ~status_req_toggle_eth_r;
                status_pending_eth_r    <= 1'b1;
            end
        end
    end

    always @(posedge sys_clk or negedge sys_resetn) begin
        if (!sys_resetn) begin
            status_req_toggle_sync1_sys_r <= 1'b0;
            status_req_toggle_sync2_sys_r <= 1'b0;
            status_req_toggle_seen_sys_r  <= 1'b0;
            status_ack_toggle_sys_r       <= 1'b0;
            sys_txs_status                <= 32'd0;
            sys_txs_held_count            <= 32'd0;
            sys_txs_pass_count            <= 32'd0;
            sys_txs_late_count            <= 32'd0;
            sys_txs_slot_miss_count       <= 32'd0;
            sys_txs_last_plan_ns          <= 64'd0;
            sys_txs_last_actual_ns        <= 64'd0;
            sys_txs_last_delta_ns         <= 32'd0;
            sys_txs_update_pulse          <= 1'b0;
        end else begin
            status_req_toggle_sync1_sys_r <= status_req_toggle_eth_r;
            status_req_toggle_sync2_sys_r <= status_req_toggle_sync1_sys_r;
            sys_txs_update_pulse          <= 1'b0;

            if (status_req_toggle_sync2_sys_r != status_req_toggle_seen_sys_r) begin
                status_req_toggle_seen_sys_r <= status_req_toggle_sync2_sys_r;
                {
                    sys_txs_status,
                    sys_txs_held_count,
                    sys_txs_pass_count,
                    sys_txs_late_count,
                    sys_txs_slot_miss_count,
                    sys_txs_last_plan_ns,
                    sys_txs_last_actual_ns,
                    sys_txs_last_delta_ns
                } <= status_shadow_eth_r;
                status_ack_toggle_sys_r <= ~status_ack_toggle_sys_r;
                sys_txs_update_pulse    <= 1'b1;
            end
        end
    end

endmodule
