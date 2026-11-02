`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_dbgext_cdc_bridge
// Description:
//   Read-only debug/scheduler-extension snapshot bridge.
//
//   Purpose:
//     Bring eth_tx-domain observability counters back into sys_clk without
//     perturbing the existing ETH status plane or the software-visible legacy
//     register map.
//
//   Transport policy:
//     - All inputs are packed into a single snapshot bus.
//     - eth_tx side launches a new snapshot only when the packed value changes
//       and the previous snapshot has already been ACKed by sys_clk.
//     - sys_clk side latches the whole snapshot atomically and returns an ACK.
//
//   This is intentionally separate from net_acq_status_cdc_bridge so the new
//   0x140+ debug/scheduler extension can evolve independently from the stable
//   ETH_STATUS / ETH_* legacy register group.
//////////////////////////////////////////////////////////////////////////////////
module net_acq_dbgext_cdc_bridge(
    // ------------------------------------------------------------------------
    // eth_tx domain inputs
    // ------------------------------------------------------------------------
    input  wire         eth_clk,
    input  wire         eth_resetn,
    input  wire [31:0]  eth_arb_data_frame_count,
    input  wire [31:0]  eth_arb_ptp_frame_count,
    input  wire [31:0]  eth_arb_stall_cycles,
    input  wire [15:0]  eth_arb_last_error,
    input  wire [31:0]  eth_backend_backpressure_cycles,
    input  wire [31:0]  eth_ptp_tx_trigger_req_count,
    input  wire [31:0]  eth_ptp_tx_trigger_accept_count,
    input  wire [31:0]  eth_ptp_tx_trigger_while_busy_count,
    input  wire [15:0]  eth_ptp_tx_last_error,

    // ------------------------------------------------------------------------
    // sys domain inputs
    // ------------------------------------------------------------------------
    input  wire         sys_clk,
    input  wire         sys_resetn,

    // ------------------------------------------------------------------------
    // sys domain outputs
    // ------------------------------------------------------------------------
    output reg  [31:0]  sys_arb_data_frame_count,
    output reg  [31:0]  sys_arb_ptp_frame_count,
    output reg  [31:0]  sys_arb_stall_cycles,
    output reg  [15:0]  sys_arb_last_error,
    output reg  [31:0]  sys_backend_backpressure_cycles,
    output reg  [31:0]  sys_ptp_tx_trigger_req_count,
    output reg  [31:0]  sys_ptp_tx_trigger_accept_count,
    output reg  [31:0]  sys_ptp_tx_trigger_while_busy_count,
    output reg  [15:0]  sys_ptp_tx_last_error,
    output reg          sys_dbgext_update_pulse
);

    localparam integer P_DBGEXT_W = 256;

    // 打包顺序：
    //   [255:224] ptp_tx_trigger_req_count
    //   [223:192] ptp_tx_trigger_accept_count
    //   [191:160] ptp_tx_trigger_while_busy_count
    //   [159:128] backend_backpressure_cycles
    //   [127:96 ] arb_data_frame_count
    //   [95 :64 ] arb_ptp_frame_count
    //   [63 :32 ] arb_stall_cycles
    //   [31 :16 ] arb_last_error
    //   [15 :0  ] ptp_tx_last_error
    wire [P_DBGEXT_W-1:0] dbgext_bus_eth_w;
    assign dbgext_bus_eth_w = {
        eth_ptp_tx_trigger_req_count,
        eth_ptp_tx_trigger_accept_count,
        eth_ptp_tx_trigger_while_busy_count,
        eth_backend_backpressure_cycles,
        eth_arb_data_frame_count,
        eth_arb_ptp_frame_count,
        eth_arb_stall_cycles,
        eth_arb_last_error,
        eth_ptp_tx_last_error
    };

    // ------------------------------------------------------------------------
    // eth -> sys snapshot toggle bridge
    // ------------------------------------------------------------------------
    reg  [P_DBGEXT_W-1:0] dbgext_shadow_eth_r;
    reg  [P_DBGEXT_W-1:0] dbgext_last_sent_eth_r;
    reg                   dbgext_req_toggle_eth_r;
    reg                   dbgext_pending_eth_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg dbgext_ack_toggle_sync1_eth_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg dbgext_ack_toggle_sync2_eth_r;
    reg                                                    dbgext_ack_toggle_seen_eth_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg dbgext_req_toggle_sync1_sys_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg dbgext_req_toggle_sync2_sys_r;
    reg                                                    dbgext_req_toggle_seen_sys_r;
    reg                                                    dbgext_ack_toggle_sys_r;

    always @(posedge eth_clk or negedge eth_resetn) begin
        if (!eth_resetn) begin
            dbgext_shadow_eth_r           <= {P_DBGEXT_W{1'b0}};
            dbgext_last_sent_eth_r        <= {P_DBGEXT_W{1'b0}};
            dbgext_req_toggle_eth_r       <= 1'b0;
            dbgext_pending_eth_r          <= 1'b0;
            dbgext_ack_toggle_sync1_eth_r <= 1'b0;
            dbgext_ack_toggle_sync2_eth_r <= 1'b0;
            dbgext_ack_toggle_seen_eth_r  <= 1'b0;
        end else begin
            dbgext_ack_toggle_sync1_eth_r <= dbgext_ack_toggle_sys_r;
            dbgext_ack_toggle_sync2_eth_r <= dbgext_ack_toggle_sync1_eth_r;

            if (dbgext_ack_toggle_sync2_eth_r != dbgext_ack_toggle_seen_eth_r) begin
                dbgext_ack_toggle_seen_eth_r <= dbgext_ack_toggle_sync2_eth_r;
                dbgext_pending_eth_r         <= 1'b0;
            end

            if (!dbgext_pending_eth_r && (dbgext_bus_eth_w != dbgext_last_sent_eth_r)) begin
                dbgext_shadow_eth_r     <= dbgext_bus_eth_w;
                dbgext_last_sent_eth_r  <= dbgext_bus_eth_w;
                dbgext_req_toggle_eth_r <= ~dbgext_req_toggle_eth_r;
                dbgext_pending_eth_r    <= 1'b1;
            end
        end
    end

    always @(posedge sys_clk or negedge sys_resetn) begin
        if (!sys_resetn) begin
            dbgext_req_toggle_sync1_sys_r     <= 1'b0;
            dbgext_req_toggle_sync2_sys_r     <= 1'b0;
            dbgext_req_toggle_seen_sys_r      <= 1'b0;
            dbgext_ack_toggle_sys_r           <= 1'b0;
            sys_arb_data_frame_count          <= 32'd0;
            sys_arb_ptp_frame_count           <= 32'd0;
            sys_arb_stall_cycles              <= 32'd0;
            sys_arb_last_error                <= 16'd0;
            sys_backend_backpressure_cycles   <= 32'd0;
            sys_ptp_tx_trigger_req_count      <= 32'd0;
            sys_ptp_tx_trigger_accept_count   <= 32'd0;
            sys_ptp_tx_trigger_while_busy_count <= 32'd0;
            sys_ptp_tx_last_error             <= 16'd0;
            sys_dbgext_update_pulse           <= 1'b0;
        end else begin
            dbgext_req_toggle_sync1_sys_r <= dbgext_req_toggle_eth_r;
            dbgext_req_toggle_sync2_sys_r <= dbgext_req_toggle_sync1_sys_r;
            sys_dbgext_update_pulse       <= 1'b0;

            if (dbgext_req_toggle_sync2_sys_r != dbgext_req_toggle_seen_sys_r) begin
                dbgext_req_toggle_seen_sys_r <= dbgext_req_toggle_sync2_sys_r;
                {
                    sys_ptp_tx_trigger_req_count,
                    sys_ptp_tx_trigger_accept_count,
                    sys_ptp_tx_trigger_while_busy_count,
                    sys_backend_backpressure_cycles,
                    sys_arb_data_frame_count,
                    sys_arb_ptp_frame_count,
                    sys_arb_stall_cycles,
                    sys_arb_last_error,
                    sys_ptp_tx_last_error
                } <= dbgext_shadow_eth_r;

                dbgext_ack_toggle_sys_r <= ~dbgext_ack_toggle_sys_r;
                sys_dbgext_update_pulse <= 1'b1;
            end
        end
    end

endmodule
