`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: ptp_rx_status_cdc
// Description:
//   Bring PTP RX filter statistics from eth_rx_clk domain back to sys_clk domain
//   using a stable shadow bus plus req/ack toggle handshake.
//
//   Exported information:
//     - accepted record count
//     - dropped record count
//     - last filter error code
//     - RX async-FIFO overflow sticky
//
//   This module is intentionally small and dedicated so ptp_rx_path can keep a
//   clean structure: RX parse path + RX data CDC + RX debug-status CDC.
//////////////////////////////////////////////////////////////////////////////////
module ptp_rx_status_cdc(
    input  wire         eth_clk,
    input  wire         eth_resetn,
    input  wire [31:0]  eth_accept_count,
    input  wire [31:0]  eth_drop_count,
    input  wire [15:0]  eth_last_error,
    input  wire         eth_fifo_overflow_sticky,

    input  wire         sys_clk,
    input  wire         sys_resetn,

    output reg  [31:0]  sys_accept_count,
    output reg  [31:0]  sys_drop_count,
    output reg  [15:0]  sys_last_error,
    output reg          sys_fifo_overflow_sticky
);

    localparam integer P_STATUS_W = 81;

    // 打包顺序：
    //   [80]    fifo_overflow_sticky
    //   [79:64] last_error
    //   [63:32] drop_count
    //   [31:0]  accept_count
    wire [P_STATUS_W-1:0] eth_status_bus_w;
    assign eth_status_bus_w = {
        eth_fifo_overflow_sticky,
        eth_last_error,
        eth_drop_count,
        eth_accept_count
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

            if (!status_pending_eth_r && (eth_status_bus_w != status_last_sent_eth_r)) begin
                status_shadow_eth_r     <= eth_status_bus_w;
                status_last_sent_eth_r  <= eth_status_bus_w;
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
            sys_accept_count              <= 32'd0;
            sys_drop_count                <= 32'd0;
            sys_last_error                <= 16'd0;
            sys_fifo_overflow_sticky      <= 1'b0;
        end else begin
            status_req_toggle_sync1_sys_r <= status_req_toggle_eth_r;
            status_req_toggle_sync2_sys_r <= status_req_toggle_sync1_sys_r;

            if (status_req_toggle_sync2_sys_r != status_req_toggle_seen_sys_r) begin
                status_req_toggle_seen_sys_r <= status_req_toggle_sync2_sys_r;
                {
                    sys_fifo_overflow_sticky,
                    sys_last_error,
                    sys_drop_count,
                    sys_accept_count
                } <= status_shadow_eth_r;
                status_ack_toggle_sys_r <= ~status_ack_toggle_sys_r;
            end
        end
    end

endmodule
