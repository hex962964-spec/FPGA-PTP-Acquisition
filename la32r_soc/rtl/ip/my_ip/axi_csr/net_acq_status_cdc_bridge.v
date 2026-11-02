`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_status_cdc_bridge
// Description:
//   把 eth_tx_clk 域中的后端状态/统计以及 PHY 管理状态安全回传到 sys_clk 域。
//
//   分两类处理：
//     1) 单 bit 动态状态（busy / tx_busy）
//        - 直接用 2FF 同步到 sys 域；
//        - 为同步链显式添加 ASYNC_REG / SHREG_EXTRACT 属性。
//
//     2) 多 bit 统计 / 长度 / 错误码 / PHY 快照
//        - 采用“快照 + req/ack toggle”方式；
//        - eth 侧只在当前值与上次已发送快照不同、且上一次快照已经收到 ACK 时，
//          才会发出下一次快照请求；
//        - sys 侧检测到新快照后，一次性采样整组 shadow bus，再回 ACK。
//
//   这样可以避免：
//     - 多位总线逐位同步造成撕裂；
//     - 多次快速变化导致最后一次状态丢失。
//////////////////////////////////////////////////////////////////////////////////
module net_acq_status_cdc_bridge(
    // ------------------------------------------------------------------------
    // eth domain inputs
    // ------------------------------------------------------------------------
    input  wire         eth_clk,
    input  wire         eth_resetn,
    input  wire         eth_stat_busy,
    input  wire         eth_stat_tx_busy,
    input  wire [31:0]  eth_stat_tx_packet_count,
    input  wire [31:0]  eth_stat_tx_byte_count,
    input  wire [31:0]  eth_stat_drop_count,
    input  wire [15:0]  eth_stat_last_ip_total_length,
    input  wire [15:0]  eth_stat_last_udp_length,
    input  wire [15:0]  eth_stat_last_error_code,
    input  wire [31:0]  eth_phy_status_word,
    input  wire [127:0] eth_phy_reg_snapshot,

    // ------------------------------------------------------------------------
    // sys domain inputs
    // ------------------------------------------------------------------------
    input  wire         sys_clk,
    input  wire         sys_resetn,

    // ------------------------------------------------------------------------
    // sys domain outputs
    // ------------------------------------------------------------------------
    output wire         sys_stat_busy,
    output wire         sys_stat_tx_busy,
    output reg  [31:0]  sys_stat_tx_packet_count,
    output reg  [31:0]  sys_stat_tx_byte_count,
    output reg  [31:0]  sys_stat_drop_count,
    output reg  [15:0]  sys_stat_last_ip_total_length,
    output reg  [15:0]  sys_stat_last_udp_length,
    output reg  [15:0]  sys_stat_last_error_code,
    output reg  [31:0]  sys_phy_status_word,
    output reg  [127:0] sys_phy_reg_snapshot,
    output reg          sys_status_update_pulse
);

    localparam integer P_STATUS_W = 304;

    // 打包顺序：
    //   [303:272] phy_status_word
    //   [271:144] phy_reg_snapshot
    //   [143:112] tx_packet_count
    //   [111:80 ] tx_byte_count
    //   [79 :48 ] drop_count
    //   [47 :32 ] last_ip_total_length
    //   [31 :16 ] last_udp_length
    //   [15 :0  ] last_error_code
    wire [P_STATUS_W-1:0] eth_status_bus_w;
    assign eth_status_bus_w = {
        eth_phy_status_word,
        eth_phy_reg_snapshot,
        eth_stat_tx_packet_count,
        eth_stat_tx_byte_count,
        eth_stat_drop_count,
        eth_stat_last_ip_total_length,
        eth_stat_last_udp_length,
        eth_stat_last_error_code
    };

    // ------------------------------------------------------------------------
    // 单 bit 状态：2FF 同步
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg busy_sync1_sys_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg busy_sync2_sys_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg tx_busy_sync1_sys_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg tx_busy_sync2_sys_r;

    assign sys_stat_busy    = busy_sync2_sys_r;
    assign sys_stat_tx_busy = tx_busy_sync2_sys_r;

    always @(posedge sys_clk or negedge sys_resetn) begin
        if (!sys_resetn) begin
            busy_sync1_sys_r    <= 1'b0;
            busy_sync2_sys_r    <= 1'b0;
            tx_busy_sync1_sys_r <= 1'b0;
            tx_busy_sync2_sys_r <= 1'b0;
        end else begin
            busy_sync1_sys_r    <= eth_stat_busy;
            busy_sync2_sys_r    <= busy_sync1_sys_r;
            tx_busy_sync1_sys_r <= eth_stat_tx_busy;
            tx_busy_sync2_sys_r <= tx_busy_sync1_sys_r;
        end
    end

    // ------------------------------------------------------------------------
    // 多 bit 快照：eth -> sys req/ack toggle
    // ------------------------------------------------------------------------
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

    // eth domain：当总线值变化且没有未完成快照时，发起一次新快照。
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

    // sys domain：接收快照并回 ACK。
    always @(posedge sys_clk or negedge sys_resetn) begin
        if (!sys_resetn) begin
            status_req_toggle_sync1_sys_r  <= 1'b0;
            status_req_toggle_sync2_sys_r  <= 1'b0;
            status_req_toggle_seen_sys_r   <= 1'b0;
            status_ack_toggle_sys_r        <= 1'b0;
            sys_stat_tx_packet_count       <= 32'd0;
            sys_stat_tx_byte_count         <= 32'd0;
            sys_stat_drop_count            <= 32'd0;
            sys_stat_last_ip_total_length  <= 16'd0;
            sys_stat_last_udp_length       <= 16'd0;
            sys_stat_last_error_code       <= 16'd0;
            sys_phy_status_word            <= 32'd0;
            sys_phy_reg_snapshot           <= 128'd0;
            sys_status_update_pulse        <= 1'b0;
        end else begin
            status_req_toggle_sync1_sys_r <= status_req_toggle_eth_r;
            status_req_toggle_sync2_sys_r <= status_req_toggle_sync1_sys_r;
            sys_status_update_pulse       <= 1'b0;

            if (status_req_toggle_sync2_sys_r != status_req_toggle_seen_sys_r) begin
                status_req_toggle_seen_sys_r <= status_req_toggle_sync2_sys_r;
                {
                    sys_phy_status_word,
                    sys_phy_reg_snapshot,
                    sys_stat_tx_packet_count,
                    sys_stat_tx_byte_count,
                    sys_stat_drop_count,
                    sys_stat_last_ip_total_length,
                    sys_stat_last_udp_length,
                    sys_stat_last_error_code
                } <= status_shadow_eth_r;

                status_ack_toggle_sys_r <= ~status_ack_toggle_sys_r;
                sys_status_update_pulse <= 1'b1;
            end
        end
    end

endmodule
