`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// top_phy
// -----------------------------------------------------------------------------
// 说明：
//   1) 保持现有 TX path 不动：
//        net_acq m_axis_eth_tx_* -> top_phy_axis_mac_tx -> rgmii_tx_if -> PHY
//   2) 保持现有最小 RX path：
//        RGMII RX -> rgmii_rx_if -> top_phy_axis_mac_rx -> wrapper/PTP RX filter
//   3) 新增标准化 PHY 管理子系统：
//        - Clause-22 MDIO master
//        - PHY 上电/复位后的自动初始化
//        - 周期性真实 link status 读取
//        - speed/duplex 状态读回
//        - 软件触发 soft reset / restart autoneg
//
// 设计边界：
//   - 不在这里做完整 MAC / ARP / 通用协议栈；
//   - RX 只提供 board-independent byte stream 和 event frame RX timestamp；
//   - mac_link_up 输出“真实物理链路 up”；
//   - 但当前 TX 主线仍按 125MHz / 1GbE 组织，因此内部实际放行条件为
//     link_up && speed==1000M && full_duplex。
//////////////////////////////////////////////////////////////////////////////////
module top_phy #(
    parameter SIMULATION                = 1'b0,
    parameter integer RESET_HOLD_CYCLES = 16'd1024,
    parameter integer MIN_FRAME_BYTES   = 60,
    parameter integer IFG_BYTES         = 12
)(
    input  wire         clk125,
    input  wire         clk125_shift,
    input  wire         rst_n,
    input  wire         rx_rst_n,

    // ------------------------------------------------------------------------
    // 来自 net_acq 后端的 board-independent MAC-side TX byte stream
    //   已经是完整 Ethernet II frame（不含 preamble/SFD/FCS）
    // ------------------------------------------------------------------------
    input  wire [7:0]   s_axis_eth_tdata,
    input  wire         s_axis_eth_tvalid,
    input  wire         s_axis_eth_tlast,
    output wire         s_axis_eth_tready,
    input  wire [3:0]   s_axis_eth_tuser,
    input  wire [63:0]  ptp_tod_time_ns,
    input  wire         ptp_tod_time_valid,

    // ------------------------------------------------------------------------
    // 来自 wrapper 的 RX 域 ToD
    //   用于在 RX SFD 处锁存 ptp_rxstamp_ns
    // ------------------------------------------------------------------------
    input  wire [63:0]  ptp_tod_time_ns_rx,
    input  wire         ptp_tod_time_valid_rx,

    // ------------------------------------------------------------------------
    // 来自 CSR/wrapper 的 PHY 管理命令（已在 eth_tx_clk 域）
    // ------------------------------------------------------------------------
    input  wire         phy_soft_reset_pulse,
    input  wire         phy_restart_an_pulse,

    // ------------------------------------------------------------------------
    // 回给 wrapper / CSR 的 PTP 时间戳 / 状态
    // ------------------------------------------------------------------------
    output wire [63:0]  ptp_txstamp_ns,
    output wire         ptp_txstamp_valid,
    output wire [63:0]  ptp_rxstamp_ns,
    output wire         ptp_rxstamp_valid,
    output wire         mac_link_up,
    output wire [31:0]  phy_status_word,
    output wire [127:0] phy_reg_snapshot,

    // ------------------------------------------------------------------------
    // 回给 wrapper 的 board-independent MAC-side RX byte stream
    //   已经去掉 preamble/SFD/FCS
    // ------------------------------------------------------------------------
    output wire [7:0]   m_axis_eth_rx_tdata,
    output wire         m_axis_eth_rx_tvalid,
    output wire         m_axis_eth_rx_tlast,

    // ------------------------------------------------------------------------
    // RGMII to/from external PHY
    // ------------------------------------------------------------------------
    output wire         rgmii_txc,
    output wire [3:0]   rgmii_txd,
    output wire         rgmii_tx_ctl,
    input  wire         rgmii_rxc,
    input  wire [3:0]   rgmii_rxd,
    input  wire         rgmii_rx_ctl,

    // PHY management interface
    output wire         phy_mdc,
    inout  wire         phy_mdio,
    output wire         phy_rst_n
);

    localparam integer P_PHY_POST_RESET_WAIT_CYCLES = SIMULATION ? 32    : 125000;
    localparam integer P_PHY_POLL_INTERVAL_CYCLES   = SIMULATION ? 2048  : 12500000;
    localparam integer P_PHY_MDC_HALF_PERIOD        = SIMULATION ? 2     : 24;

    // Local reset release trees: asynchronous assertion, two local clock edges
    // to release. Keep each bank distinct so implementation can place it near
    // its consumers; FIFO port resets and soft-clear protocols are unchanged.
    wire management_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_management_resetn (
        .clk(clk125), .rst_n_in(rst_n), .rst_n_out(management_resetn)
    );
    wire mac_tx_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_mac_tx_resetn (
        .clk(clk125), .rst_n_in(rst_n), .rst_n_out(mac_tx_resetn)
    );
    wire mac_rx_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_mac_rx_resetn (
        .clk(rgmii_rxc), .rst_n_in(rx_rst_n), .rst_n_out(mac_rx_resetn)
    );
    wire ddr_rx_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_ddr_rx_resetn (
        .clk(rgmii_rxc), .rst_n_in(rx_rst_n), .rst_n_out(ddr_rx_resetn)
    );

    // ------------------------------------------------------------------------
    // TX internal GMII-like signals
    // ------------------------------------------------------------------------
    wire [7:0] gmii_txd_w;
    wire       gmii_tx_en_w;
    wire       gmii_tx_er_w;

    wire [31:0] stat_frame_count_w;
    wire [31:0] stat_mac_byte_count_w;
    wire [15:0] stat_last_error_w;
    wire        tx_busy_w;
    wire [63:0] ptp_txstamp_ns_w;
    wire        ptp_txstamp_valid_w;

    // ------------------------------------------------------------------------
    // RX internal GMII-like signals
    // ------------------------------------------------------------------------
    wire [7:0] gmii_rxd_w;
    wire       gmii_rx_dv_w;
    wire       gmii_rx_er_w;

    wire [7:0]  m_axis_eth_rx_tdata_w;
    wire        m_axis_eth_rx_tvalid_w;
    wire        m_axis_eth_rx_tlast_w;
    wire [63:0] ptp_rxstamp_ns_w;
    wire        ptp_rxstamp_valid_w;

    wire [31:0] rx_stat_frame_count_w;
    wire [31:0] rx_stat_drop_count_w;
    wire [15:0] rx_stat_last_error_w;

    // clk125 -> rgmii_rxc 域的单比特同步链。
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0]  phy_rst_sync_rx_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0]  phy_mac_usable_sync_rx_r;
    wire        rx_enable_w;

    reg  [63:0] ptp_txstamp_ns_r;
    reg         ptp_txstamp_valid_r;
    reg  [63:0] ptp_rxstamp_ns_r;
    reg         ptp_rxstamp_valid_r;
    reg  [7:0]  m_axis_eth_rx_tdata_r;
    reg         m_axis_eth_rx_tvalid_r;
    reg         m_axis_eth_rx_tlast_r;

    // ------------------------------------------------------------------------
    // PHY 管理子系统
    // ------------------------------------------------------------------------
    wire        phy_mdio_o_w;
    wire        phy_mdio_oe_w;
    wire        phy_link_up_w;
    wire [31:0] phy_status_word_w;
    wire [127:0] phy_reg_snapshot_w;
    wire        phy_rst_n_w;

    wire [1:0] phy_speed_w        = phy_status_word_w[8:7];
    wire       phy_full_duplex_w  = phy_status_word_w[6];
    wire       phy_mac_usable_w   = phy_link_up_w && (phy_speed_w == 2'b10) && phy_full_duplex_w;

    // Register the multi-register PHY status decode before the RX CDC entry.
    // The existing RX two-flop synchronizer only receives this one source bit.
    // TX same-domain enable continues to use phy_mac_usable_w unchanged.
    reg phy_mac_usable_tx_r;
    always @(posedge clk125 or negedge management_resetn) begin
        if (!management_resetn)
            phy_mac_usable_tx_r <= 1'b0;
        else
            phy_mac_usable_tx_r <= phy_mac_usable_w;
    end

    phy_mdio_ctrl #(
        .RESET_HOLD_CYCLES      (RESET_HOLD_CYCLES),
        .POST_RESET_WAIT_CYCLES (P_PHY_POST_RESET_WAIT_CYCLES),
        .POLL_INTERVAL_CYCLES   (P_PHY_POLL_INTERVAL_CYCLES),
        .MDIO_MDC_HALF_PERIOD   (P_PHY_MDC_HALF_PERIOD)
    ) u_phy_mdio_ctrl (
        .clk                  (clk125),
        .resetn               (management_resetn),
        .cmd_soft_reset_pulse (phy_soft_reset_pulse),
        .cmd_restart_an_pulse (phy_restart_an_pulse),
        .phy_mdc              (phy_mdc),
        .phy_mdio_o           (phy_mdio_o_w),
        .phy_mdio_oe          (phy_mdio_oe_w),
        .phy_mdio_i           (phy_mdio),
        .phy_rst_n            (phy_rst_n_w),
        .link_up              (phy_link_up_w),
        .phy_status_word      (phy_status_word_w),
        .phy_reg_snapshot     (phy_reg_snapshot_w)
    );

    assign phy_mdio         = phy_mdio_oe_w ? phy_mdio_o_w : 1'bz;
    assign phy_rst_n        = phy_rst_n_w;
    assign mac_link_up      = phy_link_up_w;
    assign phy_status_word  = phy_status_word_w;
    assign phy_reg_snapshot = phy_reg_snapshot_w;

    always @(posedge rgmii_rxc or negedge mac_rx_resetn) begin
        if (!mac_rx_resetn) begin
            phy_rst_sync_rx_r        <= 2'b00;
            phy_mac_usable_sync_rx_r <= 2'b00;
        end else begin
            phy_rst_sync_rx_r        <= {phy_rst_sync_rx_r[0], phy_rst_n_w};
            phy_mac_usable_sync_rx_r <= {phy_mac_usable_sync_rx_r[0], phy_mac_usable_tx_r};
        end
    end

    assign rx_enable_w = phy_rst_sync_rx_r[1] & phy_mac_usable_sync_rx_r[1];

    assign ptp_txstamp_valid    = ptp_txstamp_valid_r;
    assign ptp_txstamp_ns       = ptp_txstamp_ns_r;
    assign ptp_rxstamp_valid    = ptp_rxstamp_valid_r;
    assign ptp_rxstamp_ns       = ptp_rxstamp_ns_r;
    assign m_axis_eth_rx_tdata  = m_axis_eth_rx_tdata_r;
    assign m_axis_eth_rx_tvalid = m_axis_eth_rx_tvalid_r;
    assign m_axis_eth_rx_tlast  = m_axis_eth_rx_tlast_r;

    always @(posedge clk125 or negedge mac_tx_resetn) begin
        if (!mac_tx_resetn) begin
            ptp_txstamp_ns_r    <= 64'd0;
            ptp_txstamp_valid_r <= 1'b0;
        end else if (!phy_mac_usable_w) begin
            ptp_txstamp_ns_r    <= 64'd0;
            ptp_txstamp_valid_r <= 1'b0;
        end else begin
            ptp_txstamp_valid_r <= ptp_txstamp_valid_w;
            if (ptp_txstamp_valid_w)
                ptp_txstamp_ns_r <= ptp_txstamp_ns_w;
        end
    end

    always @(posedge rgmii_rxc or negedge mac_rx_resetn) begin
        if (!mac_rx_resetn) begin
            ptp_rxstamp_ns_r       <= 64'd0;
            ptp_rxstamp_valid_r    <= 1'b0;
            m_axis_eth_rx_tdata_r  <= 8'd0;
            m_axis_eth_rx_tvalid_r <= 1'b0;
            m_axis_eth_rx_tlast_r  <= 1'b0;
        end else if (!rx_enable_w) begin
            ptp_rxstamp_ns_r       <= 64'd0;
            ptp_rxstamp_valid_r    <= 1'b0;
            m_axis_eth_rx_tdata_r  <= 8'd0;
            m_axis_eth_rx_tvalid_r <= 1'b0;
            m_axis_eth_rx_tlast_r  <= 1'b0;
        end else begin
            ptp_rxstamp_valid_r    <= ptp_rxstamp_valid_w;
            if (ptp_rxstamp_valid_w)
                ptp_rxstamp_ns_r   <= ptp_rxstamp_ns_w;
            m_axis_eth_rx_tdata_r  <= m_axis_eth_rx_tdata_w;
            m_axis_eth_rx_tvalid_r <= m_axis_eth_rx_tvalid_w;
            m_axis_eth_rx_tlast_r  <= m_axis_eth_rx_tlast_w;
        end
    end

    // ------------------------------------------------------------------------
    // TX path（主线保持不动，仅把放行条件改成“真实可用链路”）
    // ------------------------------------------------------------------------
    top_phy_axis_mac_tx #(
        .MIN_FRAME_BYTES (MIN_FRAME_BYTES),
        .IFG_BYTES       (IFG_BYTES)
    ) u_top_phy_axis_mac_tx (
        .clk                (clk125),
        .rst_n              (mac_tx_resetn),
        .tx_enable          (phy_mac_usable_w),
        .s_axis_tdata       (s_axis_eth_tdata),
        .s_axis_tvalid      (s_axis_eth_tvalid),
        .s_axis_tlast       (s_axis_eth_tlast),
        .s_axis_tready      (s_axis_eth_tready),
        .s_axis_tuser       (s_axis_eth_tuser),
        .ptp_tod_time_ns    (ptp_tod_time_ns),
        .ptp_tod_time_valid (ptp_tod_time_valid),
        .gmii_txd           (gmii_txd_w),
        .gmii_tx_en         (gmii_tx_en_w),
        .gmii_tx_er         (gmii_tx_er_w),
        .ptp_txstamp_ns     (ptp_txstamp_ns_w),
        .ptp_txstamp_valid  (ptp_txstamp_valid_w),
        .tx_busy            (tx_busy_w),
        .stat_frame_count   (stat_frame_count_w),
        .stat_mac_byte_count(stat_mac_byte_count_w),
        .stat_last_error    (stat_last_error_w)
    );

    rgmii_tx_if #(
        .SIMULATION (SIMULATION)
    ) u_rgmii_tx_if (
        .clk          (clk125),
        .clk_shift    (clk125_shift),
        .rst_n        (rst_n),
        .gmii_txd     (gmii_txd_w),
        .gmii_tx_en   (gmii_tx_en_w),
        .gmii_tx_er   (gmii_tx_er_w),
        .rgmii_txc    (rgmii_txc),
        .rgmii_txd    (rgmii_txd),
        .rgmii_tx_ctl (rgmii_tx_ctl)
    );

    // ------------------------------------------------------------------------
    // RX path（保持已有最小闭环实现，仅把放行条件改成“真实可用链路”）
    // ------------------------------------------------------------------------
    rgmii_rx_if u_rgmii_rx_if (
        .rgmii_rxc   (rgmii_rxc),
        .rst_n       (ddr_rx_resetn),
        .rgmii_rxd   (rgmii_rxd),
        .rgmii_rx_ctl(rgmii_rx_ctl),
        .gmii_rxd    (gmii_rxd_w),
        .gmii_rx_dv  (gmii_rx_dv_w),
        .gmii_rx_er  (gmii_rx_er_w)
    );

    top_phy_axis_mac_rx u_top_phy_axis_mac_rx (
        .clk                (rgmii_rxc),
        .rst_n              (mac_rx_resetn),
        .rx_enable          (rx_enable_w),
        .gmii_rxd           (gmii_rxd_w),
        .gmii_rx_dv         (gmii_rx_dv_w),
        .gmii_rx_er         (gmii_rx_er_w),
        .ptp_tod_time_ns    (ptp_tod_time_ns_rx),
        .ptp_tod_time_valid (ptp_tod_time_valid_rx),
        .m_axis_tdata       (m_axis_eth_rx_tdata_w),
        .m_axis_tvalid      (m_axis_eth_rx_tvalid_w),
        .m_axis_tlast       (m_axis_eth_rx_tlast_w),
        .ptp_rxstamp_ns     (ptp_rxstamp_ns_w),
        .ptp_rxstamp_valid  (ptp_rxstamp_valid_w),
        .stat_frame_count   (rx_stat_frame_count_w),
        .stat_drop_count    (rx_stat_drop_count_w),
        .stat_last_error    (rx_stat_last_error_w)
    );

    // 未进一步上报到 CSR 的局部统计先统一 OR，避免综合告警。
    wire _unused_keep;
    assign _unused_keep = stat_frame_count_w[0] ^
                          stat_mac_byte_count_w[0] ^
                          stat_last_error_w[0] ^
                          tx_busy_w ^
                          rx_stat_frame_count_w[0] ^
                          rx_stat_drop_count_w[0] ^
                          rx_stat_last_error_w[0];

endmodule
