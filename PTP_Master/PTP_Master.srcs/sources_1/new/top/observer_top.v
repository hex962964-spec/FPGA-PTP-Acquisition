`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_top
// -----------------------------------------------------------------------------
// Project:
//   fpga_eth_time_observer · PTP Master Endpoint Stage 1
//
// Role:
//   Second FPGA board foundation for the DUT net_acq verification platform.
//   This build emulates the original PC/host endpoint identity used by the
//   DUT software, so the DUT-side MAC/IP configuration does not need to change.
//   This revision keeps the Foundation-v1 RGMII/MAC/ToD/UART bring-up logic and
//   adds the PTP Master endpoint path needed by full E2E closed-loop:
//     - periodic two-step Sync on UDP/319
//     - MAC-SFD TX timestamp capture
//     - Follow_Up on UDP/320 carrying preciseOriginTimestamp
//     - Delay_Req parser in the RX clock domain
//     - Delay_Resp generator on UDP/320 carrying captured t4
//
// Still outside this revision:
//   DUT acquisition packet parser / OBS_RXD logging and TSN slot monitor.
//
// Clocking:
//   - clk_mmcm: 50 MHz input -> sys_clk=50 MHz, eth_tx_clk=125 MHz,
//     eth_tx_clk_90=125 MHz+90 deg.
//   - clk_eth_rx: RGMII RXC -> eth_rx_clk_90.
//   - All local logic uses active-low reset.
//////////////////////////////////////////////////////////////////////////////////
module observer_top #(
    parameter        SIMULATION       = 1'b0,
    parameter [47:0] LOCAL_MAC        = 48'h00_0A_35_01_FE_AA, // original PC/host MAC
    parameter [47:0] DUT_MAC          = 48'h00_0A_35_01_FE_C0, // DUT net_acq MAC
    parameter [47:0] DEBUG_DEST_MAC   = 48'hFF_FF_FF_FF_FF_FF,
    parameter [31:0] LOCAL_IP         = 32'hC0A8_0005, // 192.168.0.5, original PC/host IP
    parameter [31:0] DUT_IP           = 32'hC0A8_000E, // 192.168.0.14, DUT net_acq IP
    parameter [63:0] LOCAL_CLOCK_ID   = 64'h04CB_01FF_FEDA_AADA,
    parameter [15:0] LOCAL_PORT_NUM   = 16'd1,
    parameter [7:0]  PTP_DOMAIN       = 8'd0,
    parameter [31:0] SYS_CLK_FREQ_HZ  = 32'd50000000,
    parameter integer UART_BAUD       = 115200,
    parameter        PTP_MASTER_ENABLE = 1'b1,
    parameter        DEBUG_TX_ENABLE   = 1'b0,
    parameter [31:0] PTP_SYNC_INTERVAL_CYCLES = 32'd25000000,
    parameter [31:0] TEST_TX_INTERVAL_CYCLES  = 32'd125000000
)(
    input  wire       clk_50m,
    input  wire       resetn,

    output wire       rgmii_txc,
    output wire [3:0] rgmii_txd,
    output wire       rgmii_tx_ctl,
    input  wire       rgmii_rxc,
    input  wire [3:0] rgmii_rxd,
    input  wire       rgmii_rx_ctl,

    output wire       phy_resetn,
    output wire       uart_tx,
    output wire [1:0] led
);

    // -------------------------------------------------------------------------
    // Clock generation
    // -------------------------------------------------------------------------
    wire sys_clk_w;
    wire eth_tx_clk_w;
    wire eth_tx_clk_90_w;
    wire eth_rx_clk_90_w;
    wire mmcm_locked_w;
    wire rxclk_locked_w;

    clk_mmcm u_clk_mmcm (
        .sys_clk       (sys_clk_w),
        .eth_tx_clk    (eth_tx_clk_w),
        .eth_tx_clk_90 (eth_tx_clk_90_w),
        .resetn        (resetn),
        .locked        (mmcm_locked_w),
        .clk_in1       (clk_50m)
    );

    clk_eth_rx u_clk_eth_rx (
        .eth_rx_clk_90 (eth_rx_clk_90_w),
        .resetn        (resetn & mmcm_locked_w),
        .locked        (rxclk_locked_w),
        .clk_eth_in    (rgmii_rxc)
    );

    // -------------------------------------------------------------------------
    // Reset synchronizers.  PHY reset is released after the TX clock MMCM locks.
    // -------------------------------------------------------------------------
    wire sys_rst_n_w;
    wire tx_rst_n_w;
    wire rx_rst_n_w;

    assign phy_resetn = resetn & mmcm_locked_w;

    observer_reset_sync u_rst_sys (
        .clk    (sys_clk_w),
        .arst_n (resetn & mmcm_locked_w),
        .rst_n  (sys_rst_n_w)
    );

    observer_reset_sync u_rst_tx (
        .clk    (eth_tx_clk_w),
        .arst_n (resetn & mmcm_locked_w),
        .rst_n  (tx_rst_n_w)
    );

    observer_reset_sync u_rst_rx (
        .clk    (eth_rx_clk_90_w),
        .arst_n (resetn & mmcm_locked_w & rxclk_locked_w),
        .rst_n  (rx_rst_n_w)
    );

    // -------------------------------------------------------------------------
    // Unified ToD fabric.
    // TX-domain ToD is the Observer master time used for Sync/Follow_Up TX
    // timestamps.  RX-domain ToD is periodically loaded from a stable TX-domain
    // snapshot so future Delay_Req/sample RX timestamps use the same epoch.
    // -------------------------------------------------------------------------
    wire [63:0] tx_tod_ns_w;
    wire [47:0] tx_tod_seconds_w;
    wire [31:0] tx_tod_nanoseconds_w;
    wire        tx_tod_valid_w;
    wire        tx_pps_pulse_w;

    wire [63:0] rx_tod_ns_w;
    wire [47:0] rx_tod_seconds_w;
    wire [31:0] rx_tod_nanoseconds_w;
    wire        rx_tod_valid_w;
    wire        rx_pps_pulse_w;
    wire        tod_rx_sync_pending_w;
    wire [31:0] tod_rx_sync_count_w;
    wire [31:0] tod_rx_sync_timeout_count_w;

    observer_tod_fabric #(
        .RX_SYNC_INTERVAL_CYCLES(32'd125000) // 1 ms at 125 MHz
    ) u_tod_fabric (
        .tx_clk              (eth_tx_clk_w),
        .tx_rst_n            (tx_rst_n_w),
        .rx_clk              (eth_rx_clk_90_w),
        .rx_rst_n            (rx_rst_n_w),
        .enable              (1'b1),
        .clear_pulse_tx      (1'b0),
        .tx_tod_ns           (tx_tod_ns_w),
        .tx_tod_seconds      (tx_tod_seconds_w),
        .tx_tod_nanoseconds  (tx_tod_nanoseconds_w),
        .tx_tod_valid        (tx_tod_valid_w),
        .tx_pps_pulse        (tx_pps_pulse_w),
        .rx_tod_ns           (rx_tod_ns_w),
        .rx_tod_seconds      (rx_tod_seconds_w),
        .rx_tod_nanoseconds  (rx_tod_nanoseconds_w),
        .rx_tod_valid        (rx_tod_valid_w),
        .rx_pps_pulse        (rx_pps_pulse_w),
        .rx_sync_pending     (tod_rx_sync_pending_w),
        .rx_sync_count       (tod_rx_sync_count_w),
        .rx_sync_timeout_count(tod_rx_sync_timeout_count_w)
    );

    // -------------------------------------------------------------------------
    // PTP Master TX source: Sync + Follow_Up.
    // -------------------------------------------------------------------------
    wire [7:0]  ptp_tx_tdata_w;
    wire        ptp_tx_tvalid_w;
    wire        ptp_tx_tlast_w;
    wire        ptp_tx_tready_w;
    wire [3:0]  ptp_tx_tuser_w;
    wire        ptp_master_busy_w;
    wire [15:0] ptp_current_seq_w;
    wire [31:0] ptp_sync_count_w;
    wire [31:0] ptp_follow_up_count_w;
    wire [31:0] ptp_timeout_count_w;
    wire [63:0] ptp_last_t1_ns_w;
    wire [47:0] ptp_last_t1_seconds_w;
    wire [31:0] ptp_last_t1_nanoseconds_w;

    wire [63:0] tx_timestamp_ns_w;
    wire [47:0] tx_timestamp_seconds_w;
    wire [31:0] tx_timestamp_nanoseconds_w;
    wire        tx_timestamp_valid_w;

    observer_ptp_master_tx #(
        .LOCAL_MAC              (LOCAL_MAC),
        .DUT_MAC                (DUT_MAC),
        .LOCAL_IP               (LOCAL_IP),
        .DUT_IP                 (DUT_IP),
        .LOCAL_CLOCK_ID         (LOCAL_CLOCK_ID),
        .LOCAL_PORT_NUM         (LOCAL_PORT_NUM),
        .PTP_DOMAIN             (PTP_DOMAIN),
        .SYNC_INTERVAL_CYCLES   (PTP_SYNC_INTERVAL_CYCLES)
    ) u_ptp_master_tx (
        .clk                         (eth_tx_clk_w),
        .rst_n                       (tx_rst_n_w),
        .enable                      (PTP_MASTER_ENABLE),
        .tx_timestamp_valid          (tx_timestamp_valid_w),
        .tx_timestamp_ns             (tx_timestamp_ns_w),
        .tx_timestamp_seconds        (tx_timestamp_seconds_w),
        .tx_timestamp_nanoseconds    (tx_timestamp_nanoseconds_w),
        .m_axis_tdata                (ptp_tx_tdata_w),
        .m_axis_tvalid               (ptp_tx_tvalid_w),
        .m_axis_tlast                (ptp_tx_tlast_w),
        .m_axis_tready               (ptp_tx_tready_w),
        .m_axis_tuser                (ptp_tx_tuser_w),
        .busy                        (ptp_master_busy_w),
        .current_sequence_id         (ptp_current_seq_w),
        .stat_sync_count             (ptp_sync_count_w),
        .stat_follow_up_count        (ptp_follow_up_count_w),
        .stat_txstamp_timeout_count  (ptp_timeout_count_w),
        .stat_last_sync_t1_ns        (ptp_last_t1_ns_w),
        .stat_last_sync_t1_seconds   (ptp_last_t1_seconds_w),
        .stat_last_sync_t1_nanoseconds(ptp_last_t1_nanoseconds_w)
    );

    // -------------------------------------------------------------------------
    // RGMII PHY/MAC boundary wires. Declared before the RX parser to keep
    // default_nettype none builds explicit and tool-friendly.
    // -------------------------------------------------------------------------
    wire [7:0]  rx_axis_tdata_w;
    wire        rx_axis_tvalid_w;
    wire        rx_axis_tlast_w;
    wire [63:0] rx_axis_timestamp_ns_w;
    wire [47:0] rx_axis_timestamp_seconds_w;
    wire [31:0] rx_axis_timestamp_nanoseconds_w;
    wire        rx_axis_timestamp_valid_w;
    wire        tx_busy_w;
    wire        rx_busy_w;
    wire [31:0] tx_frame_count_w;
    wire [31:0] tx_byte_count_w;
    wire [15:0] tx_last_error_w;
    wire [31:0] rx_frame_count_w;
    wire [31:0] rx_drop_count_w;
    wire [15:0] rx_last_error_w;

    // -------------------------------------------------------------------------
    // PTP Delay_Req RX parser + Delay_Resp TX source.
    //
    // The RX parser binds the MAC RX SFD timestamp to the next received frame,
    // accepts DUT UDP/319 PTP Delay_Req messages, and forwards a single record
    // through a req/ack CDC bridge.  The TX source converts that record into a
    // UDP/320 PTP Delay_Resp.  The TX mux gives Delay_Resp priority over the
    // periodic Sync/Follow_Up source so the slave does not wait behind the next
    // periodic master message.
    // -------------------------------------------------------------------------
    localparam integer DREQ_RECORD_WIDTH = 240;

    wire        dreq_rx_record_valid_w;
    wire        dreq_rx_record_ready_w;
    wire [15:0] dreq_rx_sequence_id_w;
    wire [79:0] dreq_rx_source_port_identity_w;
    wire [63:0] dreq_rx_t4_ns_w;
    wire [47:0] dreq_rx_t4_seconds_w;
    wire [31:0] dreq_rx_t4_nanoseconds_w;

    wire [DREQ_RECORD_WIDTH-1:0] dreq_rx_record_bus_w;
    wire [DREQ_RECORD_WIDTH-1:0] dreq_tx_record_bus_w;
    wire        dreq_tx_record_valid_w;
    wire        dreq_tx_record_ready_w;

    wire [15:0] dreq_tx_sequence_id_w;
    wire [79:0] dreq_tx_source_port_identity_w;
    wire [63:0] dreq_tx_t4_ns_w;
    wire [47:0] dreq_tx_t4_seconds_w;
    wire [31:0] dreq_tx_t4_nanoseconds_w;

    wire [31:0] dreq_stat_count_rx_w;
    wire [31:0] dreq_stat_drop_rx_w;
    wire [31:0] dreq_stat_non_delay_ptp_rx_w;
    wire [15:0] dreq_stat_last_seq_rx_w;
    wire [63:0] dreq_stat_last_t4_ns_rx_w;
    wire [15:0] dreq_stat_last_error_rx_w;
    wire [31:0] dreq_cdc_drop_count_w;

    wire [7:0]  dresp_tx_tdata_w;
    wire        dresp_tx_tvalid_w;
    wire        dresp_tx_tlast_w;
    wire        dresp_tx_tready_w;
    wire [3:0]  dresp_tx_tuser_w;
    wire        dresp_busy_w;
    wire [31:0] dresp_stat_count_tx_w;
    wire [15:0] dresp_stat_last_seq_tx_w;
    wire [63:0] dresp_stat_last_t4_ns_tx_w;

    assign dreq_rx_record_bus_w = {dreq_rx_t4_nanoseconds_w,
                                   dreq_rx_t4_seconds_w,
                                   dreq_rx_t4_ns_w,
                                   dreq_rx_source_port_identity_w,
                                   dreq_rx_sequence_id_w};

    assign dreq_tx_sequence_id_w            = dreq_tx_record_bus_w[15:0];
    assign dreq_tx_source_port_identity_w   = dreq_tx_record_bus_w[95:16];
    assign dreq_tx_t4_ns_w                  = dreq_tx_record_bus_w[159:96];
    assign dreq_tx_t4_seconds_w             = dreq_tx_record_bus_w[207:160];
    assign dreq_tx_t4_nanoseconds_w         = dreq_tx_record_bus_w[239:208];

    observer_ptp_delay_req_rx #(
        .LOCAL_MAC  (LOCAL_MAC),
        .DUT_MAC    (DUT_MAC),
        .LOCAL_IP   (LOCAL_IP),
        .DUT_IP     (DUT_IP),
        .PTP_DOMAIN (PTP_DOMAIN)
    ) u_ptp_delay_req_rx (
        .clk                           (eth_rx_clk_90_w),
        .rst_n                         (rx_rst_n_w),
        .enable                        (PTP_MASTER_ENABLE),
        .s_axis_tdata                  (rx_axis_tdata_w),
        .s_axis_tvalid                 (rx_axis_tvalid_w),
        .s_axis_tlast                  (rx_axis_tlast_w),
        .s_frame_timestamp_ns          (rx_axis_timestamp_ns_w),
        .s_frame_timestamp_seconds     (rx_axis_timestamp_seconds_w),
        .s_frame_timestamp_nanoseconds (rx_axis_timestamp_nanoseconds_w),
        .s_frame_timestamp_valid       (rx_axis_timestamp_valid_w),
        .m_record_valid                (dreq_rx_record_valid_w),
        .m_record_ready                (dreq_rx_record_ready_w),
        .m_sequence_id                 (dreq_rx_sequence_id_w),
        .m_source_port_identity        (dreq_rx_source_port_identity_w),
        .m_t4_ns                       (dreq_rx_t4_ns_w),
        .m_t4_seconds                  (dreq_rx_t4_seconds_w),
        .m_t4_nanoseconds              (dreq_rx_t4_nanoseconds_w),
        .stat_delay_req_count          (dreq_stat_count_rx_w),
        .stat_delay_req_drop_count     (dreq_stat_drop_rx_w),
        .stat_non_delay_ptp_count      (dreq_stat_non_delay_ptp_rx_w),
        .stat_last_sequence_id         (dreq_stat_last_seq_rx_w),
        .stat_last_t4_ns               (dreq_stat_last_t4_ns_rx_w),
        .stat_last_error               (dreq_stat_last_error_rx_w)
    );

    observer_ptp_delay_req_cdc #(
        .WIDTH(DREQ_RECORD_WIDTH)
    ) u_ptp_delay_req_cdc (
        .src_clk         (eth_rx_clk_90_w),
        .src_rst_n       (rx_rst_n_w),
        .src_valid_pulse (dreq_rx_record_valid_w),
        .src_ready       (dreq_rx_record_ready_w),
        .src_record_bus  (dreq_rx_record_bus_w),
        .src_drop_count  (dreq_cdc_drop_count_w),
        .dst_clk         (eth_tx_clk_w),
        .dst_rst_n       (tx_rst_n_w),
        .dst_valid_pulse (dreq_tx_record_valid_w),
        .dst_ready       (dreq_tx_record_ready_w),
        .dst_record_bus  (dreq_tx_record_bus_w)
    );

    observer_ptp_delay_resp_tx #(
        .LOCAL_MAC      (LOCAL_MAC),
        .DUT_MAC        (DUT_MAC),
        .LOCAL_IP       (LOCAL_IP),
        .DUT_IP         (DUT_IP),
        .LOCAL_CLOCK_ID (LOCAL_CLOCK_ID),
        .LOCAL_PORT_NUM (LOCAL_PORT_NUM),
        .PTP_DOMAIN     (PTP_DOMAIN)
    ) u_ptp_delay_resp_tx (
        .clk                         (eth_tx_clk_w),
        .rst_n                       (tx_rst_n_w),
        .enable                      (PTP_MASTER_ENABLE),
        .s_record_valid              (dreq_tx_record_valid_w),
        .s_record_ready              (dreq_tx_record_ready_w),
        .s_sequence_id               (dreq_tx_sequence_id_w),
        .s_requesting_port_identity  (dreq_tx_source_port_identity_w),
        .s_t4_ns                     (dreq_tx_t4_ns_w),
        .s_t4_seconds                (dreq_tx_t4_seconds_w),
        .s_t4_nanoseconds            (dreq_tx_t4_nanoseconds_w),
        .m_axis_tdata                (dresp_tx_tdata_w),
        .m_axis_tvalid               (dresp_tx_tvalid_w),
        .m_axis_tlast                (dresp_tx_tlast_w),
        .m_axis_tready               (dresp_tx_tready_w),
        .m_axis_tuser                (dresp_tx_tuser_w),
        .busy                        (dresp_busy_w),
        .stat_delay_resp_count       (dresp_stat_count_tx_w),
        .stat_last_sequence_id       (dresp_stat_last_seq_tx_w),
        .stat_last_t4_ns             (dresp_stat_last_t4_ns_tx_w)
    );

    // -------------------------------------------------------------------------
    // Optional debug frame source retained from Foundation v1.  It is disabled
    // by default once PTP master traffic is enabled.
    // -------------------------------------------------------------------------
    wire [7:0]  dbg_tx_tdata_w;
    wire        dbg_tx_tvalid_w;
    wire        dbg_tx_tlast_w;
    wire        dbg_tx_tready_w;
    wire [3:0]  dbg_tx_tuser_w;
    wire [31:0] test_tx_frame_count_w;

    observer_test_frame_gen #(
        .LOCAL_MAC       (LOCAL_MAC),
        .DEST_MAC        (DEBUG_DEST_MAC),
        .INTERVAL_CYCLES (TEST_TX_INTERVAL_CYCLES),
        .ENABLE_DEFAULT  (1'b1)
    ) u_test_frame_gen (
        .clk              (eth_tx_clk_w),
        .rst_n            (tx_rst_n_w),
        .enable           (DEBUG_TX_ENABLE),
        .tod_ns           (tx_tod_ns_w),
        .m_axis_tdata     (dbg_tx_tdata_w),
        .m_axis_tvalid    (dbg_tx_tvalid_w),
        .m_axis_tlast     (dbg_tx_tlast_w),
        .m_axis_tready    (dbg_tx_tready_w),
        .m_axis_tuser     (dbg_tx_tuser_w),
        .stat_frame_count (test_tx_frame_count_w)
    );

    // -------------------------------------------------------------------------
    // TX mux: Delay_Resp has highest priority, periodic Sync/Follow_Up is next,
    // debug frames are lowest priority.
    // -------------------------------------------------------------------------
    wire [7:0]  tx_axis_tdata_w;
    wire        tx_axis_tvalid_w;
    wire        tx_axis_tlast_w;
    wire        tx_axis_tready_w;
    wire [3:0]  tx_axis_tuser_w;
    wire [31:0] mux_ptp_frame_count_w;
    wire [31:0] mux_debug_frame_count_w;

    wire [31:0] mux_delay_resp_frame_count_w;

    observer_tx_mux u_tx_mux (
        .clk                         (eth_tx_clk_w),
        .rst_n                       (tx_rst_n_w),
        .s0_tdata                    (dresp_tx_tdata_w),
        .s0_tvalid                   (dresp_tx_tvalid_w),
        .s0_tlast                    (dresp_tx_tlast_w),
        .s0_tready                   (dresp_tx_tready_w),
        .s0_tuser                    (dresp_tx_tuser_w),
        .s1_tdata                    (ptp_tx_tdata_w),
        .s1_tvalid                   (ptp_tx_tvalid_w),
        .s1_tlast                    (ptp_tx_tlast_w),
        .s1_tready                   (ptp_tx_tready_w),
        .s1_tuser                    (ptp_tx_tuser_w),
        .s2_tdata                    (dbg_tx_tdata_w),
        .s2_tvalid                   (dbg_tx_tvalid_w),
        .s2_tlast                    (dbg_tx_tlast_w),
        .s2_tready                   (dbg_tx_tready_w),
        .s2_tuser                    (dbg_tx_tuser_w),
        .m_tdata                     (tx_axis_tdata_w),
        .m_tvalid                    (tx_axis_tvalid_w),
        .m_tlast                     (tx_axis_tlast_w),
        .m_tready                    (tx_axis_tready_w),
        .m_tuser                     (tx_axis_tuser_w),
        .busy                        (),
        .stat_delay_resp_frame_count (mux_delay_resp_frame_count_w),
        .stat_ptp_frame_count        (mux_ptp_frame_count_w),
        .stat_debug_frame_count      (mux_debug_frame_count_w)
    );

    // -------------------------------------------------------------------------
    // RGMII PHY/MAC boundary.
    // -------------------------------------------------------------------------

    observer_top_phy #(
        .SIMULATION(SIMULATION)
    ) u_top_phy (
        .tx_clk                    (eth_tx_clk_w),
        .tx_clk_90                 (eth_tx_clk_90_w),
        .tx_rst_n                  (tx_rst_n_w),
        .rx_clk_90                 (eth_rx_clk_90_w),
        .rx_rst_n                  (rx_rst_n_w),
        .tx_enable                 (1'b1),
        .rx_enable                 (1'b1),
        .tx_tod_time_ns            (tx_tod_ns_w),
        .tx_tod_seconds            (tx_tod_seconds_w),
        .tx_tod_nanoseconds        (tx_tod_nanoseconds_w),
        .tx_tod_time_valid         (tx_tod_valid_w),
        .rx_tod_time_ns            (rx_tod_ns_w),
        .rx_tod_seconds            (rx_tod_seconds_w),
        .rx_tod_nanoseconds        (rx_tod_nanoseconds_w),
        .rx_tod_time_valid         (rx_tod_valid_w),
        .s_axis_tx_tdata           (tx_axis_tdata_w),
        .s_axis_tx_tvalid          (tx_axis_tvalid_w),
        .s_axis_tx_tlast           (tx_axis_tlast_w),
        .s_axis_tx_tready          (tx_axis_tready_w),
        .s_axis_tx_tuser           (tx_axis_tuser_w),
        .m_axis_rx_tdata           (rx_axis_tdata_w),
        .m_axis_rx_tvalid          (rx_axis_tvalid_w),
        .m_axis_rx_tlast           (rx_axis_tlast_w),
        .m_axis_rx_timestamp_ns          (rx_axis_timestamp_ns_w),
        .m_axis_rx_timestamp_seconds     (rx_axis_timestamp_seconds_w),
        .m_axis_rx_timestamp_nanoseconds (rx_axis_timestamp_nanoseconds_w),
        .m_axis_rx_timestamp_valid       (rx_axis_timestamp_valid_w),
        .tx_timestamp_ns           (tx_timestamp_ns_w),
        .tx_timestamp_seconds      (tx_timestamp_seconds_w),
        .tx_timestamp_nanoseconds  (tx_timestamp_nanoseconds_w),
        .tx_timestamp_valid        (tx_timestamp_valid_w),
        .rgmii_txc                 (rgmii_txc),
        .rgmii_txd                 (rgmii_txd),
        .rgmii_tx_ctl              (rgmii_tx_ctl),
        .rgmii_rxd                 (rgmii_rxd),
        .rgmii_rx_ctl              (rgmii_rx_ctl),
        .tx_busy                   (tx_busy_w),
        .rx_busy                   (rx_busy_w),
        .tx_frame_count            (tx_frame_count_w),
        .tx_byte_count             (tx_byte_count_w),
        .tx_last_error             (tx_last_error_w),
        .rx_frame_count            (rx_frame_count_w),
        .rx_drop_count             (rx_drop_count_w),
        .rx_last_error             (rx_last_error_w)
    );

    // -------------------------------------------------------------------------
    // TX/RX-domain diagnostic counters.  These are intentionally simple
    // free-running observability counters used to debug whether the recovered RX
    // clock domain is alive, whether bytes are entering the MAC RX stream, and
    // whether valid SFD timestamps are produced.  They are sampled into sys_clk
    // by the status CDC bridges below and printed in the CLK/MAC/PTP UART lines.
    // -------------------------------------------------------------------------
    reg [31:0] tx_heartbeat_count_r;
    reg [31:0] rx_heartbeat_count_r;
    reg [31:0] rx_axis_byte_count_r;
    reg [31:0] rx_axis_last_count_r;
    reg [31:0] rx_axis_timestamp_count_r;

    always @(posedge eth_tx_clk_w or negedge tx_rst_n_w) begin
        if (!tx_rst_n_w)
            tx_heartbeat_count_r <= 32'd0;
        else
            tx_heartbeat_count_r <= tx_heartbeat_count_r + 32'd1;
    end

    always @(posedge eth_rx_clk_90_w or negedge rx_rst_n_w) begin
        if (!rx_rst_n_w) begin
            rx_heartbeat_count_r      <= 32'd0;
            rx_axis_byte_count_r      <= 32'd0;
            rx_axis_last_count_r      <= 32'd0;
            rx_axis_timestamp_count_r <= 32'd0;
        end else begin
            rx_heartbeat_count_r <= rx_heartbeat_count_r + 32'd1;
            if (rx_axis_tvalid_w)
                rx_axis_byte_count_r <= rx_axis_byte_count_r + 32'd1;
            if (rx_axis_tvalid_w && rx_axis_tlast_w)
                rx_axis_last_count_r <= rx_axis_last_count_r + 32'd1;
            if (rx_axis_timestamp_valid_w)
                rx_axis_timestamp_count_r <= rx_axis_timestamp_count_r + 32'd1;
        end
    end

    // Local keepers for status/debug sidebands that are reserved for later
    // OBS_RXD/OBS_TSN logging or not yet printed in the UART diagnostics.
    wire unused_debug_keep_w;
    assign unused_debug_keep_w = ^{tx_pps_pulse_w, rx_pps_pulse_w,
                                  ptp_last_t1_seconds_w, ptp_last_t1_nanoseconds_w,
                                  rx_tod_ns_w, rx_tod_seconds_w, rx_tod_nanoseconds_w};

    // -------------------------------------------------------------------------
    // CDC snapshots for UART telemetry.
    //
    // TX and RX domain status are captured by explicit req/ack snapshot bridges.
    // The UART monitor waits for both domains, or times out after 10 ms so a
    // missing RX clock will not stop the serial diagnostics.
    // -------------------------------------------------------------------------
    localparam integer TX_STAT_WIDTH = 800;  // 25 x 32-bit words
    localparam integer RX_STAT_WIDTH = 544;  // 17 x 32-bit words
    localparam [31:0]  STAT_WAIT_TIMEOUT_CYCLES = (SYS_CLK_FREQ_HZ / 32'd100); // 10 ms

    wire [TX_STAT_WIDTH-1:0] tx_stat_sys_w;
    wire [RX_STAT_WIDTH-1:0] rx_stat_sys_w;
    wire                    tx_stat_valid_w;
    wire                    rx_stat_valid_w;
    reg                     stat_req_pulse_r;

    observer_status_cdc_bridge #(
        .WIDTH(TX_STAT_WIDTH)
    ) u_tx_status_cdc (
        .sys_clk        (sys_clk_w),
        .sys_rst_n      (sys_rst_n_w),
        .sys_req_pulse  (stat_req_pulse_r),
        .sys_status_bus (tx_stat_sys_w),
        .sys_valid_pulse(tx_stat_valid_w),
        .src_clk        (eth_tx_clk_w),
        .src_rst_n      (tx_rst_n_w),
        .src_status_bus ({tod_rx_sync_timeout_count_w,              // w24
                          tx_tod_ns_w[63:32],                       // w23
                          test_tx_frame_count_w,                    // w22
                          {29'd0, tx_tod_valid_w, dresp_busy_w, ptp_master_busy_w}, // w21
                          tx_tod_ns_w[31:0],                        // w20
                          tx_heartbeat_count_r,                     // w19
                          mux_debug_frame_count_w,                  // w18
                          mux_ptp_frame_count_w,                    // w17
                          mux_delay_resp_frame_count_w,             // w16
                          dresp_stat_last_t4_ns_tx_w[63:32],        // w15
                          dresp_stat_last_t4_ns_tx_w[31:0],         // w14
                          {16'd0, dresp_stat_last_seq_tx_w},        // w13
                          dresp_stat_count_tx_w,                    // w12
                          tod_rx_sync_count_w,                      // w11
                          ptp_last_t1_nanoseconds_w,                // w10
                          ptp_last_t1_seconds_w[31:0],              // w9
                          ptp_last_t1_ns_w[63:32],                  // w8
                          ptp_last_t1_ns_w[31:0],                   // w7
                          {16'd0, ptp_current_seq_w},               // w6
                          ptp_timeout_count_w,                      // w5
                          ptp_follow_up_count_w,                    // w4
                          ptp_sync_count_w,                         // w3
                          {16'd0, tx_last_error_w},                 // w2
                          tx_byte_count_w,                          // w1
                          tx_frame_count_w})                        // w0
    );

    observer_status_cdc_bridge #(
        .WIDTH(RX_STAT_WIDTH)
    ) u_rx_status_cdc (
        .sys_clk        (sys_clk_w),
        .sys_rst_n      (sys_rst_n_w),
        .sys_req_pulse  (stat_req_pulse_r),
        .sys_status_bus (rx_stat_sys_w),
        .sys_valid_pulse(rx_stat_valid_w),
        .src_clk        (eth_rx_clk_90_w),
        .src_rst_n      (rx_rst_n_w),
        .src_status_bus ({rx_axis_timestamp_count_r,                 // w16
                          {31'd0, rx_tod_valid_w},                  // w15
                          {31'd0, rx_busy_w},                       // w14
                          rx_tod_ns_w[63:32],                       // w13
                          rx_tod_ns_w[31:0],                        // w12
                          rx_heartbeat_count_r,                     // w11
                          dreq_cdc_drop_count_w,                    // w10
                          {16'd0, dreq_stat_last_error_rx_w},       // w9
                          dreq_stat_last_t4_ns_rx_w[63:32],         // w8
                          dreq_stat_last_t4_ns_rx_w[31:0],          // w7
                          {16'd0, dreq_stat_last_seq_rx_w},         // w6
                          dreq_stat_non_delay_ptp_rx_w,             // w5
                          dreq_stat_drop_rx_w,                      // w4
                          dreq_stat_count_rx_w,                     // w3
                          {16'd0, rx_last_error_w},                 // w2
                          rx_drop_count_w,                          // w1
                          rx_frame_count_w})                        // w0
    );

    // -------------------------------------------------------------------------
    // UART telemetry scheduler in sys_clk domain.
    //
    // Current output per second:
    //   STAT,... old-compatible one-line summary
    //   CLK,...  clock/reset/ToD monitor
    //   PTP,...  Sync/FUP master counters and t1
    //   TX,...   MAC TX / mux / Delay_Resp counters
    //   RX,...   MAC RX / Delay_Req parser counters
    //   CFG,...  static MAC/IP/domain/version line
    //   DBG,...  extra RX timestamp / ToD-sync recovery counters
    // -------------------------------------------------------------------------
    localparam [1:0] ST_MON_IDLE = 2'd0;
    localparam [1:0] ST_MON_WAIT = 2'd1;
    localparam [31:0] OBSERVER_BUILD_VERSION = 32'h20260524;
    localparam [31:0] UART_BAUD_WORD = UART_BAUD;

    reg [1:0]  mon_state_r;
    reg [31:0] sys_cycle_cnt_r;
    reg [31:0] uptime_s_r;
    reg        sec_tick_r;
    reg        have_tx_stat_r;
    reg        have_rx_stat_r;
    reg        telemetry_start_r;
    reg [31:0] stat_wait_cnt_r;

    reg [31:0] tx_frame_count_sys_r;
    reg [31:0] tx_byte_count_sys_r;
    reg [31:0] tx_last_error_sys_r;
    reg [31:0] tx_heartbeat_count_sys_r;
    reg [31:0] tx_tod_low_sys_r;
    reg [31:0] tx_tod_high_sys_r;
    reg [31:0] ptp_sync_count_sys_r;
    reg [31:0] ptp_follow_up_count_sys_r;
    reg [31:0] ptp_timeout_count_sys_r;
    reg [31:0] ptp_sequence_sys_r;
    reg [31:0] ptp_last_t1_low_sys_r;
    reg [31:0] ptp_last_t1_high_sys_r;
    reg [31:0] ptp_last_t1_seconds_low_sys_r;
    reg [31:0] ptp_last_t1_nanoseconds_sys_r;
    reg [31:0] ptp_master_busy_sys_r;
    reg [31:0] tx_tod_valid_sys_r;
    reg [31:0] tod_rx_sync_count_sys_r;
    reg [31:0] tod_rx_sync_timeout_count_sys_r;
    reg [31:0] mux_delay_resp_frame_count_sys_r;
    reg [31:0] mux_ptp_frame_count_sys_r;
    reg [31:0] mux_debug_frame_count_sys_r;
    reg [31:0] delay_resp_count_sys_r;
    reg [31:0] delay_resp_last_seq_sys_r;
    reg [31:0] delay_resp_last_t4_low_sys_r;
    reg [31:0] delay_resp_last_t4_high_sys_r;
    reg [31:0] delay_resp_busy_sys_r;
    reg [31:0] test_tx_frame_count_sys_r;

    reg [31:0] rx_frame_count_sys_r;
    reg [31:0] rx_drop_count_sys_r;
    reg [31:0] rx_last_error_sys_r;
    reg [31:0] rx_heartbeat_count_sys_r;
    reg [31:0] rx_tod_low_sys_r;
    reg [31:0] rx_tod_high_sys_r;
    reg [31:0] rx_busy_sys_r;
    reg [31:0] rx_tod_valid_word_sys_r;
    reg [31:0] rx_axis_timestamp_count_sys_r;
    reg [31:0] delay_req_count_sys_r;
    reg [31:0] delay_req_drop_count_sys_r;
    reg [31:0] delay_req_non_delay_ptp_count_sys_r;
    reg [31:0] delay_req_last_seq_sys_r;
    reg [31:0] delay_req_last_t4_low_sys_r;
    reg [31:0] delay_req_last_t4_high_sys_r;
    reg [31:0] delay_req_last_error_sys_r;
    reg [31:0] delay_req_cdc_drop_count_sys_r;

    reg [31:0] telemetry_timeout_count_r;
    reg [31:0] telemetry_last_wait_cycles_r;

    wire [7:0] status_flags_w;
    wire [7:0] uart_data_w;
    wire       uart_valid_w;
    wire       uart_ready_w;
    wire       telemetry_busy_w;
    wire       rx_status_available_w;

    // Synchronize status-only flags into sys_clk before using them in the UART
    // monitor.  These flags are not control signals for the datapath, but keeping
    // them synchronized avoids sampling RX/TX-domain status bits directly.
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg mmcm_locked_sys0_r, mmcm_locked_sys1_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg rxclk_locked_sys0_r, rxclk_locked_sys1_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg tx_rstn_sys0_r, tx_rstn_sys1_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg rx_rstn_sys0_r, rx_rstn_sys1_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg rx_tod_valid_sys0_r, rx_tod_valid_sys1_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg tod_sync_pending_sys0_r, tod_sync_pending_sys1_r;

    always @(posedge sys_clk_w or negedge sys_rst_n_w) begin
        if (!sys_rst_n_w) begin
            mmcm_locked_sys0_r       <= 1'b0;
            mmcm_locked_sys1_r       <= 1'b0;
            rxclk_locked_sys0_r      <= 1'b0;
            rxclk_locked_sys1_r      <= 1'b0;
            tx_rstn_sys0_r           <= 1'b0;
            tx_rstn_sys1_r           <= 1'b0;
            rx_rstn_sys0_r           <= 1'b0;
            rx_rstn_sys1_r           <= 1'b0;
            rx_tod_valid_sys0_r      <= 1'b0;
            rx_tod_valid_sys1_r      <= 1'b0;
            tod_sync_pending_sys0_r  <= 1'b0;
            tod_sync_pending_sys1_r  <= 1'b0;
        end else begin
            mmcm_locked_sys0_r       <= mmcm_locked_w;
            mmcm_locked_sys1_r       <= mmcm_locked_sys0_r;
            rxclk_locked_sys0_r      <= rxclk_locked_w;
            rxclk_locked_sys1_r      <= rxclk_locked_sys0_r;
            tx_rstn_sys0_r           <= tx_rst_n_w;
            tx_rstn_sys1_r           <= tx_rstn_sys0_r;
            rx_rstn_sys0_r           <= rx_rst_n_w;
            rx_rstn_sys1_r           <= rx_rstn_sys0_r;
            rx_tod_valid_sys0_r      <= rx_tod_valid_w;
            rx_tod_valid_sys1_r      <= rx_tod_valid_sys0_r;
            tod_sync_pending_sys0_r  <= tod_rx_sync_pending_w;
            tod_sync_pending_sys1_r  <= tod_sync_pending_sys0_r;
        end
    end

    assign rx_status_available_w = rxclk_locked_sys1_r & rx_rstn_sys1_r;
    assign status_flags_w = {DEBUG_TX_ENABLE, tod_sync_pending_sys1_r, rx_tod_valid_sys1_r,
                             PTP_MASTER_ENABLE, rx_rstn_sys1_r, tx_rstn_sys1_r,
                             rxclk_locked_sys1_r, mmcm_locked_sys1_r};

    wire [31:0] observer_status_ext_w;
    assign observer_status_ext_w = {16'd0,
                                    telemetry_busy_w,
                                    have_rx_stat_r,
                                    have_tx_stat_r,
                                    rx_status_available_w,
                                    rx_tod_valid_sys1_r,
                                    tod_sync_pending_sys1_r,
                                    rx_rstn_sys1_r,
                                    tx_rstn_sys1_r,
                                    rxclk_locked_sys1_r,
                                    mmcm_locked_sys1_r,
                                    PTP_MASTER_ENABLE,
                                    DEBUG_TX_ENABLE,
                                    4'd0};

    always @(posedge sys_clk_w or negedge sys_rst_n_w) begin
        if (!sys_rst_n_w) begin
            sys_cycle_cnt_r             <= 32'd0;
            uptime_s_r                  <= 32'd0;
            sec_tick_r                  <= 1'b0;
            stat_req_pulse_r            <= 1'b0;
            mon_state_r                 <= ST_MON_IDLE;
            have_tx_stat_r              <= 1'b0;
            have_rx_stat_r              <= 1'b0;
            telemetry_start_r           <= 1'b0;
            stat_wait_cnt_r             <= 32'd0;

            tx_frame_count_sys_r        <= 32'd0;
            tx_byte_count_sys_r         <= 32'd0;
            tx_last_error_sys_r         <= 32'd0;
            tx_heartbeat_count_sys_r    <= 32'd0;
            tx_tod_low_sys_r            <= 32'd0;
            tx_tod_high_sys_r           <= 32'd0;
            ptp_sync_count_sys_r        <= 32'd0;
            ptp_follow_up_count_sys_r   <= 32'd0;
            ptp_timeout_count_sys_r     <= 32'd0;
            ptp_sequence_sys_r          <= 32'd0;
            ptp_last_t1_low_sys_r       <= 32'd0;
            ptp_last_t1_high_sys_r      <= 32'd0;
            ptp_last_t1_seconds_low_sys_r <= 32'd0;
            ptp_last_t1_nanoseconds_sys_r <= 32'd0;
            ptp_master_busy_sys_r       <= 32'd0;
            tx_tod_valid_sys_r          <= 32'd0;
            tod_rx_sync_count_sys_r     <= 32'd0;
            tod_rx_sync_timeout_count_sys_r <= 32'd0;
            mux_delay_resp_frame_count_sys_r <= 32'd0;
            mux_ptp_frame_count_sys_r   <= 32'd0;
            mux_debug_frame_count_sys_r <= 32'd0;
            delay_resp_count_sys_r      <= 32'd0;
            delay_resp_last_seq_sys_r   <= 32'd0;
            delay_resp_last_t4_low_sys_r<= 32'd0;
            delay_resp_last_t4_high_sys_r<= 32'd0;
            delay_resp_busy_sys_r       <= 32'd0;
            test_tx_frame_count_sys_r   <= 32'd0;

            rx_frame_count_sys_r        <= 32'd0;
            rx_drop_count_sys_r         <= 32'd0;
            rx_last_error_sys_r         <= 32'd0;
            rx_heartbeat_count_sys_r    <= 32'd0;
            rx_tod_low_sys_r            <= 32'd0;
            rx_tod_high_sys_r           <= 32'd0;
            rx_busy_sys_r               <= 32'd0;
            rx_tod_valid_word_sys_r     <= 32'd0;
            rx_axis_timestamp_count_sys_r <= 32'd0;
            delay_req_count_sys_r       <= 32'd0;
            delay_req_drop_count_sys_r  <= 32'd0;
            delay_req_non_delay_ptp_count_sys_r <= 32'd0;
            delay_req_last_seq_sys_r    <= 32'd0;
            delay_req_last_t4_low_sys_r <= 32'd0;
            delay_req_last_t4_high_sys_r<= 32'd0;
            delay_req_last_error_sys_r  <= 32'd0;
            delay_req_cdc_drop_count_sys_r <= 32'd0;

            telemetry_timeout_count_r   <= 32'd0;
            telemetry_last_wait_cycles_r<= 32'd0;
        end else begin
            sec_tick_r        <= 1'b0;
            stat_req_pulse_r  <= 1'b0;
            telemetry_start_r <= 1'b0;

            if (sys_cycle_cnt_r == (SYS_CLK_FREQ_HZ - 32'd1)) begin
                sys_cycle_cnt_r <= 32'd0;
                uptime_s_r      <= uptime_s_r + 32'd1;
                sec_tick_r      <= 1'b1;
            end else begin
                sys_cycle_cnt_r <= sys_cycle_cnt_r + 32'd1;
            end

            if (tx_stat_valid_w) begin
                tx_frame_count_sys_r      <= tx_stat_sys_w[31:0];
                tx_byte_count_sys_r       <= tx_stat_sys_w[63:32];
                tx_last_error_sys_r       <= tx_stat_sys_w[95:64];
                ptp_sync_count_sys_r      <= tx_stat_sys_w[127:96];
                ptp_follow_up_count_sys_r <= tx_stat_sys_w[159:128];
                ptp_timeout_count_sys_r   <= tx_stat_sys_w[191:160];
                ptp_sequence_sys_r        <= tx_stat_sys_w[223:192];
                ptp_last_t1_low_sys_r     <= tx_stat_sys_w[255:224];
                ptp_last_t1_high_sys_r    <= tx_stat_sys_w[287:256];
                ptp_last_t1_seconds_low_sys_r <= tx_stat_sys_w[319:288];
                ptp_last_t1_nanoseconds_sys_r <= tx_stat_sys_w[351:320];
                tod_rx_sync_count_sys_r   <= tx_stat_sys_w[383:352];
                delay_resp_count_sys_r    <= tx_stat_sys_w[415:384];
                delay_resp_last_seq_sys_r <= tx_stat_sys_w[447:416];
                delay_resp_last_t4_low_sys_r <= tx_stat_sys_w[479:448];
                delay_resp_last_t4_high_sys_r<= tx_stat_sys_w[511:480];
                mux_delay_resp_frame_count_sys_r <= tx_stat_sys_w[543:512];
                mux_ptp_frame_count_sys_r <= tx_stat_sys_w[575:544];
                mux_debug_frame_count_sys_r <= tx_stat_sys_w[607:576];
                tx_heartbeat_count_sys_r  <= tx_stat_sys_w[639:608];
                tx_tod_low_sys_r          <= tx_stat_sys_w[671:640];
                ptp_master_busy_sys_r     <= {31'd0, tx_stat_sys_w[672]};
                delay_resp_busy_sys_r     <= {31'd0, tx_stat_sys_w[673]};
                tx_tod_valid_sys_r        <= {31'd0, tx_stat_sys_w[674]};
                test_tx_frame_count_sys_r <= tx_stat_sys_w[735:704];
                tx_tod_high_sys_r         <= tx_stat_sys_w[767:736];
                tod_rx_sync_timeout_count_sys_r <= tx_stat_sys_w[799:768];
                have_tx_stat_r            <= 1'b1;
            end

            if (rx_stat_valid_w) begin
                rx_frame_count_sys_r       <= rx_stat_sys_w[31:0];
                rx_drop_count_sys_r        <= rx_stat_sys_w[63:32];
                rx_last_error_sys_r        <= rx_stat_sys_w[95:64];
                delay_req_count_sys_r      <= rx_stat_sys_w[127:96];
                delay_req_drop_count_sys_r <= rx_stat_sys_w[159:128];
                delay_req_non_delay_ptp_count_sys_r <= rx_stat_sys_w[191:160];
                delay_req_last_seq_sys_r   <= rx_stat_sys_w[223:192];
                delay_req_last_t4_low_sys_r<= rx_stat_sys_w[255:224];
                delay_req_last_t4_high_sys_r<= rx_stat_sys_w[287:256];
                delay_req_last_error_sys_r <= rx_stat_sys_w[319:288];
                delay_req_cdc_drop_count_sys_r <= rx_stat_sys_w[351:320];
                rx_heartbeat_count_sys_r   <= rx_stat_sys_w[383:352];
                rx_tod_low_sys_r           <= rx_stat_sys_w[415:384];
                rx_tod_high_sys_r          <= rx_stat_sys_w[447:416];
                rx_busy_sys_r              <= rx_stat_sys_w[479:448];
                rx_tod_valid_word_sys_r    <= rx_stat_sys_w[511:480];
                rx_axis_timestamp_count_sys_r <= rx_stat_sys_w[543:512];
                have_rx_stat_r             <= 1'b1;
            end

            case (mon_state_r)
                ST_MON_IDLE: begin
                    if (sec_tick_r && !telemetry_busy_w) begin
                        have_tx_stat_r   <= 1'b0;
                        have_rx_stat_r   <= !rx_status_available_w;
                        stat_wait_cnt_r  <= 32'd0;
                        stat_req_pulse_r <= 1'b1;
                        if (!rx_status_available_w) begin
                            rx_frame_count_sys_r       <= 32'd0;
                            rx_drop_count_sys_r        <= 32'd0;
                            rx_last_error_sys_r        <= 32'd0;
                            rx_heartbeat_count_sys_r   <= 32'd0;
                            rx_tod_low_sys_r           <= 32'd0;
                            rx_tod_high_sys_r          <= 32'd0;
                            rx_busy_sys_r              <= 32'd0;
                            rx_tod_valid_word_sys_r    <= 32'd0;
                            rx_axis_timestamp_count_sys_r <= 32'd0;
                            delay_req_count_sys_r      <= 32'd0;
                            delay_req_drop_count_sys_r <= 32'd0;
                            delay_req_non_delay_ptp_count_sys_r <= 32'd0;
                            delay_req_last_error_sys_r <= 32'd0;
                            delay_req_cdc_drop_count_sys_r <= 32'd0;
                            delay_req_last_seq_sys_r   <= 32'd0;
                            delay_req_last_t4_low_sys_r<= 32'd0;
                            delay_req_last_t4_high_sys_r<= 32'd0;
                        end
                        mon_state_r      <= ST_MON_WAIT;
                    end
                end

                ST_MON_WAIT: begin
                    stat_wait_cnt_r <= stat_wait_cnt_r + 32'd1;
                    if ((have_tx_stat_r && have_rx_stat_r) ||
                        (stat_wait_cnt_r >= STAT_WAIT_TIMEOUT_CYCLES)) begin
                        telemetry_start_r             <= !telemetry_busy_w;
                        telemetry_last_wait_cycles_r  <= stat_wait_cnt_r;
                        if (stat_wait_cnt_r >= STAT_WAIT_TIMEOUT_CYCLES)
                            telemetry_timeout_count_r <= telemetry_timeout_count_r + 32'd1;
                        mon_state_r                   <= ST_MON_IDLE;
                    end
                end

                default: begin
                    mon_state_r <= ST_MON_IDLE;
                end
            endcase
        end
    end

    observer_telemetry_formatter u_telemetry_formatter (
        .clk                 (sys_clk_w),
        .rst_n               (sys_rst_n_w),
        .start_pulse         (telemetry_start_r),

        .uptime_s            (uptime_s_r),
        .rx_frame_count      (rx_frame_count_sys_r),
        .rx_drop_count       (rx_drop_count_sys_r),
        .tx_frame_count      (tx_frame_count_sys_r),
        .status_flags        (status_flags_w),
        .ptp_sync_count      (ptp_sync_count_sys_r),
        .ptp_follow_up_count (ptp_follow_up_count_sys_r),
        .ptp_timeout_count   (ptp_timeout_count_sys_r),
        .ptp_sequence_id     (ptp_sequence_sys_r),
        .ptp_last_t1_low     (ptp_last_t1_low_sys_r),
        .tod_rx_sync_count   (tod_rx_sync_count_sys_r),
        .delay_req_count     (delay_req_count_sys_r),
        .delay_resp_count    (delay_resp_count_sys_r),
        .delay_req_drop_count(delay_req_drop_count_sys_r),
        .delay_req_last_seq  (delay_req_last_seq_sys_r),
        .delay_req_last_t4_low(delay_req_last_t4_low_sys_r),

        .status_ext          (observer_status_ext_w),
        .tx_heartbeat_count  (tx_heartbeat_count_sys_r),
        .rx_heartbeat_count  (rx_heartbeat_count_sys_r),
        .tx_tod_low          (tx_tod_low_sys_r),
        .tx_tod_high         (tx_tod_high_sys_r),
        .rx_tod_low          (rx_tod_low_sys_r),
        .rx_tod_high         (rx_tod_high_sys_r),
        .stat_timeout_count  (telemetry_timeout_count_r),
        .stat_last_wait_cycles(telemetry_last_wait_cycles_r),
        .rx_status_available ({31'd0, rx_status_available_w}),
        .tx_tod_valid        (tx_tod_valid_sys_r),
        .rx_tod_valid        (rx_tod_valid_word_sys_r),
        .tod_sync_pending    ({31'd0, tod_sync_pending_sys1_r}),
        .tod_sync_timeout_count(tod_rx_sync_timeout_count_sys_r),
        .rx_axis_timestamp_count(rx_axis_timestamp_count_sys_r),

        .ptp_last_t1_high    (ptp_last_t1_high_sys_r),
        .ptp_last_t1_seconds_low(ptp_last_t1_seconds_low_sys_r),
        .ptp_last_t1_nanoseconds(ptp_last_t1_nanoseconds_sys_r),
        .ptp_master_busy     (ptp_master_busy_sys_r),

        .tx_byte_count       (tx_byte_count_sys_r),
        .tx_last_error       (tx_last_error_sys_r),
        .mux_delay_resp_frame_count(mux_delay_resp_frame_count_sys_r),
        .mux_ptp_frame_count (mux_ptp_frame_count_sys_r),
        .mux_debug_frame_count(mux_debug_frame_count_sys_r),
        .delay_resp_last_seq (delay_resp_last_seq_sys_r),
        .delay_resp_last_t4_low(delay_resp_last_t4_low_sys_r),
        .delay_resp_last_t4_high(delay_resp_last_t4_high_sys_r),
        .delay_resp_busy     (delay_resp_busy_sys_r),
        .test_tx_frame_count (test_tx_frame_count_sys_r),

        .rx_last_error       (rx_last_error_sys_r),
        .rx_busy             (rx_busy_sys_r),
        .delay_req_non_delay_ptp_count(delay_req_non_delay_ptp_count_sys_r),
        .delay_req_last_error(delay_req_last_error_sys_r),
        .delay_req_cdc_drop_count(delay_req_cdc_drop_count_sys_r),
        .delay_req_last_t4_high(delay_req_last_t4_high_sys_r),

        .local_mac_low       (LOCAL_MAC[31:0]),
        .local_mac_high      ({16'd0, LOCAL_MAC[47:32]}),
        .dut_mac_low         (DUT_MAC[31:0]),
        .dut_mac_high        ({16'd0, DUT_MAC[47:32]}),
        .local_ip            (LOCAL_IP),
        .dut_ip              (DUT_IP),
        .ptp_domain          ({24'd0, PTP_DOMAIN}),
        .ptp_sync_interval   (PTP_SYNC_INTERVAL_CYCLES),
        .uart_baud           (UART_BAUD_WORD),
        .sys_clk_freq        (SYS_CLK_FREQ_HZ),
        .build_version       (OBSERVER_BUILD_VERSION),

        .m_data              (uart_data_w),
        .m_valid             (uart_valid_w),
        .m_ready             (uart_ready_w),
        .busy                (telemetry_busy_w)
    );

    observer_uart_tx #(
        .CLK_FREQ_HZ(SYS_CLK_FREQ_HZ),
        .BAUD_RATE  (UART_BAUD)
    ) u_uart_tx (
        .clk     (sys_clk_w),
        .rst_n   (sys_rst_n_w),
        .s_data  (uart_data_w),
        .s_valid (uart_valid_w),
        .s_ready (uart_ready_w),
        .uart_tx (uart_tx),
        .busy    ()
    );

    // -------------------------------------------------------------------------
    // Two-LED board status output.
    //   led[0] : TX/system clock tree is locked and released from reset.
    //   led[1] : RX clock tree is locked and released from reset.
    // The previous four-LED debug assignment is intentionally compressed here
    // because the target observer board exposes only two LEDs.
    // -------------------------------------------------------------------------
    assign led[0] = mmcm_locked_w & tx_rst_n_w;
    assign led[1] = rxclk_locked_w & rx_rst_n_w;

endmodule
`default_nettype wire
