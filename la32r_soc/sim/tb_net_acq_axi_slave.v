`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Testbench: tb_net_acq_axi_slave
// Purpose:
//   1) Drive net_acq_axi_slave by AXI single-beat register accesses only.
//   2) Exercise normal acquisition packet generation through the internal data_gen.
//   3) Exercise compact PTP interaction without C code:
//        - configure ETH/PTP registers;
//        - set ToD;
//        - inject Sync / Follow_Up / Delay_Resp frames on the RX byte stream;
//        - trigger a Delay_Req TX command and emulate the MAC-side TX timestamp.
//
// Suggested paper wave groups:
//   A. Acquisition timestamp:
//      dut.u_net_acq_wrapper.u_net_acq_frontend_subsystem.u_acquisition_core.u_sampler.*
//   B. Descriptor/payload FIFO + packet formatter:
//      dut.u_net_acq_wrapper.u_net_acq_frontend_subsystem.u_acquisition_core.*packet_info*
//      dut.u_net_acq_wrapper.u_net_acq_frontend_subsystem.u_acquisition_core.*sample_fifo*
//      dut.u_net_acq_wrapper.u_net_acq_frontend_subsystem.u_acquisition_core.u_packet_formatter.*
//   C. Packet CDC:
//      dut.u_net_acq_wrapper.u_net_acq_pkt_async_fifo.*
//   D. Safe commit:
//      dut.u_net_acq_wrapper.u_net_acq_eth_cfg_manager.*commit*, *req*, *ack*, *active*
//   E. PTP TX timestamp:
//      dut.u_net_acq_wrapper.u_net_acq_ptp_subsystem.u_ptp_tx_path.*
//      dut.u_net_acq_wrapper.u_net_acq_ptp_subsystem.u_ptp_cfg_cmd_manager.*txstamp*
//   F. PTP RX mailbox:
//      dut.u_net_acq_wrapper.u_net_acq_ptp_subsystem.u_ptp_rx_path.*
//   G. TSN-lite scheduler, if enabled:
//      dut.u_net_acq_wrapper.u_net_acq_egress_subsystem.u_net_acq_data_tx_scheduler.*
//////////////////////////////////////////////////////////////////////////////////

module tb_net_acq_axi_slave;

    // -------------------------------------------------------------------------
    // Clock and reset
    // -------------------------------------------------------------------------
    reg aclk;
    reg aresetn;
    reg eth_tx_clk;
    reg eth_tx_resetn;
    reg eth_rx_clk;
    reg eth_rx_resetn;

    initial begin
        aclk = 1'b0;
        forever #10 aclk = ~aclk;          // 50 MHz sys / AXI clock
    end

    initial begin
        eth_tx_clk = 1'b0;
        forever #4 eth_tx_clk = ~eth_tx_clk; // 125 MHz TX clock
    end

    initial begin
        eth_rx_clk = 1'b0;
        forever #4 eth_rx_clk = ~eth_rx_clk; // 125 MHz RX clock
    end

    // -------------------------------------------------------------------------
    // AXI signals
    // -------------------------------------------------------------------------
    reg  [4:0]  s_awid;
    reg  [31:0] s_awaddr;
    reg  [7:0]  s_awlen;
    reg  [2:0]  s_awsize;
    reg  [1:0]  s_awburst;
    reg         s_awlock;
    reg  [3:0]  s_awcache;
    reg  [2:0]  s_awprot;
    reg         s_awvalid;
    wire        s_awready;

    reg  [4:0]  s_wid;
    reg  [31:0] s_wdata;
    reg  [3:0]  s_wstrb;
    reg         s_wlast;
    reg         s_wvalid;
    wire        s_wready;

    reg         s_bready;
    wire [4:0]  s_bid;
    wire [1:0]  s_bresp;
    wire        s_bvalid;

    reg  [4:0]  s_arid;
    reg  [31:0] s_araddr;
    reg  [7:0]  s_arlen;
    reg  [2:0]  s_arsize;
    reg  [1:0]  s_arburst;
    reg         s_arlock;
    reg  [3:0]  s_arcache;
    reg  [2:0]  s_arprot;
    reg         s_arvalid;
    wire        s_arready;

    reg         s_rready;
    wire [4:0]  s_rid;
    wire [31:0] s_rdata;
    wire [1:0]  s_rresp;
    wire        s_rlast;
    wire        s_rvalid;

    // -------------------------------------------------------------------------
    // DUT sideband
    // -------------------------------------------------------------------------
    reg  [15:0] sample_data_in;
    reg         sample_valid_in;
    wire        sample_ready_out;

    wire [15:0] pkt_data;
    wire        pkt_valid;
    wire        pkt_last;
    reg         pkt_ready;

    wire [7:0]  m_axis_eth_tx_tdata;
    wire        m_axis_eth_tx_tvalid;
    wire        m_axis_eth_tx_tlast;
    reg         m_axis_eth_tx_tready;
    wire [3:0]  m_axis_eth_tx_tuser;

    wire [63:0] ptp_tod_time_ns_eth;
    wire        ptp_tod_time_valid_eth;

    reg  [7:0]  s_axis_eth_rx_tdata;
    reg         s_axis_eth_rx_tvalid;
    reg         s_axis_eth_rx_tlast;
    reg  [63:0] ptp_rxstamp_ns;
    reg         ptp_rxstamp_valid;
    wire [63:0] ptp_tod_time_ns_rx;
    wire        ptp_tod_time_valid_rx;

    reg  [63:0] tod_time_ns;
    reg         tod_time_valid;

    wire [63:0] ptp_txstamp_ns;
    wire        ptp_txstamp_valid;

    reg         mac_link_up;
    reg  [31:0] phy_status_word;
    reg  [127:0] phy_reg_snapshot;
    wire        phy_soft_reset_pulse;
    wire        phy_restart_an_pulse;

    // Emulate top_phy MAC TX SFD timestamp return.
    // The egress subsystem puts SOF on tuser[0] and PTP need_timestamp on tuser[1].
    assign ptp_txstamp_valid = m_axis_eth_tx_tvalid &&
                               m_axis_eth_tx_tready &&
                               m_axis_eth_tx_tuser[0] &&
                               m_axis_eth_tx_tuser[1];
    assign ptp_txstamp_ns    = ptp_tod_time_ns_eth;

    // -------------------------------------------------------------------------
    // Register offsets
    // -------------------------------------------------------------------------
    localparam [31:0] R_CTRL                  = 32'h000;
    localparam [31:0] R_MODE                  = 32'h004;
    localparam [31:0] R_SAMPLE_PERIOD         = 32'h008;
    localparam [31:0] R_SAMPLES_PER_PKT       = 32'h00C;
    localparam [31:0] R_CHANNEL_ID            = 32'h010;
    localparam [31:0] R_STATUS                = 32'h014;
    localparam [31:0] R_PACKET_COUNT          = 32'h018;
    localparam [31:0] R_FIFO_LEVEL            = 32'h024;
    localparam [31:0] R_DBG_GOOD_PKT_CNT      = 32'h02C;
    localparam [31:0] R_DBG_BAD_PKT_CNT       = 32'h030;
    localparam [31:0] R_DBG_LAST_COUNTS       = 32'h040;
    localparam [31:0] R_DBG_LAST_ERROR        = 32'h044;

    localparam [31:0] R_ETH_CTRL              = 32'h080;
    localparam [31:0] R_ETH_STATUS            = 32'h084;
    localparam [31:0] R_LOCAL_MAC_L           = 32'h088;
    localparam [31:0] R_LOCAL_MAC_H           = 32'h08C;
    localparam [31:0] R_DST_MAC_L             = 32'h090;
    localparam [31:0] R_DST_MAC_H             = 32'h094;
    localparam [31:0] R_LOCAL_IP              = 32'h098;
    localparam [31:0] R_DST_IP                = 32'h09C;
    localparam [31:0] R_UDP_PORTS             = 32'h0A0;
    localparam [31:0] R_IPV4_CFG              = 32'h0A4;
    localparam [31:0] R_ETH_TX_PKT_COUNT      = 32'h0A8;
    localparam [31:0] R_ETH_LAST_LENGTHS      = 32'h0B4;

    localparam [31:0] R_PTP_CFG               = 32'h0C0;
    localparam [31:0] R_PTP_CLOCK_ID_L        = 32'h0C4;
    localparam [31:0] R_PTP_CLOCK_ID_H        = 32'h0C8;
    localparam [31:0] R_PTP_PORT_NUM          = 32'h0CC;
    localparam [31:0] R_PTP_TXSTAMP_L         = 32'h0D0;
    localparam [31:0] R_PTP_TXSTAMP_H         = 32'h0D4;
    localparam [31:0] R_PTP_STATUS            = 32'h0D8;
    localparam [31:0] R_PTP_CTRL              = 32'h0DC;
    localparam [31:0] R_PTP_TOD_SET_L         = 32'h0E0;
    localparam [31:0] R_PTP_TOD_SET_H         = 32'h0E4;
    localparam [31:0] R_PTP_FREQ_ADJ          = 32'h0EC;
    localparam [31:0] R_PTP_CUR_TIME_L        = 32'h0F0;
    localparam [31:0] R_PTP_CUR_TIME_H        = 32'h0F4;
    localparam [31:0] R_PTP_TX_CFG            = 32'h0F8;

    localparam [31:0] R_PTP_RX_MBOX_STATUS    = 32'h100;
    localparam [31:0] R_PTP_RX_RXTS_L         = 32'h104;
    localparam [31:0] R_PTP_RX_RXTS_H         = 32'h108;
    localparam [31:0] R_PTP_RX_MSGTS_L        = 32'h10C;
    localparam [31:0] R_PTP_RX_MSGTS_M        = 32'h110;
    localparam [31:0] R_PTP_RX_MSGTS_H        = 32'h114;
    localparam [31:0] R_PTP_RX_REQPORT_L      = 32'h118;
    localparam [31:0] R_PTP_RX_REQPORT_M      = 32'h11C;
    localparam [31:0] R_PTP_RX_REQPORT_H      = 32'h120;
    localparam [31:0] R_PTP_RX_MBOX_CTRL      = 32'h124;

    localparam [31:0] R_DBGEXT_ARB_DATA_CNT   = 32'h144;
    localparam [31:0] R_DBGEXT_ARB_PTP_CNT    = 32'h148;
    localparam [31:0] R_DBGEXT_PTP_TX_REQ_CNT = 32'h158;
    localparam [31:0] R_DBGEXT_PTP_TX_ACC_CNT = 32'h15C;
    localparam [31:0] R_DBGEXT_PTP_RX_ACC_CNT = 32'h168;
    localparam [31:0] R_DBGEXT_PTP_RX_DROP_CNT= 32'h16C;

    localparam [3:0]  MSG_SYNC                = 4'h0;
    localparam [3:0]  MSG_FOLLOW_UP           = 4'h8;
    localparam [3:0]  MSG_DELAY_RESP          = 4'h9;

    localparam [47:0] DUT_MAC                 = 48'h000A3501FEC0;
    localparam [47:0] OBS_MAC                 = 48'h04CB01DAAADA;
    localparam [31:0] DUT_IP                  = 32'hC0A8000E; // 192.168.0.14
    localparam [31:0] OBS_IP                  = 32'hC0A80003; // 192.168.0.3
    localparam [63:0] DUT_CLOCK_ID            = 64'h000A3501FEC00001;
    localparam [15:0] DUT_PORT_NUM            = 16'h0001;
    localparam [79:0] DUT_PORT_ID_80          = 80'h000A3501FEC000010001;

    // -------------------------------------------------------------------------
    // DUT instance
    // -------------------------------------------------------------------------
    net_acq_axi_slave dut (
        .aclk                   (aclk),
        .aresetn                (aresetn),
        .eth_tx_clk             (eth_tx_clk),
        .eth_tx_resetn          (eth_tx_resetn),
        .eth_rx_clk             (eth_rx_clk),
        .eth_rx_resetn          (eth_rx_resetn),
        .s_awid                 (s_awid),
        .s_awaddr               (s_awaddr),
        .s_awlen                (s_awlen),
        .s_awsize               (s_awsize),
        .s_awburst              (s_awburst),
        .s_awlock               (s_awlock),
        .s_awcache              (s_awcache),
        .s_awprot               (s_awprot),
        .s_awvalid              (s_awvalid),
        .s_awready              (s_awready),
        .s_wid                  (s_wid),
        .s_wdata                (s_wdata),
        .s_wstrb                (s_wstrb),
        .s_wlast                (s_wlast),
        .s_wvalid               (s_wvalid),
        .s_wready               (s_wready),
        .s_bready               (s_bready),
        .s_bid                  (s_bid),
        .s_bresp                (s_bresp),
        .s_bvalid               (s_bvalid),
        .s_arid                 (s_arid),
        .s_araddr               (s_araddr),
        .s_arlen                (s_arlen),
        .s_arsize               (s_arsize),
        .s_arburst              (s_arburst),
        .s_arlock               (s_arlock),
        .s_arcache              (s_arcache),
        .s_arprot               (s_arprot),
        .s_arvalid              (s_arvalid),
        .s_arready              (s_arready),
        .s_rready               (s_rready),
        .s_rid                  (s_rid),
        .s_rdata                (s_rdata),
        .s_rresp                (s_rresp),
        .s_rlast                (s_rlast),
        .s_rvalid               (s_rvalid),
        .sample_data_in         (sample_data_in),
        .sample_valid_in        (sample_valid_in),
        .sample_ready_out       (sample_ready_out),
        .pkt_data               (pkt_data),
        .pkt_valid              (pkt_valid),
        .pkt_last               (pkt_last),
        .pkt_ready              (pkt_ready),
        .m_axis_eth_tx_tdata    (m_axis_eth_tx_tdata),
        .m_axis_eth_tx_tvalid   (m_axis_eth_tx_tvalid),
        .m_axis_eth_tx_tlast    (m_axis_eth_tx_tlast),
        .m_axis_eth_tx_tready   (m_axis_eth_tx_tready),
        .m_axis_eth_tx_tuser    (m_axis_eth_tx_tuser),
        .ptp_tod_time_ns_eth    (ptp_tod_time_ns_eth),
        .ptp_tod_time_valid_eth (ptp_tod_time_valid_eth),
        .s_axis_eth_rx_tdata    (s_axis_eth_rx_tdata),
        .s_axis_eth_rx_tvalid   (s_axis_eth_rx_tvalid),
        .s_axis_eth_rx_tlast    (s_axis_eth_rx_tlast),
        .ptp_rxstamp_ns         (ptp_rxstamp_ns),
        .ptp_rxstamp_valid      (ptp_rxstamp_valid),
        .ptp_tod_time_ns_rx     (ptp_tod_time_ns_rx),
        .ptp_tod_time_valid_rx  (ptp_tod_time_valid_rx),
        .tod_time_ns            (tod_time_ns),
        .tod_time_valid         (tod_time_valid),
        .ptp_txstamp_ns         (ptp_txstamp_ns),
        .ptp_txstamp_valid      (ptp_txstamp_valid),
        .mac_link_up            (mac_link_up),
        .phy_status_word        (phy_status_word),
        .phy_reg_snapshot       (phy_reg_snapshot),
        .phy_soft_reset_pulse   (phy_soft_reset_pulse),
        .phy_restart_an_pulse   (phy_restart_an_pulse)
    );

    // -------------------------------------------------------------------------
    // Utility functions
    // -------------------------------------------------------------------------
    function [79:0] ns_to_ptp80;
        input [63:0] ns;
        reg   [63:0] seconds;
        reg   [63:0] nanos;
        begin
            seconds     = ns / 64'd1000000000;
            nanos       = ns % 64'd1000000000;
            ns_to_ptp80 = {seconds[47:0], nanos[31:0]};
        end
    endfunction

    function [7:0] ptp_frame_byte;
        input integer idx;
        input [3:0] msg_type;
        input [15:0] seq_id;
        input [79:0] msg_timestamp_80;
        input [79:0] req_port_id_80;
        reg [15:0] udp_port;
        reg [15:0] ptp_len;
        begin
            udp_port = (msg_type == MSG_SYNC) ? 16'd319 : 16'd320;
            ptp_len  = (msg_type == MSG_DELAY_RESP) ? 16'd54 : 16'd44;
            ptp_frame_byte = 8'h00;
            case (idx)
                // Ethernet header
                0:  ptp_frame_byte = DUT_MAC[47:40];
                1:  ptp_frame_byte = DUT_MAC[39:32];
                2:  ptp_frame_byte = DUT_MAC[31:24];
                3:  ptp_frame_byte = DUT_MAC[23:16];
                4:  ptp_frame_byte = DUT_MAC[15:8];
                5:  ptp_frame_byte = DUT_MAC[7:0];
                6:  ptp_frame_byte = OBS_MAC[47:40];
                7:  ptp_frame_byte = OBS_MAC[39:32];
                8:  ptp_frame_byte = OBS_MAC[31:24];
                9:  ptp_frame_byte = OBS_MAC[23:16];
                10: ptp_frame_byte = OBS_MAC[15:8];
                11: ptp_frame_byte = OBS_MAC[7:0];
                12: ptp_frame_byte = 8'h08;
                13: ptp_frame_byte = 8'h00;

                // IPv4 header, no checksum model is required by DUT PTP filter
                14: ptp_frame_byte = 8'h45;
                15: ptp_frame_byte = 8'h00;
                16: ptp_frame_byte = 8'h00;
                17: ptp_frame_byte = 8'h52; // total length 82 bytes
                18: ptp_frame_byte = 8'h12;
                19: ptp_frame_byte = 8'h34;
                20: ptp_frame_byte = 8'h00;
                21: ptp_frame_byte = 8'h00;
                22: ptp_frame_byte = 8'h40;
                23: ptp_frame_byte = 8'h11; // UDP
                24: ptp_frame_byte = 8'h00;
                25: ptp_frame_byte = 8'h00;
                26: ptp_frame_byte = OBS_IP[31:24];
                27: ptp_frame_byte = OBS_IP[23:16];
                28: ptp_frame_byte = OBS_IP[15:8];
                29: ptp_frame_byte = OBS_IP[7:0];
                30: ptp_frame_byte = DUT_IP[31:24];
                31: ptp_frame_byte = DUT_IP[23:16];
                32: ptp_frame_byte = DUT_IP[15:8];
                33: ptp_frame_byte = DUT_IP[7:0];

                // UDP header
                34: ptp_frame_byte = udp_port[15:8];
                35: ptp_frame_byte = udp_port[7:0];
                36: ptp_frame_byte = udp_port[15:8];
                37: ptp_frame_byte = udp_port[7:0];
                38: ptp_frame_byte = 8'h00;
                39: ptp_frame_byte = 8'h3E; // UDP length 62 bytes
                40: ptp_frame_byte = 8'h00;
                41: ptp_frame_byte = 8'h00;

                // PTP common header at byte 42
                42: ptp_frame_byte = {4'h0, msg_type};
                43: ptp_frame_byte = 8'h02;
                44: ptp_frame_byte = ptp_len[15:8];
                45: ptp_frame_byte = ptp_len[7:0];
                46: ptp_frame_byte = 8'h00; // domain
                47: ptp_frame_byte = 8'h00;
                48: ptp_frame_byte = 8'h02; // flags: two-step style placeholder
                49: ptp_frame_byte = 8'h00;
                50: ptp_frame_byte = 8'h00;
                51: ptp_frame_byte = 8'h00;
                52: ptp_frame_byte = 8'h00;
                53: ptp_frame_byte = 8'h00;
                54: ptp_frame_byte = 8'h00;
                55: ptp_frame_byte = 8'h00;
                56: ptp_frame_byte = OBS_MAC[47:40];
                57: ptp_frame_byte = OBS_MAC[39:32];
                58: ptp_frame_byte = OBS_MAC[31:24];
                59: ptp_frame_byte = OBS_MAC[23:16];
                60: ptp_frame_byte = OBS_MAC[15:8];
                61: ptp_frame_byte = OBS_MAC[7:0];
                62: ptp_frame_byte = 8'h00;
                63: ptp_frame_byte = 8'h01;
                72: ptp_frame_byte = seq_id[15:8];
                73: ptp_frame_byte = seq_id[7:0];
                74: ptp_frame_byte = 8'h00;
                75: ptp_frame_byte = 8'h7F;

                // messageTimestamp / preciseOriginTimestamp / receiveTimestamp
                76: ptp_frame_byte = msg_timestamp_80[79:72];
                77: ptp_frame_byte = msg_timestamp_80[71:64];
                78: ptp_frame_byte = msg_timestamp_80[63:56];
                79: ptp_frame_byte = msg_timestamp_80[55:48];
                80: ptp_frame_byte = msg_timestamp_80[47:40];
                81: ptp_frame_byte = msg_timestamp_80[39:32];
                82: ptp_frame_byte = msg_timestamp_80[31:24];
                83: ptp_frame_byte = msg_timestamp_80[23:16];
                84: ptp_frame_byte = msg_timestamp_80[15:8];
                85: ptp_frame_byte = msg_timestamp_80[7:0];

                // requestingPortIdentity, only meaningful for Delay_Resp
                86: ptp_frame_byte = req_port_id_80[79:72];
                87: ptp_frame_byte = req_port_id_80[71:64];
                88: ptp_frame_byte = req_port_id_80[63:56];
                89: ptp_frame_byte = req_port_id_80[55:48];
                90: ptp_frame_byte = req_port_id_80[47:40];
                91: ptp_frame_byte = req_port_id_80[39:32];
                92: ptp_frame_byte = req_port_id_80[31:24];
                93: ptp_frame_byte = req_port_id_80[23:16];
                94: ptp_frame_byte = req_port_id_80[15:8];
                95: ptp_frame_byte = req_port_id_80[7:0];
                default: ptp_frame_byte = 8'h00;
            endcase
        end
    endfunction

    // -------------------------------------------------------------------------
    // AXI tasks
    // -------------------------------------------------------------------------
    task axi_write32;
        input [31:0] addr;
        input [31:0] data;
        integer wait_count;
        begin
            @(posedge aclk);
            s_awid    <= 5'd1;
            s_awaddr  <= addr;
            s_awlen   <= 8'd0;
            s_awsize  <= 3'b010;
            s_awburst <= 2'b01;
            s_awlock  <= 1'b0;
            s_awcache <= 4'd0;
            s_awprot  <= 3'd0;
            s_awvalid <= 1'b1;

            s_wid     <= 5'd1;
            s_wdata   <= data;
            s_wstrb   <= 4'hF;
            s_wlast   <= 1'b1;
            s_wvalid  <= 1'b1;

            wait_count = 0;
            while (!(s_awready && s_wready)) begin
                @(posedge aclk);
                wait_count = wait_count + 1;
                if (wait_count > 1000) begin
                    $display("[%0t] ERROR: AXI write handshake timeout addr=%h", $time, addr);
                    $stop;
                end
            end

            @(posedge aclk);
            s_awvalid <= 1'b0;
            s_wvalid  <= 1'b0;
            s_wlast   <= 1'b0;

            wait_count = 0;
            while (!s_bvalid) begin
                @(posedge aclk);
                wait_count = wait_count + 1;
                if (wait_count > 2000) begin
                    $display("[%0t] ERROR: AXI write response timeout addr=%h", $time, addr);
                    $stop;
                end
            end
            if (s_bresp != 2'b00) begin
                $display("[%0t] ERROR: AXI write SLVERR addr=%h data=%h bresp=%b", $time, addr, data, s_bresp);
                $stop;
            end
            @(posedge aclk);
        end
    endtask

    task axi_read32;
        input  [31:0] addr;
        output [31:0] data;
        integer wait_count;
        begin
            @(posedge aclk);
            s_arid    <= 5'd2;
            s_araddr  <= addr;
            s_arlen   <= 8'd0;
            s_arsize  <= 3'b010;
            s_arburst <= 2'b01;
            s_arlock  <= 1'b0;
            s_arcache <= 4'd0;
            s_arprot  <= 3'd0;
            s_arvalid <= 1'b1;

            wait_count = 0;
            while (!s_arready) begin
                @(posedge aclk);
                wait_count = wait_count + 1;
                if (wait_count > 1000) begin
                    $display("[%0t] ERROR: AXI read address timeout addr=%h", $time, addr);
                    $stop;
                end
            end

            @(posedge aclk);
            s_arvalid <= 1'b0;

            wait_count = 0;
            while (!s_rvalid) begin
                @(posedge aclk);
                wait_count = wait_count + 1;
                if (wait_count > 2000) begin
                    $display("[%0t] ERROR: AXI read data timeout addr=%h", $time, addr);
                    $stop;
                end
            end
            data = s_rdata;
            if (s_rresp != 2'b00) begin
                $display("[%0t] ERROR: AXI read SLVERR addr=%h rresp=%b", $time, addr, s_rresp);
                $stop;
            end
            @(posedge aclk);
        end
    endtask

    task wait_reg_bit_set;
        input [31:0] addr;
        input integer bit_index;
        input integer max_polls;
        reg [31:0] data;
        integer i;
        integer ok;
        begin
            ok = 0;
            for (i = 0; i < max_polls; i = i + 1) begin
                axi_read32(addr, data);
                if (data[bit_index]) begin
                    ok = 1;
                    i = max_polls;
                end else begin
                    repeat (4) @(posedge aclk);
                end
            end
            if (!ok) begin
                $display("[%0t] ERROR: timeout waiting bit %0d at reg %h", $time, bit_index, addr);
                $stop;
            end
        end
    endtask


    task wait_reg_nonzero;
        input [31:0] addr;
        input integer max_polls;
        reg [31:0] data;
        integer i;
        integer ok;
        begin
            ok = 0;
            for (i = 0; i < max_polls; i = i + 1) begin
                axi_read32(addr, data);
                if (data != 32'd0) begin
                    ok = 1;
                    i = max_polls;
                end else begin
                    repeat (4) @(posedge aclk);
                end
            end
            if (!ok) begin
                $display("[%0t] ERROR: timeout waiting nonzero reg %h", $time, addr);
                $stop;
            end
        end
    endtask

    task wait_mbox_and_pop;
        input [3:0] expected_msg_type;
        input [15:0] expected_seq;
        reg [31:0] status;
        reg [31:0] lo;
        reg [31:0] hi;
        reg [31:0] m0;
        reg [31:0] m1;
        reg [31:0] m2;
        reg [31:0] r0;
        reg [31:0] r1;
        reg [31:0] r2;
        integer i;
        integer ok;
        begin
            ok = 0;
            for (i = 0; i < 512; i = i + 1) begin
                axi_read32(R_PTP_RX_MBOX_STATUS, status);
                if (status[0]) begin
                    ok = 1;
                    i = 512;
                end else begin
                    repeat (8) @(posedge aclk);
                end
            end
            if (!ok) begin
                $display("[%0t] ERROR: PTP RX mailbox timeout expected type=%0d seq=%0d", $time, expected_msg_type, expected_seq);
                $stop;
            end

            if ((status[7:4] != expected_msg_type) || (status[31:16] != expected_seq)) begin
                $display("[%0t] ERROR: PTP mailbox mismatch: status=%h expected type=%0d seq=%0d", $time, status, expected_msg_type, expected_seq);
                $stop;
            end

            axi_read32(R_PTP_RX_RXTS_L, lo);
            axi_read32(R_PTP_RX_RXTS_H, hi);
            axi_read32(R_PTP_RX_MSGTS_L, m0);
            axi_read32(R_PTP_RX_MSGTS_M, m1);
            axi_read32(R_PTP_RX_MSGTS_H, m2);
            axi_read32(R_PTP_RX_REQPORT_L, r0);
            axi_read32(R_PTP_RX_REQPORT_M, r1);
            axi_read32(R_PTP_RX_REQPORT_H, r2);
            $display("[%0t] PTP MBOX type=%0d seq=%0d rx_ts=0x%08h_%08h msgts=0x%04h_%08h_%08h reqport=0x%04h_%08h_%08h",
                     $time, status[7:4], status[31:16], hi, lo,
                     m2[15:0], m1, m0, r2[15:0], r1, r0);
            axi_write32(R_PTP_RX_MBOX_CTRL, 32'h1);
            repeat (10) @(posedge aclk);
        end
    endtask

    task wait_txstamp_and_consume;
        output [63:0] stamp_ns;
        reg [31:0] status;
        reg [31:0] lo;
        reg [31:0] hi;
        integer i;
        integer ok;
        begin
            ok = 0;
            for (i = 0; i < 512; i = i + 1) begin
                axi_read32(R_PTP_STATUS, status);
                if (status[0]) begin
                    ok = 1;
                    i = 512;
                end else begin
                    repeat (8) @(posedge aclk);
                end
            end
            if (!ok) begin
                $display("[%0t] ERROR: TXSTAMP valid timeout", $time);
                $stop;
            end
            // Contract: low then high consumes the single outstanding timestamp.
            axi_read32(R_PTP_TXSTAMP_L, lo);
            axi_read32(R_PTP_TXSTAMP_H, hi);
            stamp_ns = {hi, lo};
            $display("[%0t] TXSTAMP consumed: 0x%08h_%08h", $time, hi, lo);
            repeat (10) @(posedge aclk);
        end
    endtask

    task send_ptp_frame;
        input [3:0]  msg_type;
        input [15:0] seq_id;
        input [79:0] msg_timestamp_80;
        input [79:0] req_port_id_80;
        input [63:0] rxstamp;
        integer i;
        begin
            $display("[%0t] Inject PTP frame type=%0d seq=%0d rxstamp=0x%016h", $time, msg_type, seq_id, rxstamp);
            for (i = 0; i < 96; i = i + 1) begin
                @(negedge eth_rx_clk);
                s_axis_eth_rx_tdata  = ptp_frame_byte(i, msg_type, seq_id, msg_timestamp_80, req_port_id_80);
                s_axis_eth_rx_tvalid = 1'b1;
                s_axis_eth_rx_tlast  = (i == 95) ? 1'b1 : 1'b0;
                if (i == 0) begin
                    ptp_rxstamp_ns    = rxstamp;
                    ptp_rxstamp_valid = 1'b1;
                end else begin
                    ptp_rxstamp_valid = 1'b0;
                end
                @(posedge eth_rx_clk);
            end
            @(negedge eth_rx_clk);
            s_axis_eth_rx_tdata  = 8'd0;
            s_axis_eth_rx_tvalid = 1'b0;
            s_axis_eth_rx_tlast  = 1'b0;
            ptp_rxstamp_valid    = 1'b0;
            repeat (20) @(posedge eth_rx_clk);
        end
    endtask

    task configure_frontend;
        begin
            axi_write32(R_CTRL, 32'h0000_0002); // soft reset pulse, enable=0
            axi_write32(R_CTRL, 32'h0000_0004); // clear status pulse, enable=0
            axi_write32(R_MODE, 32'd0);         // internal sawtooth source
            axi_write32(R_SAMPLE_PERIOD, 32'd2);
            axi_write32(R_SAMPLES_PER_PKT, 32'd8);
            axi_write32(R_CHANNEL_ID, 32'd1);
        end
    endtask

    task configure_eth;
        begin
            axi_write32(R_LOCAL_MAC_L, DUT_MAC[31:0]);
            axi_write32(R_LOCAL_MAC_H, {16'd0, DUT_MAC[47:32]});
            axi_write32(R_DST_MAC_L,   OBS_MAC[31:0]);
            axi_write32(R_DST_MAC_H,   {16'd0, OBS_MAC[47:32]});
            axi_write32(R_LOCAL_IP,    DUT_IP);
            axi_write32(R_DST_IP,      OBS_IP);
            axi_write32(R_UDP_PORTS,   {16'd1234, 16'd6000});
            axi_write32(R_IPV4_CFG,    {16'h1357, 8'h00, 8'd64});

            // bit0 backend_enable, bit1 commit pulse, bit2 tx_mirror_enable
            axi_write32(R_ETH_CTRL,    32'h0000_0007);
            wait_reg_bit_set(R_ETH_STATUS, 4, 128); // cfg_active
            $display("[%0t] ETH active config committed", $time);
        end
    endtask

    task configure_ptp;
        begin
            // PTP shadow fields
            axi_write32(R_PTP_CFG,        32'h0000_000D); // enable + two_step + event priority, domain 0
            axi_write32(R_PTP_CLOCK_ID_L, DUT_CLOCK_ID[31:0]);
            axi_write32(R_PTP_CLOCK_ID_H, DUT_CLOCK_ID[63:32]);
            axi_write32(R_PTP_PORT_NUM,   {16'd0, DUT_PORT_NUM});
            axi_write32(R_PTP_TX_CFG,     32'h0000_0000);

            // Seed ToD to 1 second, then pulse set_time.
            axi_write32(R_PTP_TOD_SET_L,  32'h3B9A_CA00);
            axi_write32(R_PTP_TOD_SET_H,  32'h0000_0000);
            axi_write32(R_PTP_FREQ_ADJ,   32'h0000_0000);
            axi_write32(R_PTP_CTRL,       32'h0000_0009); // tod_enable + set_time pulse
            wait_reg_bit_set(R_PTP_STATUS, 4, 256);      // cur_time_valid
            $display("[%0t] PTP ToD initialized", $time);
        end
    endtask

    task trigger_delay_req;
        input [15:0] seq_id;
        begin
            // bit0 tod_enable, bit6 tx_trigger, seq_id at [31:16]
            axi_write32(R_PTP_CTRL, {seq_id, 16'h0041});
            $display("[%0t] Trigger DUT Delay_Req seq=%0d", $time, seq_id);
        end
    endtask

    task run_ptp_exchange;
        input [15:0] sync_seq;
        input [15:0] delay_req_seq;
        reg [63:0] t1_ns;
        reg [63:0] t2_ns;
        reg [63:0] t3_ns;
        reg [63:0] t4_ns;
        begin
            t1_ns = 64'd2_000_000_000 + {48'd0, sync_seq} * 64'd1_000_000;
            t2_ns = t1_ns + 64'd4000;
            t4_ns = t1_ns + 64'd2_000_000 + 64'd36000;

            send_ptp_frame(MSG_SYNC, sync_seq, 80'd0, 80'd0, t2_ns);
            wait_mbox_and_pop(MSG_SYNC, sync_seq);

            send_ptp_frame(MSG_FOLLOW_UP, sync_seq, ns_to_ptp80(t1_ns), 80'd0, 64'd0);
            wait_mbox_and_pop(MSG_FOLLOW_UP, sync_seq);

            trigger_delay_req(delay_req_seq);
            wait_txstamp_and_consume(t3_ns);

            // t4 must be an Observer timestamp stored in Delay_Resp messageTimestamp.
            if (t4_ns <= t3_ns)
                t4_ns = t3_ns + 64'd32000;
            send_ptp_frame(MSG_DELAY_RESP, delay_req_seq, ns_to_ptp80(t4_ns), DUT_PORT_ID_80, 64'd0);
            wait_mbox_and_pop(MSG_DELAY_RESP, delay_req_seq);
        end
    endtask

    // -------------------------------------------------------------------------
    // Main stimulus
    // -------------------------------------------------------------------------
    reg [31:0] rd;
    reg [63:0] cur_time;

    initial begin
        $timeformat(-9, 3, " ns", 12);
        $dumpfile("tb_net_acq_axi_slave.vcd");
        $dumpvars(0, tb_net_acq_axi_slave);

        // Defaults
        aresetn               = 1'b0;
        eth_tx_resetn         = 1'b0;
        eth_rx_resetn         = 1'b0;
        s_awid                = 5'd0;
        s_awaddr              = 32'd0;
        s_awlen               = 8'd0;
        s_awsize              = 3'b010;
        s_awburst             = 2'b01;
        s_awlock              = 1'b0;
        s_awcache             = 4'd0;
        s_awprot              = 3'd0;
        s_awvalid             = 1'b0;
        s_wid                 = 5'd0;
        s_wdata               = 32'd0;
        s_wstrb               = 4'hF;
        s_wlast               = 1'b0;
        s_wvalid              = 1'b0;
        s_bready              = 1'b1;
        s_arid                = 5'd0;
        s_araddr              = 32'd0;
        s_arlen               = 8'd0;
        s_arsize              = 3'b010;
        s_arburst             = 2'b01;
        s_arlock              = 1'b0;
        s_arcache             = 4'd0;
        s_arprot              = 3'd0;
        s_arvalid             = 1'b0;
        s_rready              = 1'b1;

        sample_data_in        = 16'h0000;
        sample_valid_in       = 1'b0;
        pkt_ready             = 1'b1;
        m_axis_eth_tx_tready  = 1'b1;
        s_axis_eth_rx_tdata   = 8'h00;
        s_axis_eth_rx_tvalid  = 1'b0;
        s_axis_eth_rx_tlast   = 1'b0;
        ptp_rxstamp_ns        = 64'd0;
        ptp_rxstamp_valid     = 1'b0;
        tod_time_ns           = 64'd0;
        tod_time_valid        = 1'b0;
        mac_link_up           = 1'b1;
        phy_status_word       = 32'h0000_0070;
        phy_reg_snapshot      = 128'h2000_7809_01E1_C915_0000_0000_0000_0000;

        repeat (20) @(posedge aclk);
        aresetn       = 1'b1;
        eth_tx_resetn = 1'b1;
        eth_rx_resetn = 1'b1;
        repeat (20) @(posedge aclk);

        $display("[%0t] === Configure frontend / ETH / PTP ===", $time);
        configure_frontend();
        configure_eth();
        configure_ptp();

        axi_read32(R_PTP_CUR_TIME_L, rd);
        cur_time[31:0] = rd;
        axi_read32(R_PTP_CUR_TIME_H, rd);
        cur_time[63:32] = rd;
        $display("[%0t] Current sys ToD = 0x%016h", $time, cur_time);

        $display("[%0t] === PTP exchange before data stream ===", $time);
        run_ptp_exchange(16'd100, 16'd1);

        $display("[%0t] === Enable acquisition frontend ===", $time);
        axi_write32(R_CTRL, 32'h0000_0001); // enable internal data_gen, src_sel=0
        wait_reg_nonzero(R_PACKET_COUNT, 256); // wait until at least one packet completed
        repeat (300) @(posedge aclk);

        axi_read32(R_PACKET_COUNT, rd);
        $display("[%0t] PACKET_COUNT=%0d", $time, rd);
        axi_read32(R_DBG_GOOD_PKT_CNT, rd);
        $display("[%0t] DBG_GOOD_PKT_CNT=%0d", $time, rd);
        axi_read32(R_DBG_BAD_PKT_CNT, rd);
        $display("[%0t] DBG_BAD_PKT_CNT=%0d", $time, rd);
        axi_read32(R_DBG_LAST_COUNTS, rd);
        $display("[%0t] DBG_LAST_COUNTS payload=%0d sample=%0d", $time, rd[31:16], rd[15:0]);
        axi_read32(R_ETH_LAST_LENGTHS, rd);
        $display("[%0t] ETH_LAST_LENGTHS ip_total=%0d udp=%0d", $time, rd[31:16], rd[15:0]);

        $display("[%0t] === PTP exchange while acquisition is running ===", $time);
        run_ptp_exchange(16'd101, 16'd2);

        repeat (500) @(posedge aclk);
        axi_read32(R_DBGEXT_ARB_DATA_CNT, rd);
        $display("[%0t] DBGEXT data frame count=%0d", $time, rd);
        axi_read32(R_DBGEXT_ARB_PTP_CNT, rd);
        $display("[%0t] DBGEXT PTP frame count=%0d", $time, rd);
        axi_read32(R_DBGEXT_PTP_RX_ACC_CNT, rd);
        $display("[%0t] DBGEXT PTP RX accept count=%0d", $time, rd);
        axi_read32(R_DBGEXT_PTP_RX_DROP_CNT, rd);
        $display("[%0t] DBGEXT PTP RX drop count=%0d", $time, rd);

        $display("[%0t] === Simulation finished successfully ===", $time);
        $finish;
    end

endmodule

`default_nettype wire
