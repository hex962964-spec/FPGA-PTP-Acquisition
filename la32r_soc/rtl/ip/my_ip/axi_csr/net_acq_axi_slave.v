`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_axi_slave
// Description:
//   AXI register slave wrapper for net_acq_wrapper.
//
// Fixes inherited from previous revision:
//   1) Wait for local CSR `csr_ready` before returning AXI B/R response.
//   2) Keep local CSR request combinationally asserted for a full cycle so the
//      downstream CSR block can sample it correctly on the shared clock edge.
//   3) Avoid accepting a new read while a write is partially captured, and vice
//      versa, to keep the wrapper strictly single-transaction.
//
// CDC extension in this revision:
//   4) 新增 eth_tx_clk / eth_tx_resetn 端口，并继续传入 net_acq_wrapper。
//      这样 AXI/CSR 仍停留在 aclk 域，而真实以太网后端可运行在 125MHz 域。
//
// Notes:
//   - Local CSR offset width is widened to 12 bits so the internal
//     register map can extend beyond 0xFF (for example 0x100+ PTP RX mailbox).
//   - Still intended for single-beat register accesses.
//   - This revision hardens the register-model contract: illegal/non-32-bit/
//     burst-like accesses return SLVERR and do not reach the local CSR block.
//   - pkt_* 保持在 aclk/sys_clk 域；m_axis_eth_tx_* 输出在 eth_tx_clk 域。
//////////////////////////////////////////////////////////////////////////////////

module net_acq_axi_slave(
    // ------------------------------------------------------------------------
    // AXI clock / reset
    // ------------------------------------------------------------------------
    input wire               aclk,               // AXI接口工作时钟，所有 AXI/CSR 时序逻辑都在该时钟上升沿触发
    input wire               aresetn,            // AXI低有效复位信号，0=复位，1=正常工作

    // ------------------------------------------------------------------------
    // Ethernet clock / reset domains (for backend / minimal RX path)
    // ------------------------------------------------------------------------
    input wire               eth_tx_clk,         // 以太网发送域时钟，建议 125MHz
    input wire               eth_tx_resetn,      // 以太网发送域低有效复位
    input wire               eth_rx_clk,         // 以太网接收域时钟（来自 PHY）
    input wire               eth_rx_resetn,      // 以太网接收域低有效复位

    // ------------------------------------------------------------------------
    // AXI write address channel
    // ------------------------------------------------------------------------
    input wire       [4:0]   s_awid,
    input wire       [31:0]  s_awaddr,
    input wire       [7:0]   s_awlen,
    input wire       [2:0]   s_awsize,
    input wire       [1:0]   s_awburst,
    input wire               s_awlock,
    input wire       [3:0]   s_awcache,
    input wire       [2:0]   s_awprot,
    input wire               s_awvalid,
    output wire              s_awready,

    // ------------------------------------------------------------------------
    // AXI write data channel
    // ------------------------------------------------------------------------
    input wire       [4:0]   s_wid,
    input wire       [31:0]  s_wdata,
    input wire       [3:0]   s_wstrb,
    input wire               s_wlast,
    input wire               s_wvalid,
    output wire              s_wready,

    // ------------------------------------------------------------------------
    // AXI write response channel
    // ------------------------------------------------------------------------
    input wire               s_bready,
    output reg [4:0]   s_bid,
    output reg [1:0]   s_bresp,
    output reg         s_bvalid,

    // ------------------------------------------------------------------------
    // AXI read address channel
    // ------------------------------------------------------------------------
    input wire       [4:0]   s_arid,
    input wire       [31:0]  s_araddr,
    input wire       [7:0]   s_arlen,
    input wire       [2:0]   s_arsize,
    input wire       [1:0]   s_arburst,
    input wire               s_arlock,
    input wire       [3:0]   s_arcache,
    input wire       [2:0]   s_arprot,
    input wire               s_arvalid,
    output wire              s_arready,

    // ------------------------------------------------------------------------
    // AXI read data channel
    // ------------------------------------------------------------------------
    input wire               s_rready,
    output reg [4:0]   s_rid,
    output reg [31:0]  s_rdata,
    output reg [1:0]   s_rresp,
    output reg         s_rlast,
    output reg         s_rvalid,

    // ------------------------------------------------------------------------
    // Optional external sample input side (aclk/sys_clk domain)
    // ------------------------------------------------------------------------
    input wire       [15:0]  sample_data_in,
    input wire               sample_valid_in,
    output wire              sample_ready_out,

    // ------------------------------------------------------------------------
    // Legacy packet stream mirror / bypass output side (aclk/sys_clk domain)
    // ------------------------------------------------------------------------
    output wire      [15:0]  pkt_data,
    output wire              pkt_valid,
    output wire              pkt_last,
    input wire               pkt_ready,

    // ------------------------------------------------------------------------
    // Board-independent MAC-side TX AXIS-like stream (eth_tx_clk domain)
    // ------------------------------------------------------------------------
    output wire      [7:0]   m_axis_eth_tx_tdata,
    output wire              m_axis_eth_tx_tvalid,
    output wire              m_axis_eth_tx_tlast,
    input wire               m_axis_eth_tx_tready,
    output wire      [3:0]   m_axis_eth_tx_tuser,

    // ------------------------------------------------------------------------
    // Exported PTP ToD (eth_tx_clk domain) for top_phy TX timestamping
    // ------------------------------------------------------------------------
    output wire      [63:0]  ptp_tod_time_ns_eth,
    output wire              ptp_tod_time_valid_eth,

    // ------------------------------------------------------------------------
    // Board-independent MAC-side RX byte stream (eth_rx_clk domain)
    // ------------------------------------------------------------------------
    input wire       [7:0]   s_axis_eth_rx_tdata,
    input wire               s_axis_eth_rx_tvalid,
    input wire               s_axis_eth_rx_tlast,
    input wire       [63:0]  ptp_rxstamp_ns,
    input wire               ptp_rxstamp_valid,
    output wire      [63:0]  ptp_tod_time_ns_rx,
    output wire              ptp_tod_time_valid_rx,

    // ------------------------------------------------------------------------
    // Reserved future PTP sideband / board status
    // ------------------------------------------------------------------------
    input wire       [63:0]  tod_time_ns,
    input wire               tod_time_valid,
    input wire       [63:0]  ptp_txstamp_ns,
    input wire               ptp_txstamp_valid,
    input wire               mac_link_up,
    input wire       [31:0]  phy_status_word,
    input wire       [127:0] phy_reg_snapshot,
    output wire              phy_soft_reset_pulse,
    output wire              phy_restart_an_pulse
);

    // ------------------------------------------------------------------------
    // Captured AXI request fields
    // ------------------------------------------------------------------------
    reg                 wr_aw_hold;
    reg         [4:0]   wr_awid_hold;
    reg         [31:0]  wr_awaddr_hold;
    reg         [7:0]   wr_awlen_hold;
    reg         [2:0]   wr_awsize_hold;

    reg                 wr_w_hold;
    reg         [31:0]  wr_wdata_hold;
    reg         [3:0]   wr_wstrb_hold;
    reg                 wr_wlast_hold;

    reg                 rd_ar_hold;
    reg         [4:0]   rd_arid_hold;
    reg         [31:0]  rd_araddr_hold;
    reg         [7:0]   rd_arlen_hold;
    reg         [2:0]   rd_arsize_hold;

    // ------------------------------------------------------------------------
    // Current local CSR request being serviced
    // ------------------------------------------------------------------------
    reg                 req_is_write;
    reg         [4:0]   req_id_hold;
    reg         [11:0]  req_addr_hold;
    reg         [31:0]  req_wdata_hold;

    // ------------------------------------------------------------------------
    // Local CSR signals to net_acq_wrapper
    // ------------------------------------------------------------------------
    wire                csr_en_w;
    wire                csr_we_w;
    wire        [11:0]  csr_addr_w;
    wire        [31:0]  csr_wdata_w;
    wire        [31:0]  csr_rdata_w;
    wire                csr_ready_w;

    // ------------------------------------------------------------------------
    // FSM states
    // ------------------------------------------------------------------------
    localparam ST_IDLE      = 2'd0;
    localparam ST_CSR_REQ   = 2'd1;
    localparam ST_CSR_WAIT  = 2'd2;
    localparam ST_RESP      = 2'd3;

    localparam [1:0] AXI_RESP_OKAY   = 2'b00;
    localparam [1:0] AXI_RESP_SLVERR = 2'b10;

    reg [1:0] state;

    wire wr_req_legal_w;
    wire rd_req_legal_w;

    // ------------------------------------------------------------------------
    // Only allow one in-flight transaction.
    // ------------------------------------------------------------------------
    assign s_awready = (state == ST_IDLE) && !wr_aw_hold && !rd_ar_hold;
    assign s_wready  = (state == ST_IDLE) && !wr_w_hold  && !rd_ar_hold;
    assign s_arready = (state == ST_IDLE) && !rd_ar_hold && !wr_aw_hold && !wr_w_hold;

    // ------------------------------------------------------------------------
    // Drive the local CSR request combinationally while in ST_CSR_REQ.
    // ------------------------------------------------------------------------
    assign csr_en_w    = (state == ST_CSR_REQ);
    assign csr_we_w    = req_is_write;
    assign csr_addr_w  = req_addr_hold;
    assign csr_wdata_w = req_wdata_hold;

    assign wr_req_legal_w = (wr_awlen_hold == 8'd0) &&
                            (wr_awsize_hold == 3'b010) &&
                            (wr_awaddr_hold[1:0] == 2'b00) &&
                            (wr_wstrb_hold == 4'hF) &&
                            wr_wlast_hold;

    assign rd_req_legal_w = (rd_arlen_hold == 8'd0) &&
                            (rd_arsize_hold == 3'b010) &&
                            (rd_araddr_hold[1:0] == 2'b00);

    // ------------------------------------------------------------------------
    // Main sequential logic
    // ------------------------------------------------------------------------
    always @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            wr_aw_hold     <= 1'b0;
            wr_awid_hold   <= 5'd0;
            wr_awaddr_hold <= 32'd0;
            wr_awlen_hold  <= 8'd0;
            wr_awsize_hold <= 3'd0;
            wr_w_hold      <= 1'b0;
            wr_wdata_hold  <= 32'd0;
            wr_wstrb_hold  <= 4'd0;
            wr_wlast_hold  <= 1'b0;
            rd_ar_hold     <= 1'b0;
            rd_arid_hold   <= 5'd0;
            rd_araddr_hold <= 32'd0;
            rd_arlen_hold  <= 8'd0;
            rd_arsize_hold <= 3'd0;

            req_is_write   <= 1'b0;
            req_id_hold    <= 5'd0;
            req_addr_hold  <= 12'd0;
            req_wdata_hold <= 32'd0;

            state          <= ST_IDLE;

            s_bid          <= 5'd0;
            s_bresp        <= 2'b00;
            s_bvalid       <= 1'b0;
            s_rid          <= 5'd0;
            s_rdata        <= 32'd0;
            s_rresp        <= 2'b00;
            s_rlast        <= 1'b0;
            s_rvalid       <= 1'b0;
        end else begin
            // ------------------------------------------------------------
            // Capture AXI request channels
            // ------------------------------------------------------------
            if (s_awready && s_awvalid) begin
                wr_aw_hold     <= 1'b1;
                wr_awid_hold   <= s_awid;
                wr_awaddr_hold <= s_awaddr;
                wr_awlen_hold  <= s_awlen;
                wr_awsize_hold <= s_awsize;
            end

            if (s_wready && s_wvalid) begin
                wr_w_hold      <= 1'b1;
                wr_wdata_hold  <= s_wdata;
                wr_wstrb_hold  <= s_wstrb;
                wr_wlast_hold  <= s_wlast;
            end

            if (s_arready && s_arvalid) begin
                rd_ar_hold      <= 1'b1;
                rd_arid_hold    <= s_arid;
                rd_araddr_hold  <= s_araddr;
                rd_arlen_hold   <= s_arlen;
                rd_arsize_hold  <= s_arsize;
            end

            // ------------------------------------------------------------
            // Retire AXI responses
            // ------------------------------------------------------------
            if (s_bvalid && s_bready)
                s_bvalid <= 1'b0;

            if (s_rvalid && s_rready) begin
                s_rvalid <= 1'b0;
                s_rlast  <= 1'b0;
            end

            // ------------------------------------------------------------
            // FSM
            // ------------------------------------------------------------
            case (state)
                ST_IDLE: begin
                    if (wr_aw_hold && wr_w_hold && !s_bvalid && !s_rvalid) begin
                        req_is_write   <= 1'b1;
                        req_id_hold    <= wr_awid_hold;

                        wr_aw_hold     <= 1'b0;
                        wr_w_hold      <= 1'b0;

                        if (!wr_req_legal_w) begin
                            s_bid    <= wr_awid_hold;
                            s_bresp  <= AXI_RESP_SLVERR;
                            s_bvalid <= 1'b1;
                            state    <= ST_RESP;
                        end else begin
                            req_addr_hold  <= wr_awaddr_hold[11:0];
                            req_wdata_hold <= wr_wdata_hold;
                            state          <= ST_CSR_REQ;
                        end
                    end else if (rd_ar_hold && !s_bvalid && !s_rvalid) begin
                        req_is_write   <= 1'b0;
                        req_id_hold    <= rd_arid_hold;

                        rd_ar_hold     <= 1'b0;

                        if (!rd_req_legal_w) begin
                            s_rid    <= rd_arid_hold;
                            s_rdata  <= 32'd0;
                            s_rresp  <= AXI_RESP_SLVERR;
                            s_rlast  <= 1'b1;
                            s_rvalid <= 1'b1;
                            state    <= ST_RESP;
                        end else begin
                            req_addr_hold  <= rd_araddr_hold[11:0];
                            req_wdata_hold <= 32'd0;
                            state          <= ST_CSR_REQ;
                        end
                    end
                end

                ST_CSR_REQ: begin
                    state <= ST_CSR_WAIT;
                end

                ST_CSR_WAIT: begin
                    if (csr_ready_w) begin
                        if (req_is_write) begin
                            s_bid    <= req_id_hold;
                            s_bresp  <= AXI_RESP_OKAY;
                            s_bvalid <= 1'b1;
                        end else begin
                            s_rid    <= req_id_hold;
                            s_rdata  <= csr_rdata_w;
                            s_rresp  <= AXI_RESP_OKAY;
                            s_rlast  <= 1'b1;
                            s_rvalid <= 1'b1;
                        end

                        state <= ST_RESP;
                    end
                end

                ST_RESP: begin
                    if (req_is_write) begin
                        if (!s_bvalid)
                            state <= ST_IDLE;
                    end else begin
                        if (!s_rvalid)
                            state <= ST_IDLE;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Local wrapper instance
    // ------------------------------------------------------------------------
    net_acq_wrapper u_net_acq_wrapper (
        .clk_in               (aclk),
        .resetn               (aresetn),
        .eth_tx_clk           (eth_tx_clk),
        .eth_tx_resetn        (eth_tx_resetn),
        .eth_rx_clk           (eth_rx_clk),
        .eth_rx_resetn        (eth_rx_resetn),

        .csr_en               (csr_en_w),
        .csr_we               (csr_we_w),
        .csr_addr             (csr_addr_w),
        .csr_wdata            (csr_wdata_w),
        .csr_rdata            (csr_rdata_w),
        .csr_ready            (csr_ready_w),

        .sample_data_in       (sample_data_in),
        .sample_valid_in      (sample_valid_in),
        .sample_ready_out     (sample_ready_out),

        .pkt_data             (pkt_data),
        .pkt_valid            (pkt_valid),
        .pkt_last             (pkt_last),
        .pkt_ready            (pkt_ready),

        .m_axis_eth_tx_tdata  (m_axis_eth_tx_tdata),
        .m_axis_eth_tx_tvalid (m_axis_eth_tx_tvalid),
        .m_axis_eth_tx_tlast  (m_axis_eth_tx_tlast),
        .m_axis_eth_tx_tready (m_axis_eth_tx_tready),
        .m_axis_eth_tx_tuser  (m_axis_eth_tx_tuser),
        .ptp_tod_time_ns_eth  (ptp_tod_time_ns_eth),
        .ptp_tod_time_valid_eth(ptp_tod_time_valid_eth),

        .s_axis_eth_rx_tdata  (s_axis_eth_rx_tdata),
        .s_axis_eth_rx_tvalid (s_axis_eth_rx_tvalid),
        .s_axis_eth_rx_tlast  (s_axis_eth_rx_tlast),
        .ptp_rxstamp_ns       (ptp_rxstamp_ns),
        .ptp_rxstamp_valid    (ptp_rxstamp_valid),
        .ptp_tod_time_ns_rx   (ptp_tod_time_ns_rx),
        .ptp_tod_time_valid_rx(ptp_tod_time_valid_rx),

        .tod_time_ns          (tod_time_ns),
        .tod_time_valid       (tod_time_valid),
        .ptp_txstamp_ns       (ptp_txstamp_ns),
        .ptp_txstamp_valid    (ptp_txstamp_valid),
        .mac_link_up          (mac_link_up),
        .phy_status_word      (phy_status_word),
        .phy_reg_snapshot     (phy_reg_snapshot),
        .phy_soft_reset_pulse (phy_soft_reset_pulse),
        .phy_restart_an_pulse (phy_restart_an_pulse)
    );

endmodule

`default_nettype wire
