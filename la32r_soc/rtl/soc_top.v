/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
Copyright (c) 2016, Loongson Technology Corporation Limited.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of Loongson Technology Corporation Limited nor the names of 
its contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL LOONGSON TECHNOLOGY CORPORATION LIMITED BE LIABLE
TO ANY PARTY FOR DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/
//1f00_0000 apb
//1f10_0000 dvi
//1f20_0000 confreg
//1f30_0000 dma

`include "config.h"

module soc_top #(parameter SIMULATION=1'b0, parameter RAM_INIT_FILE = `SRAM_Init_File)
(
    input           clk,
    input           reset_n,

    // 图像/按键/拨码/LED/数码管等外部接口在当前版本中全部封口，
    // 仅保留时钟、复位与串口接口。若后续需要恢复，对应端口定义可直接取消注释。
    // output [2:0]    video_red,          //红色像素，3位
    // output [2:0]    video_green,        //绿色像素，3位
    // output [1:0]    video_blue,         //蓝色像素，2位
    // output          video_hsync,        //行同步（水平同步）信号
    // output          video_vsync,        //场同步（水平同步）信号
    // output          video_clk,          //像素时钟输出
    // output          video_de,           //行数据有效信号，用于区分消隐区

    // input  [3:0]    touch_btn,          //BTN1~BTN4，按钮开关，按下时为1
    // input  [31:0]   dip_sw,             //32位拨码开关，拨到“ON”时为1
    // output [15:0]   leds,               //16位LED，输出时1点亮
    // output [7:0]    dpy0,               //数码管低位信号，包括小数点，输出1点亮
    // output [7:0]    dpy1,               //数码管高位信号，包括小数点，输出1点亮


    // Wildfire MINI six-digit common-anode display: DIG1 is the PTP lock indicator.
    output [7:0]    seg_cathode,         // {DP,G,F,E,D,C,B,A}, active low
    output [5:0]    seg_anode,           // {DIG6,...,DIG1}, active high

    //------uart-------
    inout           UART_RX,            //串口RX接收
    inout           UART_TX,            //串口TX发送

    //------RGMII PHY (TX currently used, RX/MDIO reserved)-------
    output          rgmii_txc,
    output [3:0]    rgmii_txd,
    output          rgmii_tx_ctl,
    input           rgmii_rxc,
    input  [3:0]    rgmii_rxd,
    input           rgmii_rx_ctl,
    output          phy_mdc,
    inout           phy_mdio,
    output          phy_rst_n

    //------net_acq external sample source-------
    // input  [15:0]   net_acq_sample_data_in,
    // input           net_acq_sample_valid_in,
    // output          net_acq_sample_ready_out
);

    wire        reset = ~reset_n;
    // ------------------------------------------------------------------------
    // 板级外部接口已从顶层 port list 注释掉；为尽量不改动其余逻辑，以下保留
    // 同名内部连线，供 confreg / DVI 等模块继续使用。
    // ------------------------------------------------------------------------
    wire [2:0]  video_red;
    wire [2:0]  video_green;
    wire [1:0]  video_blue;
    wire        video_hsync;
    wire        video_vsync;
    wire        video_clk;
    wire        video_de;

    wire [3:0]  touch_btn;
    wire [31:0] dip_sw;
    wire [15:0] leds;
    wire [7:0]  dpy0;
    wire [7:0]  dpy1;
    wire [3:0]  digital_value;

    assign touch_btn = 4'b0;
    assign dip_sw    = 32'b0;

    // wire        rgmii_rxc;
    // wire [3:0]  rgmii_rxd;
    // wire        rgmii_rx_ctl;
    // wire        phy_mdc;
    // wire        phy_mdio;
    // wire        phy_rst_n;
    // wire        rgmii_txc;
    // wire [3:0]  rgmii_txd;
    // wire        rgmii_tx_ctl;

    wire cpu_clk;
    wire cpu_resetn;
    wire sys_clk;
    wire sys_resetn;
    wire eth_tx_clk;
    wire eth_tx_clk_90;
    wire eth_tx_resetn;
    wire eth_rx_clk;
    wire eth_rx_resetn;
    wire pll_locked;
    wire eth_rx_pll_locked;
    // Assert all domains directly from the board reset / PLL lock. Each domain
    // releases independently on its own clock; do not cascade a sys-clock
    // synchronizer output into another domain's asynchronous reset input.
    wire reset_release_n = reset_n && (SIMULATION ? 1'b1 : pll_locked);

    // 旧版调试时曾预留把 RX 口封住、用内部时钟顶住 RX 域的做法。
    // 现在已经改为：真实 rgmii_rxc -> rgmii_rx_pll(90°) -> eth_rx_clk。
    // assign rgmii_rxc    = eth_tx_clk;
    // assign rgmii_rxd    = 4'b0;
    // assign rgmii_rx_ctl = 1'b0;

    // 默认不接外部采样源时，valid 应拉低；否则若软件误切到外部源模式，会持续采到 0。
    wire  [15:0]   net_acq_sample_data_in  = 16'b0;
    wire           net_acq_sample_valid_in = 1'b0;
    wire           net_acq_sample_ready_out;

    // legacy pkt_* mirror，保留下来便于 ILA/波形观察。
    wire  [15:0]   net_acq_pkt_data;
    wire           net_acq_pkt_valid;
    wire           net_acq_pkt_last;
    wire           net_acq_pkt_ready;

    // net_acq 新增的 MAC-side TX 输出，先在 soc_top 内部引出，便于仿真/ILA 观察。
    wire  [7:0]    net_acq_m_axis_eth_tx_tdata;
    wire           net_acq_m_axis_eth_tx_tvalid;
    wire           net_acq_m_axis_eth_tx_tlast;
    wire           net_acq_m_axis_eth_tx_tready;
    wire  [3:0]    net_acq_m_axis_eth_tx_tuser;

    // 预留 PTP / 板级状态输入。
    // 当前 wrapper 内部已经维护 sys/eth 两份 ToD，这里保留 legacy 输入为 0，
    // 同时把 eth 域 ToD 从 net_acq_axi_slave 导出给 top_phy 做 TX 时间戳。
    wire  [63:0]   net_acq_tod_time_ns        = 64'd0;
    wire           net_acq_tod_time_valid     = 1'b0;
    wire  [63:0]   net_acq_ptp_tod_time_ns;
    wire           net_acq_ptp_tod_time_valid;
    wire  [63:0]   net_acq_ptp_txstamp_ns;
    wire           net_acq_ptp_txstamp_valid;
    wire  [63:0]   net_acq_ptp_rxstamp_ns;
    wire           net_acq_ptp_rxstamp_valid;
    wire  [63:0]   net_acq_ptp_tod_time_ns_rx;
    wire           net_acq_ptp_tod_time_valid_rx;
    wire  [7:0]    net_acq_m_axis_eth_rx_tdata;
    wire           net_acq_m_axis_eth_rx_tvalid;
    wire           net_acq_m_axis_eth_rx_tlast;
    wire           net_acq_mac_link_up;
    wire  [31:0]   net_acq_phy_status_word;
    wire  [127:0]  net_acq_phy_reg_snapshot;
    wire           net_acq_phy_soft_reset_pulse;
    wire           net_acq_phy_restart_an_pulse;

    // legacy pkt_* 观察口当前仍保持始终 ready。
    assign net_acq_pkt_ready = 1'b1;

generate if(SIMULATION) begin: sim_clk
    //simulation clk.
    reg clk_sim;
    initial begin
        clk_sim = 1'b0;
    end
    always #15 clk_sim = ~clk_sim;

    assign cpu_clk = clk_sim;
    assign sys_clk = clk;
    rst_sync u_rst_sys(
        .clk(sys_clk),
        .rst_n_in(reset_release_n),
        .rst_n_out(sys_resetn)
    );
    rst_sync u_rst_cpu(
        .clk(cpu_clk),
        .rst_n_in(reset_release_n),
        .rst_n_out(cpu_resetn)
    );

    // 仿真阶段先让 eth 发送域与 sys_clk 同域，便于直接跑通后端。
    assign eth_tx_clk       = sys_clk;
    assign eth_tx_clk_90    = sys_clk;
    assign eth_tx_resetn    = sys_resetn;

    // 仿真时旁路 RX MMCM，直接使用 testbench 驱动的 rgmii_rxc。
    assign eth_rx_clk       = rgmii_rxc;
    assign eth_rx_pll_locked = 1'b1;
end
else begin: pll_clk
    clk_pll u_clk_pll(
        .cpu_clk    (cpu_clk),
        .sys_clk    (sys_clk),
        .eth_clk    (eth_tx_clk),
        .eth_clk_90 (eth_tx_clk_90),
        .resetn     (~reset),
        .locked     (pll_locked),
        .clk_in1    (clk)
    );
    rst_sync u_rst_sys(
        .clk(sys_clk),
        .rst_n_in(reset_release_n),
        .rst_n_out(sys_resetn)
    );
    rst_sync u_rst_cpu(
        .clk(cpu_clk),
        .rst_n_in(reset_release_n),
        .rst_n_out(cpu_resetn)
    );
    rst_sync u_rst_eth(
        .clk(eth_tx_clk),
        .rst_n_in(reset_release_n),
        .rst_n_out(eth_tx_resetn)
    );

    // RGMII RX sampling clock:
    //   clk_in1  = external PHY recovered 125 MHz RX clock
    //   clk_out1 = same 125 MHz clock shifted by +90° (~2 ns @ 1 Gbps)
    rgmii_rx_pll u_rgmii_rx_pll (
        .clk_out1 (eth_rx_clk),
        .reset    (reset),
        .locked   (eth_rx_pll_locked),
        .clk_in1  (rgmii_rxc)
    );
end
endgenerate

// RX helper domain follows the phase-shifted recovered RX clock so that
// rgmii_rx_if / top_phy_axis_mac_rx / wrapper PTP RX logic stay in one domain.
rst_sync u_rst_eth_rx(
    .clk       (eth_rx_clk),
    .rst_n_in  (reset_release_n & eth_rx_pll_locked),
    .rst_n_out (eth_rx_resetn)
);

//debug signals
wire [31:0] debug_wb_pc;
wire [31:0] debug_wb_inst;
wire [3 :0] debug_wb_rf_wen;
wire [4 :0] debug_wb_rf_wnum;
wire [31:0] debug_wb_rf_wdata;

//cpu axi
wire [3 :0] cpu_arid   ;
wire [31:0] cpu_araddr ;
wire [7 :0] cpu_arlen  ;
wire [2 :0] cpu_arsize ;
wire [1 :0] cpu_arburst;
wire [1 :0] cpu_arlock ;
wire [3 :0] cpu_arcache;
wire [2 :0] cpu_arprot ;
wire        cpu_arvalid;
wire        cpu_arready;
wire [3 :0] cpu_rid    ;
wire [31:0] cpu_rdata  ;
wire [1 :0] cpu_rresp  ;
wire        cpu_rlast  ;
wire        cpu_rvalid ;
wire        cpu_rready ;
wire [3 :0] cpu_awid   ;
wire [31:0] cpu_awaddr ;
wire [7 :0] cpu_awlen  ;
wire [2 :0] cpu_awsize ;
wire [1 :0] cpu_awburst;
wire [1 :0] cpu_awlock ;
wire [3 :0] cpu_awcache;
wire [2 :0] cpu_awprot ;
wire        cpu_awvalid;
wire        cpu_awready;
wire [3 :0] cpu_wid    ;
wire [31:0] cpu_wdata  ;
wire [3 :0] cpu_wstrb  ;
wire        cpu_wlast  ;
wire        cpu_wvalid ;
wire        cpu_wready ;
wire [3 :0] cpu_bid    ;
wire [1 :0] cpu_bresp  ;
wire        cpu_bvalid ;
wire        cpu_bready ;
wire        cpu_sync_awid_4 ;
wire        cpu_bid_4  ;
wire        cpu_sync_arid_4 ;
wire        cpu_rid_4  ;

//cpu axi sync
wire [3 :0] cpu_sync_arid   ;
wire [31:0] cpu_sync_araddr ;
wire [7 :0] cpu_sync_arlen  ;
wire [2 :0] cpu_sync_arsize ;
wire [1 :0] cpu_sync_arburst;
wire        cpu_sync_arlock ;
wire [3 :0] cpu_sync_arcache;
wire [2 :0] cpu_sync_arprot ;
wire        cpu_sync_arvalid;
wire        cpu_sync_arready;
wire [3 :0] cpu_sync_rid    ;
wire [31:0] cpu_sync_rdata  ;
wire [1 :0] cpu_sync_rresp  ;
wire        cpu_sync_rlast  ;
wire        cpu_sync_rvalid ;
wire        cpu_sync_rready ;
wire [3 :0] cpu_sync_awid   ;
wire [31:0] cpu_sync_awaddr ;
wire [7 :0] cpu_sync_awlen  ;
wire [2 :0] cpu_sync_awsize ;
wire [1 :0] cpu_sync_awburst;
wire        cpu_sync_awlock ;
wire [3 :0] cpu_sync_awcache;
wire [2 :0] cpu_sync_awprot ;
wire        cpu_sync_awvalid;
wire        cpu_sync_awready;
wire [3 :0] cpu_sync_wid    ;
wire [31:0] cpu_sync_wdata  ;
wire [3 :0] cpu_sync_wstrb  ;
wire        cpu_sync_wlast  ;
wire        cpu_sync_wvalid ;
wire        cpu_sync_wready ;
wire [3 :0] cpu_sync_bid    ;
wire [1 :0] cpu_sync_bresp  ;
wire        cpu_sync_bvalid ;
wire        cpu_sync_bready ;

//axi ram
wire [4 :0] ram_arid   ;
wire [31:0] ram_araddr ;
wire [7 :0] ram_arlen  ;
wire [2 :0] ram_arsize ;
wire [1 :0] ram_arburst;
wire        ram_arlock ;
wire [3 :0] ram_arcache;
wire [2 :0] ram_arprot ;
wire        ram_arvalid;
wire        ram_arready;
wire [4 :0] ram_rid    ;
wire [31:0] ram_rdata  ;
wire [1 :0] ram_rresp  ;
wire        ram_rlast  ;
wire        ram_rvalid ;
wire        ram_rready ;
wire [4 :0] ram_awid   ;
wire [31:0] ram_awaddr ;
wire [7 :0] ram_awlen  ;
wire [2 :0] ram_awsize ;
wire [1 :0] ram_awburst;
wire        ram_awlock ;
wire [3 :0] ram_awcache;
wire [2 :0] ram_awprot ;
wire        ram_awvalid;
wire        ram_awready;
wire [4 :0] ram_wid    ;
wire [31:0] ram_wdata  ;
wire [3 :0] ram_wstrb  ;
wire        ram_wlast  ;
wire        ram_wvalid ;
wire        ram_wready ;
wire [4 :0] ram_bid    ;
wire [1 :0] ram_bresp  ;
wire        ram_bvalid ;
wire        ram_bready ;

//uart axi
wire  uart_arready;
wire  [ 4:0]  uart_rid;
wire  [31:0]  uart_rdata;
wire  [ 1:0]  uart_rresp;
wire  uart_rlast;
wire  uart_rvalid;
wire  uart_awready;
wire  uart_wready;
wire  [ 4:0]  uart_bid;
wire  [ 1:0]  uart_bresp;
wire  uart_bvalid;
wire  [ 4:0]  uart_arid;
wire  [31:0]  uart_araddr;
wire  [ 7:0]  uart_arlen;
wire  [ 2:0]  uart_arsize;
wire  [ 1:0]  uart_arburst;
wire          uart_arlock;
wire  [ 3:0]  uart_arcache;
wire  [ 2:0]  uart_arprot;
wire  uart_arvalid;
wire  uart_rready;
wire  [ 4:0]  uart_awid;
wire  [31:0]  uart_awaddr;
wire  [ 7:0]  uart_awlen;
wire  [ 2:0]  uart_awsize;
wire  [ 1:0]  uart_awburst;
wire          uart_awlock;
wire  [ 3:0]  uart_awcache;
wire  [ 2:0]  uart_awprot;
wire  uart_awvalid;
wire  [ 4:0]  uart_wid;
wire  [31:0]  uart_wdata;
wire  [ 3:0]  uart_wstrb;
wire  uart_wlast;
wire  uart_wvalid;
wire  uart_bready;
wire  irq_rx;

//uart
wire UART_CTS,   UART_RTS;
wire UART_DTR,   UART_DSR;
wire UART_RI,    UART_DCD;
assign UART_CTS = 1'b0;
assign UART_DSR = 1'b0;
assign UART_DCD = 1'b0;
assign UART_RI  = 1'b0;
wire uart0_int   ;
wire uart0_txd_o ;
wire uart0_txd_i ;
wire uart0_txd_oe;
wire uart0_rxd_o ;
wire uart0_rxd_i ;
wire uart0_rxd_oe;
wire uart0_rts_o ;
wire uart0_cts_i ;
wire uart0_dsr_i ;
wire uart0_dcd_i ;
wire uart0_dtr_o ;
wire uart0_ri_i  ;
assign     UART_RX     = uart0_rxd_oe ? 1'bz : uart0_rxd_o ;
assign     UART_TX     = uart0_txd_oe ? 1'bz : uart0_txd_o ;
assign     UART_RTS    = uart0_rts_o ;
assign     UART_DTR    = uart0_dtr_o ;
assign     uart0_txd_i = UART_TX;
assign     uart0_rxd_i = UART_RX;
assign     uart0_cts_i = UART_CTS;
assign     uart0_dcd_i = UART_DCD;
assign     uart0_dsr_i = UART_DSR;
assign     uart0_ri_i  = UART_RI ;

//dma master axi
wire [3 :0] dma_m_arid   ;
wire [31:0] dma_m_araddr ;
wire [7 :0] dma_m_arlen  ;
wire [2 :0] dma_m_arsize ;
wire [1 :0] dma_m_arburst;
wire        dma_m_arlock ;
wire [3 :0] dma_m_arcache;
wire [2 :0] dma_m_arprot ;
wire        dma_m_arvalid;
wire        dma_m_arready;
wire [3 :0] dma_m_rid    ;
wire [31:0] dma_m_rdata  ;
wire [1 :0] dma_m_rresp  ;
wire        dma_m_rlast  ;
wire        dma_m_rvalid ;
wire        dma_m_rready ;
wire [3 :0] dma_m_awid   ;
wire [31:0] dma_m_awaddr ;
wire [7 :0] dma_m_awlen  ;
wire [2 :0] dma_m_awsize ;
wire [1 :0] dma_m_awburst;
wire        dma_m_awlock ;
wire [3 :0] dma_m_awcache;
wire [2 :0] dma_m_awprot ;
wire        dma_m_awvalid;
wire        dma_m_awready;
wire [3 :0] dma_m_wid    ;
wire [31:0] dma_m_wdata  ;
wire [3 :0] dma_m_wstrb  ;
wire        dma_m_wlast  ;
wire        dma_m_wvalid ;
wire        dma_m_wready ;
wire [3 :0] dma_m_bid    ;
wire [1 :0] dma_m_bresp  ;
wire        dma_m_bvalid ;
wire        dma_m_bready ;

// assign dma_m_arid       = 4'b0;
// assign dma_m_araddr     = 32'h0;
// assign dma_m_arlen      = 8'b0;
// assign dma_m_arsize     = 3'b0;
// assign dma_m_arburst    = 2'b0;
// assign dma_m_arlock     = 1'b0;
// assign dma_m_arcache    = 4'b0;
// assign dma_m_arprot     = 3'b0;
// assign dma_m_arvalid    = 1'b0;
// assign dma_m_rready     = 1'b1;
// assign dma_m_awid       = 4'b0;
// assign dma_m_awaddr     = 32'b0;
// assign dma_m_awlen      = 8'b0;
// assign dma_m_awsize     = 3'b0;
// assign dma_m_awburst    = 2'b0;
// assign dma_m_awlock     = 1'b0;
// assign dma_m_awcache    = 4'b0;
// assign dma_m_awprot     = 3'b0;
// assign dma_m_awvalid    = 1'b1;
// assign dma_m_wid        = 4'b0;
// assign dma_m_wdata      = 32'b0;
// assign dma_m_wstrb      = 4'b0;
// assign dma_m_wlast      = 1'b0;
// assign dma_m_wvalid     = 1'b0;
// assign dma_m_bready     = 1'b1;

wire [4 :0] dma_s_arid   ;
wire [31:0] dma_s_araddr ;
wire [7 :0] dma_s_arlen  ;
wire [2 :0] dma_s_arsize ;
wire [1 :0] dma_s_arburst;
wire        dma_s_arlock ;
wire [3 :0] dma_s_arcache;
wire [2 :0] dma_s_arprot ;
wire        dma_s_arvalid;
wire        dma_s_arready;
wire [4 :0] dma_s_rid    ;
wire [31:0] dma_s_rdata  ;
wire [1 :0] dma_s_rresp  ;
wire        dma_s_rlast  ;
wire        dma_s_rvalid ;
wire        dma_s_rready ;
wire [4 :0] dma_s_awid   ;
wire [31:0] dma_s_awaddr ;
wire [7 :0] dma_s_awlen  ;
wire [2 :0] dma_s_awsize ;
wire [1 :0] dma_s_awburst;
wire        dma_s_awlock ;
wire [3 :0] dma_s_awcache;
wire [2 :0] dma_s_awprot ;
wire        dma_s_awvalid;
wire        dma_s_awready;
wire [31:0] dma_s_wdata  ;
wire [3 :0] dma_s_wstrb  ;
wire        dma_s_wlast  ;
wire        dma_s_wvalid ;
wire        dma_s_wready ;
wire [4 :0] dma_s_bid    ;
wire [1 :0] dma_s_bresp  ;
wire        dma_s_bvalid ;
wire        dma_s_bready ;
wire        dma_finish   ;

// assign dma_s_arready    = 1'b1;
// assign dma_s_rid        = 5'b0;
// assign dma_s_rdata      = 32'b0;
// assign dma_s_rresp      = 2'b0;
// assign dma_s_rlast      = 1'b0;
// assign dma_s_rvalid     = 1'b0;
// assign dma_s_awready    = 1'b1;
// assign dma_s_wready     = 1'b1;
// assign dma_s_bid        = 5'b0;
// assign dma_s_bresp      = 2'b0;
// assign dma_s_bvalid     = 1'b0;

// reserved
wire [4 :0] axiOut_1_arid   ;
wire [31:0] axiOut_1_araddr ;
wire [7 :0] axiOut_1_arlen  ;
wire [2 :0] axiOut_1_arsize ;
wire [1 :0] axiOut_1_arburst;
wire        axiOut_1_arlock ;
wire [3 :0] axiOut_1_arcache;
wire [2 :0] axiOut_1_arprot ;
wire        axiOut_1_arvalid;
wire        axiOut_1_arready;
wire [4 :0] axiOut_1_rid    ;
wire [31:0] axiOut_1_rdata  ;
wire [1 :0] axiOut_1_rresp  ;
wire        axiOut_1_rlast  ;
wire        axiOut_1_rvalid ;
wire        axiOut_1_rready ;
wire [4 :0] axiOut_1_awid   ;
wire [31:0] axiOut_1_awaddr ;
wire [7 :0] axiOut_1_awlen  ;
wire [2 :0] axiOut_1_awsize ;
wire [1 :0] axiOut_1_awburst;
wire        axiOut_1_awlock ;
wire [3 :0] axiOut_1_awcache;
wire [2 :0] axiOut_1_awprot ;
wire        axiOut_1_awvalid;
wire        axiOut_1_awready;
wire [4 :0] axiOut_1_wid    ;
wire [31:0] axiOut_1_wdata  ;
wire [3 :0] axiOut_1_wstrb  ;
wire        axiOut_1_wlast  ;
wire        axiOut_1_wvalid ;
wire        axiOut_1_wready ;
wire [4 :0] axiOut_1_bid    ;
wire [1 :0] axiOut_1_bresp  ;
wire        axiOut_1_bvalid ;
wire        axiOut_1_bready ;

assign axiOut_1_arready = 1'b1;
assign axiOut_1_rid    = 5'b0;
assign axiOut_1_rdata  = 32'b0;
assign axiOut_1_rresp  = 2'b0;
assign axiOut_1_rlast  = 1'b0;
assign axiOut_1_rvalid = 1'b0;
assign axiOut_1_awready = 1'b1;
assign axiOut_1_wready = 1'b1;
assign axiOut_1_bid    = 5'b0;
assign axiOut_1_bresp = 2'b0;
assign axiOut_1_bvalid = 1'b0;


//axi dvi
wire [4 :0] dvi_arid   ;
wire [31:0] dvi_araddr ;
wire [7 :0] dvi_arlen  ;
wire [2 :0] dvi_arsize ;
wire [1 :0] dvi_arburst;
wire [1 :0] dvi_arlock ;
wire [3 :0] dvi_arcache;
wire [2 :0] dvi_arprot ;
wire        dvi_arvalid;
wire        dvi_arready;
wire [4 :0] dvi_rid    ;
wire [31:0] dvi_rdata  ;
wire [1 :0] dvi_rresp  ;
wire        dvi_rlast  ;
wire        dvi_rvalid ;
wire        dvi_rready ;
wire [4 :0] dvi_awid   ;
wire [31:0] dvi_awaddr ;
wire [7 :0] dvi_awlen  ;
wire [2 :0] dvi_awsize ;
wire [1 :0] dvi_awburst;
wire [1 :0] dvi_awlock ;
wire [3 :0] dvi_awcache;
wire [2 :0] dvi_awprot ;
wire        dvi_awvalid;
wire        dvi_awready;
wire [4 :0] dvi_wid    ;
wire [31:0] dvi_wdata  ;
wire [3 :0] dvi_wstrb  ;
wire        dvi_wlast  ;
wire        dvi_wvalid ;
wire        dvi_wready ;
wire [4 :0] dvi_bid    ;
wire [1 :0] dvi_bresp  ;
wire        dvi_bvalid ;
wire        dvi_bready ;

// assign dvi_arready  = 1'b1;
// assign dvi_rid      = 5'b0;
// assign dvi_rdata    = 32'b0;
// assign dvi_rresp    = 2'b0;
// assign dvi_rlast    = 1'b0;
// assign dvi_rvalid   = 1'b0;
// assign dvi_awready  = 1'b1;
// assign dvi_wready   = 1'b1;
// assign dvi_bid      = 5'b0;
// assign dvi_bresp    = 2'b0;
// assign dvi_bvalid   = 1'b0;

//axi confreg
wire [4 :0] confreg_arid   ;
wire [31:0] confreg_araddr ;
wire [7 :0] confreg_arlen  ;
wire [2 :0] confreg_arsize ;
wire [1 :0] confreg_arburst;
wire        confreg_arlock ;
wire [3 :0] confreg_arcache;
wire [2 :0] confreg_arprot ;
wire        confreg_arvalid;
wire        confreg_arready;
wire [4 :0] confreg_rid    ;
wire [31:0] confreg_rdata  ;
wire [1 :0] confreg_rresp  ;
wire        confreg_rlast  ;
wire        confreg_rvalid ;
wire        confreg_rready ;
wire [4 :0] confreg_awid   ;
wire [31:0] confreg_awaddr ;
wire [7 :0] confreg_awlen  ;
wire [2 :0] confreg_awsize ;
wire [1 :0] confreg_awburst;
wire        confreg_awlock ;
wire [3 :0] confreg_awcache;
wire [2 :0] confreg_awprot ;
wire        confreg_awvalid;
wire        confreg_awready;
wire [4 :0] confreg_wid    ;
wire [31:0] confreg_wdata  ;
wire [3 :0] confreg_wstrb  ;
wire        confreg_wlast  ;
wire        confreg_wvalid ;
wire        confreg_wready ;
wire [4 :0] confreg_bid    ;
wire [1 :0] confreg_bresp  ;
wire        confreg_bvalid ;
wire        confreg_bready ;

//slave 6 FFT/IFFT
wire [4 :0] fft_arid   ;
wire [31:0] fft_araddr ;
wire [7 :0] fft_arlen  ;
wire [2 :0] fft_arsize ;
wire [1 :0] fft_arburst;
wire        fft_arlock ;
wire [3 :0] fft_arcache;
wire [2 :0] fft_arprot ;
wire        fft_arvalid;
wire        fft_arready;
wire [4 :0] fft_rid    ;
wire [31:0] fft_rdata  ;
wire [1 :0] fft_rresp  ;
wire        fft_rlast  ;
wire        fft_rvalid ;
wire        fft_rready ;
wire [4 :0] fft_awid   ;
wire [31:0] fft_awaddr ;
wire [7 :0] fft_awlen  ;
wire [2 :0] fft_awsize ;
wire [1 :0] fft_awburst;
wire        fft_awlock ;
wire [3 :0] fft_awcache;
wire [2 :0] fft_awprot ;
wire        fft_awvalid;
wire        fft_awready;
wire [4 :0] fft_wid    ;
wire [31:0] fft_wdata  ;
wire [3 :0] fft_wstrb  ;
wire        fft_wlast  ;
wire        fft_wvalid ;
wire        fft_wready ;
wire [4 :0] fft_bid    ;
wire [1 :0] fft_bresp  ;
wire        fft_bvalid ;
wire        fft_bready ;
wire        fft_finish ;

// assign fft_arready = 1'b1;
// assign fft_rid    = 5'b0;
// assign fft_rdata  = 32'b0;
// assign fft_rresp  = 2'b0;
// assign fft_rlast  = 1'b0;
// assign fft_rvalid = 1'b0;
// assign fft_awready = 1'b1;
// assign fft_wready = 1'b1;
// assign fft_bid    = 5'b0;
// assign fft_bresp = 2'b0;
// assign fft_bvalid = 1'b0;

//slave 7
wire [4 :0] axiOut_7_arid   ;
wire [31:0] axiOut_7_araddr ;
wire [7 :0] axiOut_7_arlen  ;
wire [2 :0] axiOut_7_arsize ;
wire [1 :0] axiOut_7_arburst;
wire        axiOut_7_arlock ;
wire [3 :0] axiOut_7_arcache;
wire [2 :0] axiOut_7_arprot ;
wire        axiOut_7_arvalid;
wire        axiOut_7_arready;
wire [4 :0] axiOut_7_rid    ;
wire [31:0] axiOut_7_rdata  ;
wire [1 :0] axiOut_7_rresp  ;
wire        axiOut_7_rlast  ;
wire        axiOut_7_rvalid ;
wire        axiOut_7_rready ;
wire [4 :0] axiOut_7_awid   ;
wire [31:0] axiOut_7_awaddr ;
wire [7 :0] axiOut_7_awlen  ;
wire [2 :0] axiOut_7_awsize ;
wire [1 :0] axiOut_7_awburst;
wire        axiOut_7_awlock ;
wire [3 :0] axiOut_7_awcache;
wire [2 :0] axiOut_7_awprot ;
wire        axiOut_7_awvalid;
wire        axiOut_7_awready;
wire [4 :0] axiOut_7_wid    ;
wire [31:0] axiOut_7_wdata  ;
wire [3 :0] axiOut_7_wstrb  ;
wire        axiOut_7_wlast  ;
wire        axiOut_7_wvalid ;
wire        axiOut_7_wready ;
wire [4 :0] axiOut_7_bid    ;
wire [1 :0] axiOut_7_bresp  ;
wire        axiOut_7_bvalid ;
wire        axiOut_7_bready ;

// assign axiOut_7_arready = 1'b1;
// assign axiOut_7_rid    = 5'b0;
// assign axiOut_7_rdata  = 32'b0;
// assign axiOut_7_rresp  = 2'b0;
// assign axiOut_7_rlast  = 1'b0;
// assign axiOut_7_rvalid = 1'b0;
// assign axiOut_7_awready = 1'b1;
// assign axiOut_7_wready = 1'b1;
// assign axiOut_7_bid    = 5'b0;
// assign axiOut_7_bresp = 2'b0;
// assign axiOut_7_bvalid = 1'b0;

wire confreg_int;

AxiCrossbar_2x8  u_AxiCrossbar_2x8 (
    .clk                     ( sys_clk             ),
    .resetn                  ( sys_resetn          ),
    
    //master 0
    //aw
    .axiIn_0_awvalid         ( cpu_sync_awvalid    ),
    .axiIn_0_awready         ( cpu_sync_awready    ),
    .axiIn_0_awaddr          ( cpu_sync_awaddr     ),
    .axiIn_0_awid            ( cpu_sync_awid       ),
    .axiIn_0_awlen           ( cpu_sync_awlen      ),
    .axiIn_0_awsize          ( cpu_sync_awsize     ),
    .axiIn_0_awburst         ( cpu_sync_awburst    ),
    .axiIn_0_awlock          ( cpu_sync_awlock     ),
    .axiIn_0_awcache         ( cpu_sync_awcache    ),
    .axiIn_0_awprot          ( cpu_sync_awprot     ),
    //w
    .axiIn_0_wvalid          ( cpu_sync_wvalid     ),
    .axiIn_0_wready          ( cpu_sync_wready     ),
    .axiIn_0_wdata           ( cpu_sync_wdata      ),
    .axiIn_0_wstrb           ( cpu_sync_wstrb      ),
    .axiIn_0_wlast           ( cpu_sync_wlast      ),
    //b
    .axiIn_0_bready          ( cpu_sync_bready     ),
    .axiIn_0_bvalid          ( cpu_sync_bvalid     ),
    .axiIn_0_bid             ( cpu_sync_bid        ),
    .axiIn_0_bresp           ( cpu_sync_bresp      ),
    //ar
    .axiIn_0_arvalid         ( cpu_sync_arvalid    ),
    .axiIn_0_arready         ( cpu_sync_arready    ),
    .axiIn_0_araddr          ( cpu_sync_araddr     ),
    .axiIn_0_arid            ( cpu_sync_arid       ),
    .axiIn_0_arlen           ( cpu_sync_arlen      ),
    .axiIn_0_arsize          ( cpu_sync_arsize     ),
    .axiIn_0_arburst         ( cpu_sync_arburst    ),
    .axiIn_0_arlock          ( cpu_sync_arlock     ),
    .axiIn_0_arcache         ( cpu_sync_arcache    ),
    .axiIn_0_arprot          ( cpu_sync_arprot     ),
    //r
    .axiIn_0_rvalid          ( cpu_sync_rvalid     ),
    .axiIn_0_rready          ( cpu_sync_rready     ),
    .axiIn_0_rdata           ( cpu_sync_rdata      ),
    .axiIn_0_rid             ( cpu_sync_rid        ),
    .axiIn_0_rresp           ( cpu_sync_rresp      ),
    .axiIn_0_rlast           ( cpu_sync_rlast      ),

    //master 1
    //aw
    .axiIn_1_awvalid         ( dma_m_awvalid       ),
    .axiIn_1_awready         ( dma_m_awready       ),
    .axiIn_1_awaddr          ( dma_m_awaddr        ),
    .axiIn_1_awid            ( dma_m_awid          ),
    .axiIn_1_awlen           ( dma_m_awlen         ),
    .axiIn_1_awsize          ( dma_m_awsize        ),
    .axiIn_1_awburst         ( dma_m_awburst       ),
    .axiIn_1_awlock          ( dma_m_awlock        ),
    .axiIn_1_awcache         ( dma_m_awcache       ),
    .axiIn_1_awprot          ( dma_m_awprot        ),
    //w
    .axiIn_1_wvalid          ( dma_m_wvalid        ),
    .axiIn_1_wready          ( dma_m_wready        ),
    .axiIn_1_wdata           ( dma_m_wdata         ),
    .axiIn_1_wstrb           ( dma_m_wstrb         ),
    .axiIn_1_wlast           ( dma_m_wlast         ),
    //b
    .axiIn_1_bready          ( dma_m_bready        ),
    .axiIn_1_bvalid          ( dma_m_bvalid        ),
    .axiIn_1_bid             ( dma_m_bid           ),
    .axiIn_1_bresp           ( dma_m_bresp         ),
    //ar
    .axiIn_1_arvalid         ( dma_m_arvalid       ),
    .axiIn_1_arready         ( dma_m_arready       ),
    .axiIn_1_araddr          ( dma_m_araddr        ),
    .axiIn_1_arid            ( dma_m_arid          ),
    .axiIn_1_arlen           ( dma_m_arlen         ),
    .axiIn_1_arsize          ( dma_m_arsize        ),
    .axiIn_1_arburst         ( dma_m_arburst       ),
    .axiIn_1_arlock          ( dma_m_arlock        ),
    .axiIn_1_arcache         ( dma_m_arcache       ),
    .axiIn_1_arprot          ( dma_m_arprot        ),
    //r
    .axiIn_1_rvalid          ( dma_m_rvalid        ),
    .axiIn_1_rready          ( dma_m_rready        ),
    .axiIn_1_rdata           ( dma_m_rdata         ),
    .axiIn_1_rid             ( dma_m_rid           ),
    .axiIn_1_rresp           ( dma_m_rresp         ),
    .axiIn_1_rlast           ( dma_m_rlast         ),

    //slave 0
    //aw
    .axiOut_0_awvalid        ( ram_awvalid   ),
    .axiOut_0_awready        ( ram_awready   ),
    .axiOut_0_awaddr         ( ram_awaddr    ),
    .axiOut_0_awid           ( ram_awid      ),
    .axiOut_0_awlen          ( ram_awlen     ),
    .axiOut_0_awsize         ( ram_awsize    ),
    .axiOut_0_awburst        ( ram_awburst   ),
    .axiOut_0_awlock         ( ram_awlock    ),
    .axiOut_0_awcache        ( ram_awcache   ),
    .axiOut_0_awprot         ( ram_awprot    ),
    //w
    .axiOut_0_wvalid         ( ram_wvalid    ),
    .axiOut_0_wready         ( ram_wready    ),
    .axiOut_0_wdata          ( ram_wdata     ),
    .axiOut_0_wstrb          ( ram_wstrb     ),
    .axiOut_0_wlast          ( ram_wlast     ),
    //b
    .axiOut_0_bready         ( ram_bready    ),
    .axiOut_0_bvalid         ( ram_bvalid    ),
    .axiOut_0_bid            ( ram_bid       ),
    .axiOut_0_bresp          ( ram_bresp     ),
    //ar
    .axiOut_0_arvalid        ( ram_arvalid   ),
    .axiOut_0_arready        ( ram_arready   ),
    .axiOut_0_araddr         ( ram_araddr    ),
    .axiOut_0_arid           ( ram_arid      ),
    .axiOut_0_arlen          ( ram_arlen     ),
    .axiOut_0_arsize         ( ram_arsize    ),
    .axiOut_0_arburst        ( ram_arburst   ),
    .axiOut_0_arlock         ( ram_arlock    ),
    .axiOut_0_arcache        ( ram_arcache   ),
    .axiOut_0_arprot         ( ram_arprot    ),
    //r
    .axiOut_0_rvalid         ( ram_rvalid    ),
    .axiOut_0_rready         ( ram_rready    ),
    .axiOut_0_rdata          ( ram_rdata     ),
    .axiOut_0_rid            ( ram_rid       ),
    .axiOut_0_rresp          ( ram_rresp     ),
    .axiOut_0_rlast          ( ram_rlast     ),

    //slave 1
    //aw
    .axiOut_1_awvalid        ( axiOut_1_awvalid   ),
    .axiOut_1_awready        ( axiOut_1_awready   ),
    .axiOut_1_awaddr         ( axiOut_1_awaddr    ),
    .axiOut_1_awid           ( axiOut_1_awid      ),
    .axiOut_1_awlen          ( axiOut_1_awlen     ),
    .axiOut_1_awsize         ( axiOut_1_awsize    ),
    .axiOut_1_awburst        ( axiOut_1_awburst   ),
    .axiOut_1_awlock         ( axiOut_1_awlock    ),
    .axiOut_1_awcache        ( axiOut_1_awcache   ),
    .axiOut_1_awprot         ( axiOut_1_awprot    ),
    //w
    .axiOut_1_wvalid         ( axiOut_1_wvalid    ),
    .axiOut_1_wready         ( axiOut_1_wready    ),
    .axiOut_1_wdata          ( axiOut_1_wdata     ),
    .axiOut_1_wstrb          ( axiOut_1_wstrb     ),
    .axiOut_1_wlast          ( axiOut_1_wlast     ),
    //b
    .axiOut_1_bready         ( axiOut_1_bready    ),
    .axiOut_1_bvalid         ( axiOut_1_bvalid    ),
    .axiOut_1_bid            ( axiOut_1_bid       ),
    .axiOut_1_bresp          ( axiOut_1_bresp     ),
    //ar
    .axiOut_1_arvalid        ( axiOut_1_arvalid   ),
    .axiOut_1_arready        ( axiOut_1_arready   ),
    .axiOut_1_araddr         ( axiOut_1_araddr    ),
    .axiOut_1_arid           ( axiOut_1_arid      ),
    .axiOut_1_arlen          ( axiOut_1_arlen     ),
    .axiOut_1_arsize         ( axiOut_1_arsize    ),
    .axiOut_1_arburst        ( axiOut_1_arburst   ),
    .axiOut_1_arlock         ( axiOut_1_arlock    ),
    .axiOut_1_arcache        ( axiOut_1_arcache   ),
    .axiOut_1_arprot         ( axiOut_1_arprot    ),
    //r
    .axiOut_1_rvalid         ( axiOut_1_rvalid    ),
    .axiOut_1_rready         ( axiOut_1_rready    ),
    .axiOut_1_rdata          ( axiOut_1_rdata     ),
    .axiOut_1_rid            ( axiOut_1_rid       ),
    .axiOut_1_rresp          ( axiOut_1_rresp     ),
    .axiOut_1_rlast          ( axiOut_1_rlast     ),

    //slave 2
    //aw
    .axiOut_2_awvalid        ( uart_awvalid   ),
    .axiOut_2_awready        ( uart_awready   ),
    .axiOut_2_awaddr         ( uart_awaddr    ),
    .axiOut_2_awid           ( uart_awid      ),
    .axiOut_2_awlen          ( uart_awlen     ),
    .axiOut_2_awsize         ( uart_awsize    ),
    .axiOut_2_awburst        ( uart_awburst   ),
    .axiOut_2_awlock         ( uart_awlock    ),
    .axiOut_2_awcache        ( uart_awcache   ),
    .axiOut_2_awprot         ( uart_awprot    ),
    //w
    .axiOut_2_wvalid         ( uart_wvalid    ),
    .axiOut_2_wready         ( uart_wready    ),
    .axiOut_2_wdata          ( uart_wdata     ),
    .axiOut_2_wstrb          ( uart_wstrb     ),
    .axiOut_2_wlast          ( uart_wlast     ),
    //b
    .axiOut_2_bready         ( uart_bready    ),
    .axiOut_2_bvalid         ( uart_bvalid    ),
    .axiOut_2_bid            ( uart_bid       ),
    .axiOut_2_bresp          ( uart_bresp     ),
    //ar
    .axiOut_2_arvalid        ( uart_arvalid   ),
    .axiOut_2_arready        ( uart_arready   ),
    .axiOut_2_araddr         ( uart_araddr    ),
    .axiOut_2_arid           ( uart_arid      ),
    .axiOut_2_arlen          ( uart_arlen     ),
    .axiOut_2_arsize         ( uart_arsize    ),
    .axiOut_2_arburst        ( uart_arburst   ),
    .axiOut_2_arlock         ( uart_arlock    ),
    .axiOut_2_arcache        ( uart_arcache   ),
    .axiOut_2_arprot         ( uart_arprot    ),
    //r
    .axiOut_2_rvalid         ( uart_rvalid    ),
    .axiOut_2_rready         ( uart_rready    ),
    .axiOut_2_rdata          ( uart_rdata     ),
    .axiOut_2_rid            ( uart_rid       ),
    .axiOut_2_rresp          ( uart_rresp     ),
    .axiOut_2_rlast          ( uart_rlast     ),

    //slave 3
    //aw
    .axiOut_3_awvalid        ( dvi_awvalid   ),
    .axiOut_3_awready        ( dvi_awready   ),
    .axiOut_3_awaddr         ( dvi_awaddr    ),
    .axiOut_3_awid           ( dvi_awid      ),
    .axiOut_3_awlen          ( dvi_awlen     ),
    .axiOut_3_awsize         ( dvi_awsize    ),
    .axiOut_3_awburst        ( dvi_awburst   ),
    .axiOut_3_awlock         ( dvi_awlock    ),
    .axiOut_3_awcache        ( dvi_awcache   ),
    .axiOut_3_awprot         ( dvi_awprot    ),
    //w
    .axiOut_3_wvalid         ( dvi_wvalid    ),
    .axiOut_3_wready         ( dvi_wready    ),
    .axiOut_3_wdata          ( dvi_wdata     ),
    .axiOut_3_wstrb          ( dvi_wstrb     ),
    .axiOut_3_wlast          ( dvi_wlast     ),
    //b
    .axiOut_3_bready         ( dvi_bready    ),
    .axiOut_3_bvalid         ( dvi_bvalid    ),
    .axiOut_3_bid            ( dvi_bid       ),
    .axiOut_3_bresp          ( dvi_bresp     ),
    //ar
    .axiOut_3_arvalid        ( dvi_arvalid   ),
    .axiOut_3_arready        ( dvi_arready   ),
    .axiOut_3_araddr         ( dvi_araddr    ),
    .axiOut_3_arid           ( dvi_arid      ),
    .axiOut_3_arlen          ( dvi_arlen     ),
    .axiOut_3_arsize         ( dvi_arsize    ),
    .axiOut_3_arburst        ( dvi_arburst   ),
    .axiOut_3_arlock         ( dvi_arlock    ),
    .axiOut_3_arcache        ( dvi_arcache   ),
    .axiOut_3_arprot         ( dvi_arprot    ),
    //r
    .axiOut_3_rvalid         ( dvi_rvalid    ),
    .axiOut_3_rready         ( dvi_rready    ),
    .axiOut_3_rdata          ( dvi_rdata     ),
    .axiOut_3_rid            ( dvi_rid       ),
    .axiOut_3_rresp          ( dvi_rresp     ),
    .axiOut_3_rlast          ( dvi_rlast     ),


    //slave 4
    //aw
    .axiOut_4_awvalid        ( confreg_awvalid   ),
    .axiOut_4_awready        ( confreg_awready   ),
    .axiOut_4_awaddr         ( confreg_awaddr    ),
    .axiOut_4_awid           ( confreg_awid      ),
    .axiOut_4_awlen          ( confreg_awlen     ),
    .axiOut_4_awsize         ( confreg_awsize    ),
    .axiOut_4_awburst        ( confreg_awburst   ),
    .axiOut_4_awlock         ( confreg_awlock    ),
    .axiOut_4_awcache        ( confreg_awcache   ),
    .axiOut_4_awprot         ( confreg_awprot    ),
    //w
    .axiOut_4_wvalid         ( confreg_wvalid    ),
    .axiOut_4_wready         ( confreg_wready    ),
    .axiOut_4_wdata          ( confreg_wdata     ),
    .axiOut_4_wstrb          ( confreg_wstrb     ),
    .axiOut_4_wlast          ( confreg_wlast     ),
    //b
    .axiOut_4_bready         ( confreg_bready    ),
    .axiOut_4_bvalid         ( confreg_bvalid    ),
    .axiOut_4_bid            ( confreg_bid       ),
    .axiOut_4_bresp          ( confreg_bresp     ),
    //ar
    .axiOut_4_arvalid        ( confreg_arvalid   ),
    .axiOut_4_arready        ( confreg_arready   ),
    .axiOut_4_araddr         ( confreg_araddr    ),
    .axiOut_4_arid           ( confreg_arid      ),
    .axiOut_4_arlen          ( confreg_arlen     ),
    .axiOut_4_arsize         ( confreg_arsize    ),
    .axiOut_4_arburst        ( confreg_arburst   ),
    .axiOut_4_arlock         ( confreg_arlock    ),
    .axiOut_4_arcache        ( confreg_arcache   ),
    .axiOut_4_arprot         ( confreg_arprot    ),
    //r
    .axiOut_4_rvalid         ( confreg_rvalid    ),
    .axiOut_4_rready         ( confreg_rready    ),
    .axiOut_4_rdata          ( confreg_rdata     ),
    .axiOut_4_rid            ( confreg_rid       ),
    .axiOut_4_rresp          ( confreg_rresp     ),
    .axiOut_4_rlast          ( confreg_rlast     ),

    //slave 5
    //aw
    .axiOut_5_awvalid        ( dma_s_awvalid   ),
    .axiOut_5_awready        ( dma_s_awready   ),
    .axiOut_5_awaddr         ( dma_s_awaddr    ),
    .axiOut_5_awid           ( dma_s_awid      ),
    .axiOut_5_awlen          ( dma_s_awlen     ),
    .axiOut_5_awsize         ( dma_s_awsize    ),
    .axiOut_5_awburst        ( dma_s_awburst   ),
    .axiOut_5_awlock         ( dma_s_awlock    ),
    .axiOut_5_awcache        ( dma_s_awcache   ),
    .axiOut_5_awprot         ( dma_s_awprot    ),
    //w
    .axiOut_5_wvalid         ( dma_s_wvalid    ),
    .axiOut_5_wready         ( dma_s_wready    ),
    .axiOut_5_wdata          ( dma_s_wdata     ),
    .axiOut_5_wstrb          ( dma_s_wstrb     ),
    .axiOut_5_wlast          ( dma_s_wlast     ),
    //b
    .axiOut_5_bready         ( dma_s_bready    ),
    .axiOut_5_bvalid         ( dma_s_bvalid    ),
    .axiOut_5_bid            ( dma_s_bid       ),
    .axiOut_5_bresp          ( dma_s_bresp     ),
    //ar
    .axiOut_5_arvalid        ( dma_s_arvalid   ),
    .axiOut_5_arready        ( dma_s_arready   ),
    .axiOut_5_araddr         ( dma_s_araddr    ),
    .axiOut_5_arid           ( dma_s_arid      ),
    .axiOut_5_arlen          ( dma_s_arlen     ),
    .axiOut_5_arsize         ( dma_s_arsize    ),
    .axiOut_5_arburst        ( dma_s_arburst   ),
    .axiOut_5_arlock         ( dma_s_arlock    ),
    .axiOut_5_arcache        ( dma_s_arcache   ),
    .axiOut_5_arprot         ( dma_s_arprot    ),
    //r
    .axiOut_5_rvalid         ( dma_s_rvalid    ),
    .axiOut_5_rready         ( dma_s_rready    ),
    .axiOut_5_rdata          ( dma_s_rdata     ),
    .axiOut_5_rid            ( dma_s_rid       ),
    .axiOut_5_rresp          ( dma_s_rresp     ),
    .axiOut_5_rlast          ( dma_s_rlast     ),

    //slave 6
    //aw
    .axiOut_6_awvalid        ( fft_awvalid     ),
    .axiOut_6_awready        ( fft_awready     ),
    .axiOut_6_awaddr         ( fft_awaddr      ),
    .axiOut_6_awid           ( fft_awid        ),
    .axiOut_6_awlen          ( fft_awlen       ),
    .axiOut_6_awsize         ( fft_awsize      ),
    .axiOut_6_awburst        ( fft_awburst     ),
    .axiOut_6_awlock         ( fft_awlock      ),
    .axiOut_6_awcache        ( fft_awcache     ),
    .axiOut_6_awprot         ( fft_awprot      ),
    //w
    .axiOut_6_wvalid         ( fft_wvalid      ),
    .axiOut_6_wready         ( fft_wready      ),
    .axiOut_6_wdata          ( fft_wdata       ),
    .axiOut_6_wstrb          ( fft_wstrb       ),
    .axiOut_6_wlast          ( fft_wlast       ),
    //b
    .axiOut_6_bready         ( fft_bready      ),
    .axiOut_6_bvalid         ( fft_bvalid      ),
    .axiOut_6_bid            ( fft_bid         ),
    .axiOut_6_bresp          ( fft_bresp       ),
    //ar
    .axiOut_6_arvalid        ( fft_arvalid     ),
    .axiOut_6_arready        ( fft_arready     ),
    .axiOut_6_araddr         ( fft_araddr      ),
    .axiOut_6_arid           ( fft_arid        ),
    .axiOut_6_arlen          ( fft_arlen       ),
    .axiOut_6_arsize         ( fft_arsize      ),
    .axiOut_6_arburst        ( fft_arburst     ),
    .axiOut_6_arlock         ( fft_arlock      ),
    .axiOut_6_arcache        ( fft_arcache     ),
    .axiOut_6_arprot         ( fft_arprot      ),
    //r
    .axiOut_6_rvalid         ( fft_rvalid      ),
    .axiOut_6_rready         ( fft_rready      ),
    .axiOut_6_rdata          ( fft_rdata       ),
    .axiOut_6_rid            ( fft_rid         ),
    .axiOut_6_rresp          ( fft_rresp       ),
    .axiOut_6_rlast          ( fft_rlast       ),

    //slave 7
    //aw
    .axiOut_7_awvalid        ( axiOut_7_awvalid   ),
    .axiOut_7_awready        ( axiOut_7_awready   ),
    .axiOut_7_awaddr         ( axiOut_7_awaddr    ),
    .axiOut_7_awid           ( axiOut_7_awid      ),
    .axiOut_7_awlen          ( axiOut_7_awlen     ),
    .axiOut_7_awsize         ( axiOut_7_awsize    ),
    .axiOut_7_awburst        ( axiOut_7_awburst   ),
    .axiOut_7_awlock         ( axiOut_7_awlock    ),
    .axiOut_7_awcache        ( axiOut_7_awcache   ),
    .axiOut_7_awprot         ( axiOut_7_awprot    ),
    //w
    .axiOut_7_wvalid         ( axiOut_7_wvalid    ),
    .axiOut_7_wready         ( axiOut_7_wready    ),
    .axiOut_7_wdata          ( axiOut_7_wdata     ),
    .axiOut_7_wstrb          ( axiOut_7_wstrb     ),
    .axiOut_7_wlast          ( axiOut_7_wlast     ),
    //b
    .axiOut_7_bready         ( axiOut_7_bready    ),
    .axiOut_7_bvalid         ( axiOut_7_bvalid    ),
    .axiOut_7_bid            ( axiOut_7_bid       ),
    .axiOut_7_bresp          ( axiOut_7_bresp     ),
    //ar
    .axiOut_7_arvalid        ( axiOut_7_arvalid   ),
    .axiOut_7_arready        ( axiOut_7_arready   ),
    .axiOut_7_araddr         ( axiOut_7_araddr    ),
    .axiOut_7_arid           ( axiOut_7_arid      ),
    .axiOut_7_arlen          ( axiOut_7_arlen     ),
    .axiOut_7_arsize         ( axiOut_7_arsize    ),
    .axiOut_7_arburst        ( axiOut_7_arburst   ),
    .axiOut_7_arlock         ( axiOut_7_arlock    ),
    .axiOut_7_arcache        ( axiOut_7_arcache   ),
    .axiOut_7_arprot         ( axiOut_7_arprot    ),
    //r
    .axiOut_7_rvalid         ( axiOut_7_rvalid    ),
    .axiOut_7_rready         ( axiOut_7_rready    ),
    .axiOut_7_rdata          ( axiOut_7_rdata     ),
    .axiOut_7_rid            ( axiOut_7_rid       ),
    .axiOut_7_rresp          ( axiOut_7_rresp     ),
    .axiOut_7_rlast          ( axiOut_7_rlast     )

);

// add your code

core_top u_cpu(  
    .intrpt    ({7'h0,confreg_int}),   //high active  
  
    .aclk      (cpu_clk       ),  
    .aresetn   (cpu_resetn    ),   //low active  
  
    .arid      (cpu_arid      ),  
    .araddr    (cpu_araddr    ),  
    .arlen     (cpu_arlen     ),  
    .arsize    (cpu_arsize    ),  
    .arburst   (cpu_arburst   ),  
    .arlock    (cpu_arlock    ),  
    .arcache   (cpu_arcache   ),  
    .arprot    (cpu_arprot    ),  
    .arvalid   (cpu_arvalid   ),  
    .arready   (cpu_arready   ),  
                  
    .rid       (cpu_rid       ),  
    .rdata     (cpu_rdata     ),  
    .rresp     (cpu_rresp     ),  
    .rlast     (cpu_rlast     ),  
    .rvalid    (cpu_rvalid    ),  
    .rready    (cpu_rready    ),  
                 
    .awid      (cpu_awid      ),  
    .awaddr    (cpu_awaddr    ),  
    .awlen     (cpu_awlen     ),  
    .awsize    (cpu_awsize    ),  
    .awburst   (cpu_awburst   ),  
    .awlock    (cpu_awlock    ),  
    .awcache   (cpu_awcache   ),  
    .awprot    (cpu_awprot    ),  
    .awvalid   (cpu_awvalid   ),  
    .awready   (cpu_awready   ),  
      
    .wid       (cpu_wid       ),  
    .wdata     (cpu_wdata     ),  
    .wstrb     (cpu_wstrb     ),  
    .wlast     (cpu_wlast     ),  
    .wvalid    (cpu_wvalid    ),  
    .wready    (cpu_wready    ),  
      
    .bid       (cpu_bid       ),  
    .bresp     (cpu_bresp     ),  
    .bvalid    (cpu_bvalid    ),  
    .bready    (cpu_bready    ),  
  
    //debug interface  
    .break_point        (1'b0               ),  
    .infor_flag         (1'b0               ),  
    .reg_num            (5'b0               ),  
    .ws_valid           (                   ),  
    .rf_rdata           (                   ),  
  
    .debug0_wb_pc       (debug_wb_pc        ),  
    .debug0_wb_inst     (debug_wb_inst      ),  
    .debug0_wb_rf_wen   (debug_wb_rf_wen    ),  
    .debug0_wb_rf_wnum  (debug_wb_rf_wnum   ),  
    .debug0_wb_rf_wdata (debug_wb_rf_wdata  )  
);  

//clock sync: from CPU to AXI_Crossbar  
Axi_CDC  u_Axi_CDC (  
    .axiInClk                ( cpu_clk             ),  
    .axiInRstn               ( cpu_resetn          ),  
    .axiOutClk               ( sys_clk             ),  
    .axiOutRstn              ( sys_resetn          ),  
  
    .axiIn_awvalid           ( cpu_awvalid         ),  
    .axiIn_awaddr            ( cpu_awaddr          ),  
    .axiIn_awid              ( {1'b0,cpu_awid}     ),  
    .axiIn_awlen             ( cpu_awlen           ),  
    .axiIn_awsize            ( cpu_awsize          ),  
    .axiIn_awburst           ( cpu_awburst         ),  
    .axiIn_awlock            ( cpu_awlock[0]       ),  
    .axiIn_awcache           ( cpu_awcache         ),  
    .axiIn_awprot            ( cpu_awprot          ),  
    .axiIn_wvalid            ( cpu_wvalid          ),  
    .axiIn_wdata             ( cpu_wdata           ),  
    .axiIn_wstrb             ( cpu_wstrb           ),  
    .axiIn_wlast             ( cpu_wlast           ),  
    .axiIn_bready            ( cpu_bready          ),  
    .axiIn_arvalid           ( cpu_arvalid         ),  
    .axiIn_araddr            ( cpu_araddr          ),  
    .axiIn_arid              ( {1'b0,cpu_arid}     ),  
    .axiIn_arlen             ( cpu_arlen           ),  
    .axiIn_arsize            ( cpu_arsize          ),  
    .axiIn_arburst           ( cpu_arburst         ),  
    .axiIn_arlock            ( cpu_arlock[0]       ),  
    .axiIn_arcache           ( cpu_arcache         ),  
    .axiIn_arprot            ( cpu_arprot          ),  
    .axiIn_rready            ( cpu_rready          ),  
    .axiOut_awready          ( cpu_sync_awready    ),  
    .axiOut_wready           ( cpu_sync_wready     ),  
    .axiOut_bvalid           ( cpu_sync_bvalid     ),  
    .axiOut_bid              ( {1'b0,cpu_sync_bid}),  
    .axiOut_bresp            ( cpu_sync_bresp      ),  
    .axiOut_arready          ( cpu_sync_arready    ),  
    .axiOut_rvalid           ( cpu_sync_rvalid     ),  
    .axiOut_rdata            ( cpu_sync_rdata      ),  
    .axiOut_rid              ( {1'b0,cpu_sync_rid}),  
    .axiOut_rresp            ( cpu_sync_rresp      ),  
    .axiOut_rlast            ( cpu_sync_rlast      ),  
  
    .axiIn_awready           ( cpu_awready         ),  
    .axiIn_wready            ( cpu_wready          ),  
    .axiIn_bvalid            ( cpu_bvalid          ),  
    .axiIn_bid               ( {cpu_bid_4,cpu_bid} ),  
    .axiIn_bresp             ( cpu_bresp           ),  
    .axiIn_arready           ( cpu_arready         ),  
    .axiIn_rvalid            ( cpu_rvalid          ),  
    .axiIn_rdata             ( cpu_rdata           ),  
    .axiIn_rid               ( {cpu_rid_4,cpu_rid} ),  
    .axiIn_rresp             ( cpu_rresp           ),  
    .axiIn_rlast             ( cpu_rlast           ),  
    .axiOut_awvalid          ( cpu_sync_awvalid    ),  
    .axiOut_awaddr           ( cpu_sync_awaddr     ),  
    .axiOut_awid             ( {cpu_sync_awid_4,cpu_sync_awid}),  
    .axiOut_awlen            ( cpu_sync_awlen      ),  
    .axiOut_awsize           ( cpu_sync_awsize     ),  
    .axiOut_awburst          ( cpu_sync_awburst    ),  
    .axiOut_awlock           ( cpu_sync_awlock     ),  
    .axiOut_awcache          ( cpu_sync_awcache    ),  
    .axiOut_awprot           ( cpu_sync_awprot     ),  
    .axiOut_wvalid           ( cpu_sync_wvalid     ),  
    .axiOut_wdata            ( cpu_sync_wdata      ),  
    .axiOut_wstrb            ( cpu_sync_wstrb      ),  
    .axiOut_wlast            ( cpu_sync_wlast      ),  
    .axiOut_bready           ( cpu_sync_bready     ),  
    .axiOut_arvalid          ( cpu_sync_arvalid    ),  
    .axiOut_araddr           ( cpu_sync_araddr     ),  
    .axiOut_arid             ( {cpu_sync_arid_4,cpu_sync_arid}),  
    .axiOut_arlen            ( cpu_sync_arlen      ),  
    .axiOut_arsize           ( cpu_sync_arsize     ),  
    .axiOut_arburst          ( cpu_sync_arburst    ),  
    .axiOut_arlock           ( cpu_sync_arlock     ),  
    .axiOut_arcache          ( cpu_sync_arcache    ),  
    .axiOut_arprot           ( cpu_sync_arprot     ),  
    .axiOut_rready           ( cpu_sync_rready     )  
);  

//axi ram
// 按 soc_top_bram.v 中已验证通过的片上 SRAM 方案，将 axiOut_0
// 从外部异步 SRAM 控制器切换为 axi_wrap_ram_sp + fpga_sram_sp。
axi_wrap_ram_sp #(
    .Init_File     ( RAM_INIT_FILE       )
) u_axi_ram (
    .aclk          ( sys_clk            ),
    .aresetn       ( sys_resetn         ),
    //ar
    .axi_arid      ( ram_arid           ),
    .axi_araddr    ( ram_araddr         ),
    .axi_arlen     ( ram_arlen          ),
    .axi_arsize    ( ram_arsize         ),
    .axi_arburst   ( ram_arburst        ),
    .axi_arlock    ( {1'b0, ram_arlock} ),
    .axi_arcache   ( ram_arcache        ),
    .axi_arprot    ( ram_arprot         ),
    .axi_arvalid   ( ram_arvalid        ),
    .axi_arready   ( ram_arready        ),
    //r
    .axi_rid       ( ram_rid            ),
    .axi_rdata     ( ram_rdata          ),
    .axi_rresp     ( ram_rresp          ),
    .axi_rlast     ( ram_rlast          ),
    .axi_rvalid    ( ram_rvalid         ),
    .axi_rready    ( ram_rready         ),
    //aw
    .axi_awid      ( ram_awid           ),
    .axi_awaddr    ( ram_awaddr         ),
    .axi_awlen     ( ram_awlen          ),
    .axi_awsize    ( ram_awsize         ),
    .axi_awburst   ( ram_awburst        ),
    .axi_awlock    ( {1'b0, ram_awlock} ),
    .axi_awcache   ( ram_awcache        ),
    .axi_awprot    ( ram_awprot         ),
    .axi_awvalid   ( ram_awvalid        ),
    .axi_awready   ( ram_awready        ),
    //w
    .axi_wdata     ( ram_wdata          ),
    .axi_wstrb     ( ram_wstrb          ),
    .axi_wlast     ( ram_wlast          ),
    .axi_wvalid    ( ram_wvalid         ),
    .axi_wready    ( ram_wready         ),
    //b
    .axi_bid       ( ram_bid            ),
    .axi_bresp     ( ram_bresp          ),
    .axi_bvalid    ( ram_bvalid         ),
    .axi_bready    ( ram_bready         )
);

//AXI2APB  
axi_uart_controller u_axi_uart_controller  
(  
    .clk                (sys_clk                ),  
    .rst_n              (sys_resetn             ),  
  
    .axi_s_awid         (uart_awid              ),  
    .axi_s_awaddr       (uart_awaddr            ),  
    .axi_s_awlen        (uart_awlen             ),  
    .axi_s_awsize       (uart_awsize            ),  
    .axi_s_awburst      (uart_awburst           ),  
    .axi_s_awlock       (uart_awlock            ),  
    .axi_s_awcache      (uart_awcache           ),  
    .axi_s_awprot       (uart_awprot            ),  
    .axi_s_awvalid      (uart_awvalid           ),  
    .axi_s_awready      (uart_awready           ),  
    .axi_s_wid          (uart_awid              ),  
    .axi_s_wdata        (uart_wdata             ),  
    .axi_s_wstrb        (uart_wstrb             ),  
    .axi_s_wlast        (uart_wlast             ),  
    .axi_s_wvalid       (uart_wvalid            ),  
    .axi_s_wready       (uart_wready            ),  
    .axi_s_bid          (uart_bid               ),  
    .axi_s_bresp        (uart_bresp             ),  
    .axi_s_bvalid       (uart_bvalid            ),  
    .axi_s_bready       (uart_bready            ),  
    .axi_s_arid         (uart_arid              ),  
    .axi_s_araddr       (uart_araddr            ),  
    .axi_s_arlen        (uart_arlen             ),  
    .axi_s_arsize       (uart_arsize            ),  
    .axi_s_arburst      (uart_arburst           ),  
    .axi_s_arlock       (uart_arlock            ),  
    .axi_s_arcache      (uart_arcache           ),  
    .axi_s_arprot       (uart_arprot            ),  
    .axi_s_arvalid      (uart_arvalid           ),  
    .axi_s_arready      (uart_arready           ),  
    .axi_s_rid          (uart_rid               ),  
    .axi_s_rdata        (uart_rdata             ),  
    .axi_s_rresp        (uart_rresp             ),  
    .axi_s_rlast        (uart_rlast             ),  
    .axi_s_rvalid       (uart_rvalid            ),  
    .axi_s_rready       (uart_rready            ),  
  
    .apb_rw_dma         (1'b0                   ),  
    .apb_psel_dma       (1'b0                   ),  
    .apb_enab_dma       (1'b0                   ),  
    .apb_addr_dma       (20'b0                  ),  
    .apb_valid_dma      (1'b0                   ),  
    .apb_wdata_dma      (32'b0                  ),  
    .apb_rdata_dma      (                       ),  
    .apb_ready_dma      (                       ),   
    .dma_grant          (                       ),  
  
    .dma_req_o          (                       ),  
    .dma_ack_i          (1'b0                   ),  
  
    //UART0  
    .uart0_txd_i        (uart0_txd_i            ),  
    .uart0_txd_o        (uart0_txd_o            ),  
    .uart0_txd_oe       (uart0_txd_oe           ),  
    .uart0_rxd_i        (uart0_rxd_i            ),  
    .uart0_rxd_o        (uart0_rxd_o            ),  
    .uart0_rxd_oe       (uart0_rxd_oe           ),  
    .uart0_rts_o        (uart0_rts_o            ),  
    .uart0_dtr_o        (uart0_dtr_o            ),  
    .uart0_cts_i        (uart0_cts_i            ),  
    .uart0_dsr_i        (uart0_dsr_i            ),  
    .uart0_dcd_i        (uart0_dcd_i            ),  
    .uart0_ri_i         (uart0_ri_i             ),  
    .uart0_int          (uart0_int              )  
);  

confreg #(.SIMULATION(SIMULATION)) u_confreg (
    .aclk                    ( sys_clk             ),
    .aresetn                 ( sys_resetn          ),
    .cpu_clk                 ( cpu_clk             ),
    .cpu_resetn              ( cpu_resetn          ),
    .s_awid                  ( confreg_awid        ),
    .s_awaddr                ( confreg_awaddr      ),
    .s_awlen                 ( confreg_awlen       ),
    .s_awsize                ( confreg_awsize      ),
    .s_awburst               ( confreg_awburst     ),
    .s_awlock                ( confreg_awlock      ),
    .s_awcache               ( confreg_awcache     ),
    .s_awprot                ( confreg_awprot      ),
    .s_awvalid               ( confreg_awvalid     ),
    .s_wid                   ( confreg_wid         ),
    .s_wdata                 ( confreg_wdata       ),
    .s_wstrb                 ( confreg_wstrb       ),
    .s_wlast                 ( confreg_wlast       ),
    .s_wvalid                ( confreg_wvalid      ),
    .s_bready                ( confreg_bready      ),
    .s_arid                  ( confreg_arid        ),
    .s_araddr                ( confreg_araddr      ),
    .s_arlen                 ( confreg_arlen       ),
    .s_arsize                ( confreg_arsize      ),
    .s_arburst               ( confreg_arburst     ),
    .s_arlock                ( confreg_arlock      ),
    .s_arcache               ( confreg_arcache     ),
    .s_arprot                ( confreg_arprot      ),
    .s_arvalid               ( confreg_arvalid     ),
    .s_rready                ( confreg_rready      ),

    .s_awready               ( confreg_awready     ),
    .s_wready                ( confreg_wready      ),
    .s_bid                   ( confreg_bid         ),
    .s_bresp                 ( confreg_bresp       ),
    .s_bvalid                ( confreg_bvalid      ),
    .s_arready               ( confreg_arready     ),
    .s_rid                   ( confreg_rid         ),
    .s_rdata                 ( confreg_rdata       ),
    .s_rresp                 ( confreg_rresp       ),
    .s_rlast                 ( confreg_rlast       ),
    .s_rvalid                ( confreg_rvalid      ),

    .switch                  ( dip_sw              ),
    .touch_btn               ( touch_btn           ),
    .led                     ( leds                ),
    .dpy0                    ( dpy0                ),
    .dpy1                    ( dpy1                ),
    .digital_value           ( digital_value       ),
    .dma_finish              ( dma_finish          ),
    .fft_finish              ( fft_finish          ), 
    .confreg_int             ( confreg_int         )
);

wildfire_lock_display u_lock_display (
    .clk         (sys_clk),
    .resetn      (sys_resetn),
    .digit       (digital_value),
    .seg_cathode (seg_cathode),
    .seg_anode   (seg_anode)
);

// DVI
axi_dvi  u_axi_dvi (  
    .s_awvalid                  ( dvi_awvalid   ),  
    .s_awaddr                   ( dvi_awaddr    ),  
    .s_awid                     ( dvi_awid      ),  
    .s_awlen                    ( dvi_awlen     ),  
    .s_awsize                   ( dvi_awsize    ),  
    .s_awburst                  ( dvi_awburst   ),  
    .s_awlock                   ( dvi_awlock    ),  
    .s_awcache                  ( dvi_awcache   ),  
    .s_awprot                   ( dvi_awprot    ),  
    .s_wvalid                   ( dvi_wvalid    ),  
    .s_wdata                    ( dvi_wdata     ),  
    .s_wstrb                    ( dvi_wstrb     ),  
    .s_wlast                    ( dvi_wlast     ),  
    .s_bready                   ( dvi_bready    ),  
    .s_arvalid                  ( dvi_arvalid   ),  
    .s_araddr                   ( dvi_araddr    ),  
    .s_arid                     ( dvi_arid      ),  
    .s_arlen                    ( dvi_arlen     ),  
    .s_arsize                   ( dvi_arsize    ),  
    .s_arburst                  ( dvi_arburst   ),  
    .s_arlock                   ( dvi_arlock    ),  
    .s_arcache                  ( dvi_arcache   ),  
    .s_arprot                   ( dvi_arprot    ),  
    .s_rready                   ( dvi_rready    ),  
    .aclk                       ( sys_clk       ),  
    .aresetn                    ( sys_resetn    ),  
  
    .s_awready                  ( dvi_awready   ),  
    .s_wready                   ( dvi_wready    ),  
    .s_bvalid                   ( dvi_bvalid    ),  
    .s_bid                      ( dvi_bid       ),  
    .s_bresp                    ( dvi_bresp     ),  
    .s_arready                  ( dvi_arready   ),  
    .s_rvalid                   ( dvi_rvalid    ),  
    .s_rdata                    ( dvi_rdata     ),  
    .s_rid                      ( dvi_rid       ),  
    .s_rresp                    ( dvi_rresp     ),  
    .s_rlast                    ( dvi_rlast     ),  
    .video_clk                  ( video_clk     ),  
    .hsync                      ( video_hsync   ),  
    .vsync                      ( video_vsync   ),  
    .data_enable                ( video_de      ),  
    .video_red                  ( video_red     ),  
    .video_green                ( video_green   ),  
    .video_blue                 ( video_blue    )  
);  



// DMA
dma_top  u_dma_top (  
    .aclk                    ( sys_clk         ),  
    .aresetn                 ( sys_resetn      ),  
  
    .s_awid                  ( dma_s_awid       ),  
    .s_awaddr                ( dma_s_awaddr     ),  
    .s_awlen                 ( dma_s_awlen      ),  
    .s_awsize                ( dma_s_awsize     ),  
    .s_awburst               ( dma_s_awburst    ),  
    .s_awlock                ( dma_s_awlock     ),  
    .s_awcache               ( dma_s_awcache    ),  
    .s_awprot                ( dma_s_awprot     ),  
    .s_awvalid               ( dma_s_awvalid    ),   
    .s_wdata                 ( dma_s_wdata      ),  
    .s_wstrb                 ( dma_s_wstrb      ),  
    .s_wlast                 ( dma_s_wlast      ),  
    .s_wvalid                ( dma_s_wvalid     ),  
    .s_bready                ( dma_s_bready     ),  
    .s_arid                  ( dma_s_arid       ),  
    .s_araddr                ( dma_s_araddr     ),  
    .s_arlen                 ( dma_s_arlen      ),  
    .s_arsize                ( dma_s_arsize     ),  
    .s_arburst               ( dma_s_arburst    ),  
    .s_arlock                ( dma_s_arlock     ),  
    .s_arcache               ( dma_s_arcache    ),  
    .s_arprot                ( dma_s_arprot     ),  
    .s_arvalid               ( dma_s_arvalid    ),  
    .s_rready                ( dma_s_rready     ),  
    .m_arready               ( dma_m_arready    ),  
    .m_rid                   ( dma_m_rid        ),  
    .m_rdata                 ( dma_m_rdata      ),  
    .m_rresp                 ( dma_m_rresp      ),  
    .m_rlast                 ( dma_m_rlast      ),  
    .m_rvalid                ( dma_m_rvalid     ),  
    .m_awready               ( dma_m_awready    ),  
    .m_wready                ( dma_m_wready     ),  
    .m_bid                   ( dma_m_bid        ),  
    .m_bresp                 ( dma_m_bresp      ),  
    .m_bvalid                ( dma_m_bvalid     ),  
  
    .dma_finish              ( dma_finish       ),  
    .s_awready               ( dma_s_awready    ),  
    .s_wready                ( dma_s_wready     ),  
    .s_bid                   ( dma_s_bid        ),  
    .s_bresp                 ( dma_s_bresp      ),  
    .s_bvalid                ( dma_s_bvalid     ),  
    .s_arready               ( dma_s_arready    ),  
    .s_rid                   ( dma_s_rid        ),  
    .s_rdata                 ( dma_s_rdata      ),  
    .s_rresp                 ( dma_s_rresp      ),  
    .s_rlast                 ( dma_s_rlast      ),  
    .s_rvalid                ( dma_s_rvalid     ),  
    .m_arid                  ( dma_m_arid       ),  
    .m_araddr                ( dma_m_araddr     ),  
    .m_arlen                 ( dma_m_arlen      ),  
    .m_arsize                ( dma_m_arsize     ),  
    .m_arburst               ( dma_m_arburst    ),  
    .m_arlock                ( dma_m_arlock     ),  
    .m_arcache               ( dma_m_arcache    ),  
    .m_arprot                ( dma_m_arprot     ),  
    .m_arvalid               ( dma_m_arvalid    ),  
    .m_rready                ( dma_m_rready     ),  
    .m_awid                  ( dma_m_awid       ),  
    .m_awaddr                ( dma_m_awaddr     ),  
    .m_awlen                 ( dma_m_awlen      ),  
    .m_awsize                ( dma_m_awsize     ),  
    .m_awburst               ( dma_m_awburst    ),  
    .m_awlock                ( dma_m_awlock     ),  
    .m_awcache               ( dma_m_awcache    ),  
    .m_awprot                ( dma_m_awprot     ),  
    .m_awvalid               ( dma_m_awvalid    ),  
    .m_wdata                 ( dma_m_wdata      ),  
    .m_wstrb                 ( dma_m_wstrb      ),  
    .m_wlast                 ( dma_m_wlast      ),  
    .m_wvalid                ( dma_m_wvalid     ),  
    .m_bready                ( dma_m_bready     )  
);

// FFT
axi_fft_top u_axi_fft_top (  
    .aclk                    ( sys_clk             ),  
    .aresetn                 ( sys_resetn          ),  
    .fft_clk                 ( cpu_clk             ),  
    .fft_resetn              ( cpu_resetn          ),      
  
    .s_awid                  ( fft_awid            ),  
    .s_awaddr                ( fft_awaddr          ),  
    .s_awlen                 ( fft_awlen           ),  
    .s_awsize                ( fft_awsize          ),  
    .s_awburst               ( fft_awburst         ),  
    .s_awlock                ( fft_awlock          ),  
    .s_awcache               ( fft_awcache         ),  
    .s_awprot                ( fft_awprot          ),  
    .s_awvalid               ( fft_awvalid         ),  
    .s_wid                   ( fft_wid             ),  
    .s_wdata                 ( fft_wdata           ),  
    .s_wstrb                 ( fft_wstrb           ),  
    .s_wlast                 ( fft_wlast           ),  
    .s_wvalid                ( fft_wvalid          ),  
    .s_bready                ( fft_bready          ),  
    .s_arid                  ( fft_arid            ),  
    .s_araddr                ( fft_araddr          ),  
    .s_arlen                 ( fft_arlen           ),  
    .s_arsize                ( fft_arsize          ),  
    .s_arburst               ( fft_arburst         ),  
    .s_arlock                ( fft_arlock          ),  
    .s_arcache               ( fft_arcache         ),  
    .s_arprot                ( fft_arprot          ),  
    .s_arvalid               ( fft_arvalid         ),  
    .s_rready                ( fft_rready          ),  
      
    .s_awready               ( fft_awready         ),  
    .s_wready                ( fft_wready          ),  
    .s_bid                   ( fft_bid             ),  
    .s_bresp                 ( fft_bresp           ),  
    .s_bvalid                ( fft_bvalid          ),  
    .s_arready               ( fft_arready         ),  
    .s_rid                   ( fft_rid             ),  
    .s_rdata                 ( fft_rdata           ),  
    .s_rresp                 ( fft_rresp           ),  
    .s_rlast                 ( fft_rlast           ),  
    .s_rvalid                ( fft_rvalid          ),  
  
    .fft_finish              ( fft_finish          )  
);  


// ------------------------------------------------------------------------
// net_acq AXI slave @ axiOut_7
// ------------------------------------------------------------------------
net_acq_axi_slave u_net_acq_axi_slave (
    .aclk               (sys_clk),
    .aresetn            (sys_resetn),
    .eth_tx_clk         (eth_tx_clk),
    .eth_tx_resetn      (eth_tx_resetn),
    .eth_rx_clk         (eth_rx_clk),
    .eth_rx_resetn      (eth_rx_resetn),

    // AXI write address channel
    .s_awid             (axiOut_7_awid),
    .s_awaddr           (axiOut_7_awaddr),
    .s_awlen            (axiOut_7_awlen),
    .s_awsize           (axiOut_7_awsize),
    .s_awburst          (axiOut_7_awburst),
    .s_awlock           (axiOut_7_awlock),
    .s_awcache          (axiOut_7_awcache),
    .s_awprot           (axiOut_7_awprot),
    .s_awvalid          (axiOut_7_awvalid),
    .s_awready          (axiOut_7_awready),

    // AXI write data channel
    .s_wid              (axiOut_7_wid),
    .s_wdata            (axiOut_7_wdata),
    .s_wstrb            (axiOut_7_wstrb),
    .s_wlast            (axiOut_7_wlast),
    .s_wvalid           (axiOut_7_wvalid),
    .s_wready           (axiOut_7_wready),

    // AXI write response channel
    .s_bready           (axiOut_7_bready),
    .s_bid              (axiOut_7_bid),
    .s_bresp            (axiOut_7_bresp),
    .s_bvalid           (axiOut_7_bvalid),

    // AXI read address channel
    .s_arid             (axiOut_7_arid),
    .s_araddr           (axiOut_7_araddr),
    .s_arlen            (axiOut_7_arlen),
    .s_arsize           (axiOut_7_arsize),
    .s_arburst          (axiOut_7_arburst),
    .s_arlock           (axiOut_7_arlock),
    .s_arcache          (axiOut_7_arcache),
    .s_arprot           (axiOut_7_arprot),
    .s_arvalid          (axiOut_7_arvalid),
    .s_arready          (axiOut_7_arready),

    // AXI read data channel
    .s_rready           (axiOut_7_rready),
    .s_rid              (axiOut_7_rid),
    .s_rdata            (axiOut_7_rdata),
    .s_rresp            (axiOut_7_rresp),
    .s_rlast            (axiOut_7_rlast),
    .s_rvalid           (axiOut_7_rvalid),

    // external sample source bridge
    .sample_data_in     (net_acq_sample_data_in),
    .sample_valid_in    (net_acq_sample_valid_in),
    .sample_ready_out   (net_acq_sample_ready_out),

    // legacy pkt_* mirror / bypass
    .pkt_data           (net_acq_pkt_data),
    .pkt_valid          (net_acq_pkt_valid),
    .pkt_last           (net_acq_pkt_last),
    .pkt_ready          (net_acq_pkt_ready),

    // new MAC-side TX AXIS-like byte stream
    .m_axis_eth_tx_tdata  (net_acq_m_axis_eth_tx_tdata),
    .m_axis_eth_tx_tvalid (net_acq_m_axis_eth_tx_tvalid),
    .m_axis_eth_tx_tlast  (net_acq_m_axis_eth_tx_tlast),
    .m_axis_eth_tx_tready (net_acq_m_axis_eth_tx_tready),
    .m_axis_eth_tx_tuser  (net_acq_m_axis_eth_tx_tuser),
    .ptp_tod_time_ns_eth  (net_acq_ptp_tod_time_ns),
    .ptp_tod_time_valid_eth(net_acq_ptp_tod_time_valid),

    // Sealed RGMII RX feedback path from top_phy to wrapper.
    .s_axis_eth_rx_tdata  (net_acq_m_axis_eth_rx_tdata),
    .s_axis_eth_rx_tvalid (net_acq_m_axis_eth_rx_tvalid),
    .s_axis_eth_rx_tlast  (net_acq_m_axis_eth_rx_tlast),
    .ptp_rxstamp_ns       (net_acq_ptp_rxstamp_ns),
    .ptp_rxstamp_valid    (net_acq_ptp_rxstamp_valid),
    .ptp_tod_time_ns_rx   (net_acq_ptp_tod_time_ns_rx),
    .ptp_tod_time_valid_rx(net_acq_ptp_tod_time_valid_rx),

    // reserved future PTP / board status
    .tod_time_ns        (net_acq_tod_time_ns),
    .tod_time_valid     (net_acq_tod_time_valid),
    .ptp_txstamp_ns     (net_acq_ptp_txstamp_ns),
    .ptp_txstamp_valid  (net_acq_ptp_txstamp_valid),
    .mac_link_up        (net_acq_mac_link_up),
    .phy_status_word    (net_acq_phy_status_word),
    .phy_reg_snapshot   (net_acq_phy_reg_snapshot),
    .phy_soft_reset_pulse(net_acq_phy_soft_reset_pulse),
    .phy_restart_an_pulse(net_acq_phy_restart_an_pulse)
);

// ------------------------------------------------------------------------
// top_phy : RGMII PHY backend
//   - TX: 接收 net_acq 后端输出的完整 Ethernet frame 字节流并发到外部 PHY
//   - RX: 使用已经过恢复的 rx_clk 做 RGMII 采样并回送到 wrapper/PTP RX
//   - 新增标准化 Clause-22 MDIO 管理与真实 link/speed/duplex 读回
//   - clk125_shift 继续维持当前接法，不在本轮修改中动它
// ------------------------------------------------------------------------
top_phy #(.SIMULATION(SIMULATION)) u_top_phy (
    .clk125              (eth_tx_clk),
    .clk125_shift        (eth_tx_clk),   // 暂时不启用延时
    .rst_n               (eth_tx_resetn),
    .rx_rst_n            (eth_rx_resetn),

    .s_axis_eth_tdata    (net_acq_m_axis_eth_tx_tdata),
    .s_axis_eth_tvalid   (net_acq_m_axis_eth_tx_tvalid),
    .s_axis_eth_tlast    (net_acq_m_axis_eth_tx_tlast),
    .s_axis_eth_tready   (net_acq_m_axis_eth_tx_tready),
    .s_axis_eth_tuser    (net_acq_m_axis_eth_tx_tuser),
    .ptp_tod_time_ns     (net_acq_ptp_tod_time_ns),
    .ptp_tod_time_valid  (net_acq_ptp_tod_time_valid),
    .ptp_tod_time_ns_rx  (net_acq_ptp_tod_time_ns_rx),
    .ptp_tod_time_valid_rx(net_acq_ptp_tod_time_valid_rx),
    .phy_soft_reset_pulse(net_acq_phy_soft_reset_pulse),
    .phy_restart_an_pulse(net_acq_phy_restart_an_pulse),

    .ptp_txstamp_ns      (net_acq_ptp_txstamp_ns),
    .ptp_txstamp_valid   (net_acq_ptp_txstamp_valid),
    .ptp_rxstamp_ns      (net_acq_ptp_rxstamp_ns),
    .ptp_rxstamp_valid   (net_acq_ptp_rxstamp_valid),
    .mac_link_up         (net_acq_mac_link_up),
    .phy_status_word     (net_acq_phy_status_word),
    .phy_reg_snapshot    (net_acq_phy_reg_snapshot),

    .m_axis_eth_rx_tdata (net_acq_m_axis_eth_rx_tdata),
    .m_axis_eth_rx_tvalid(net_acq_m_axis_eth_rx_tvalid),
    .m_axis_eth_rx_tlast (net_acq_m_axis_eth_rx_tlast),

    .rgmii_txc           (rgmii_txc),
    .rgmii_txd           (rgmii_txd),
    .rgmii_tx_ctl        (rgmii_tx_ctl),

    .rgmii_rxc           (eth_rx_clk),
    .rgmii_rxd           (rgmii_rxd),
    .rgmii_rx_ctl        (rgmii_rx_ctl),

    .phy_mdc             (phy_mdc),
    .phy_mdio            (phy_mdio),
    .phy_rst_n           (phy_rst_n)
);
endmodule

