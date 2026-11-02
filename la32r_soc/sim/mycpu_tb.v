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
`timescale 1ns / 1ps
`include "config.h"
`define FFT_OUTPUT_TXT

`define UART_PSEL               u_soc_top.u_axi_uart_controller.uart0.PSEL
`define UART_PENBLE             u_soc_top.u_axi_uart_controller.uart0.PENABLE
`define UART_PWRITE             u_soc_top.u_axi_uart_controller.uart0.PWRITE
`define UART_WADDR              u_soc_top.u_axi_uart_controller.uart0.PADDR[7:0]
`define UART_WDATA              u_soc_top.u_axi_uart_controller.uart0.PWDATA[7:0]
`define END_PC                  32'h1c000200

module tb_top( );
reg reset;
reg clk;
reg   [3:0]  touch_btn;
reg   [31:0]  dip_sw;

wire         UART_RX;
wire         UART_TX;
wire  [2:0]  video_red;
wire  [2:0]  video_green;
wire  [1:0]  video_blue;
wire  video_hsync;
wire  video_vsync;
wire  video_clk;
wire  video_de;
wire  [15:0]  leds;
wire  [7:0]  dpy0;
wire  [7:0]  dpy1;
wire  [19:0]  base_ram_addr;
wire  [ 3:0]  base_ram_be_n;
wire  base_ram_ce_n;
wire  base_ram_oe_n;
wire  base_ram_we_n;
wire  [19:0]  ext_ram_addr;
wire  [ 3:0]  ext_ram_be_n;
wire  ext_ram_ce_n;
wire  ext_ram_oe_n;
wire  ext_ram_we_n;
wire  [31:0]  base_ram_data;
wire  [31:0]  ext_ram_data;

// net_acq external sample source driven by TB
reg   [15:0]  net_acq_sample_data_in;
reg           net_acq_sample_valid_in;
wire          net_acq_sample_ready_out;
reg   [15:0]  net_acq_ext_sample_cnt;
integer       net_acq_ext_accept_cnt;
reg           net_acq_ext_started;

//产生时钟与复位信号
initial begin
    clk                     = 1'b0;
    reset                   = 1'b1;
    dip_sw                  = 32'h0;
    net_acq_sample_data_in  = 16'd0;
    net_acq_sample_valid_in = 1'b0;
    net_acq_ext_sample_cnt  = 16'd0;
    net_acq_ext_accept_cnt  = 0;
    net_acq_ext_started     = 1'b0;
    #2000;
    reset = 1'b0;
end
always #10 clk=~clk;

//产生按键信号
initial begin
    touch_btn = 4'h0;
    dip_sw    = 32'h0000_abcd;

    #3000000;

    #100000
    touch_btn = 4'b0001;
    #50
    touch_btn = 4'b0000;

    #100000
    touch_btn = 4'b0010;
    #50
    touch_btn = 4'b0000;

    #100000
    touch_btn = 4'b0100;
    #50
    touch_btn = 4'b0000;

    #100000
    touch_btn = 4'b1000;
    #50
    touch_btn = 4'b0000;

end

reg rgmii_rxc = 1'b0;
always #4 rgmii_rxc = ~rgmii_rxc;
soc_top #(.SIMULATION(1'b1)) u_soc_top (
    .clk(clk), .reset_n(~reset),
    .UART_RX(UART_RX), .UART_TX(UART_TX),
    .rgmii_rxc(rgmii_rxc), .rgmii_rxd(4'b0), .rgmii_rx_ctl(1'b0),
    .rgmii_txc(), .rgmii_txd(), .rgmii_tx_ctl(),
    .phy_mdc(), .phy_mdio(), .phy_rst_n()
);
assign UART_RX = 1'b1;
assign net_acq_sample_ready_out = u_soc_top.net_acq_sample_ready_out;
initial begin
    force u_soc_top.touch_btn = touch_btn;
    force u_soc_top.dip_sw = dip_sw;
    force u_soc_top.net_acq_sample_data_in = net_acq_sample_data_in;
    force u_soc_top.net_acq_sample_valid_in = net_acq_sample_valid_in;
end

// -----------------------------------------------------------------------------
// net_acq external sample source driver
// 说明：
//   1) internal source 阶段，sample_ready_out 会保持 0，TB 不会送外部样本；
//   2) 软件切到 src_sel=1 且 enable=1 后，sample_ready_out 开始按 sampler
//      真正 ready 语义拉高，TB 在下一个 posedge 到来前送一个 16-bit 样本；
//   3) 这里用递增计数样本，便于波形和后续包内容观察。
// -----------------------------------------------------------------------------
always @(negedge clk) begin
    if (!u_soc_top.sys_resetn) begin
        net_acq_sample_data_in  <= 16'd0;
        net_acq_sample_valid_in <= 1'b0;
        net_acq_ext_sample_cnt  <= 16'd0;
        net_acq_ext_accept_cnt  <= 0;
        net_acq_ext_started     <= 1'b0;
    end else begin
        if (net_acq_sample_ready_out && !net_acq_ext_started) begin
            net_acq_ext_started <= 1'b1;
            $display("[TB] net_acq external sample source active @%0t", $time);
        end

        if (net_acq_sample_ready_out) begin
            net_acq_sample_valid_in <= 1'b1;
            net_acq_sample_data_in  <= net_acq_ext_sample_cnt;
            net_acq_ext_sample_cnt  <= net_acq_ext_sample_cnt + 16'h0001;
            net_acq_ext_accept_cnt  <= net_acq_ext_accept_cnt + 1;
        end else begin
            net_acq_sample_valid_in <= 1'b0;
            net_acq_sample_data_in  <= 16'd0;
        end
    end
end

// The current SoC uses initialized internal BRAM, not external SRAM models.

//模拟串口打印
wire uart_display;
wire [7:0] uart_data;
wire uart_wen;
assign uart_wen = (`UART_PSEL == 1'b1) &&  (`UART_PENBLE == 1'b1) && (`UART_PWRITE == 1'b1);
assign uart_display = (uart_wen == 1'b1) && (`UART_WADDR == 8'h0);
assign uart_data    = `UART_WDATA;

always @(posedge clk)
begin
    if(uart_display)
    begin
        if(uart_data==8'hff)
        begin
            ;//$finish;
        end
        else
        begin
            $write("%c",uart_data);
        end
    end
end

//仿真结束
wire [31:0] debug_wb_pc;
assign debug_wb_pc = u_soc_top.debug_wb_pc;
wire test_end = debug_wb_pc==`END_PC;
always @(posedge u_soc_top.cpu_clk)
begin
    if (!u_soc_top.cpu_resetn) begin
    end
    else if(test_end) begin
        $display("==============================================================");
        $display("Test end!");
	    $finish;
	end
end

//FFT测试结果输出
`ifdef FFT_OUTPUT_TXT
    integer fft_output_re;
    integer fft_output_im;
    initial begin
        fft_output_re = $fopen("fft512_output_re.txt", "w"); 
        fft_output_im = $fopen("fft512_output_im.txt", "w");
        forever begin
        @(posedge u_soc_top.u_axi_fft_top.u_axi_fft_wrap.aclk);
        if(u_soc_top.u_axi_fft_top.u_axi_fft_wrap.valid_out) begin
            $fwrite(fft_output_re, "%04h\n", u_soc_top.u_axi_fft_top.u_axi_fft_wrap.y_re);
            $fwrite(fft_output_im, "%04h\n", u_soc_top.u_axi_fft_top.u_axi_fft_wrap.y_im);
        end
        end
    end
`endif

endmodule
