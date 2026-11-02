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
`define FFT_CONFIG_ADDR    16'h0000 //1f40_0000
`define FFT_INPUT_ADDR     16'h1000 //1f40_1000 - 1f40_17fc
`define FFT_OUTPUT_ADDR    16'h2000 //1f40_2000 - 1f40_27fc

module axi_fft_wrap (
    input           aclk,
    input           aresetn,

    output          fft_finish,

    input  [4 :0]   s_awid,
    input  [31:0]   s_awaddr,
    input  [7 :0]   s_awlen,
    input  [2 :0]   s_awsize,
    input  [1 :0]   s_awburst,
    input           s_awlock,
    input  [3 :0]   s_awcache,
    input  [2 :0]   s_awprot,
    input           s_awvalid,
    output          s_awready,
    input  [4 :0]   s_wid,
    input  [31:0]   s_wdata,
    input  [3 :0]   s_wstrb,
    input           s_wlast,
    input           s_wvalid,
    output reg      s_wready,
    output [4 :0]   s_bid,
    output [1 :0]   s_bresp,
    output reg      s_bvalid,
    input           s_bready,
    input  [4 :0]   s_arid,
    input  [31:0]   s_araddr,
    input  [7 :0]   s_arlen,
    input  [2 :0]   s_arsize,
    input  [1 :0]   s_arburst,
    input           s_arlock,
    input  [3 :0]   s_arcache,
    input  [2 :0]   s_arprot,
    input           s_arvalid,
    output          s_arready,
    output [4 :0]   s_rid,
    output reg [31:0]  s_rdata,
    output [1 :0]   s_rresp,
    output reg      s_rlast,
    output reg      s_rvalid,
    input           s_rready
);

// bit0: start, 1 represents start the operation
// bit1: inv, 0 represents FFT operation, 1 represents IFFT operation
// bit2-bit3: np, FFT/IFFT points, 00 represents 64 points, 01 represents 128 points, 10 represents 256 points, and 11 represents 512 points
// bit4: completed, 1 represents operation completed
reg [31:0] fft_config;

wire fft_input_wen  ;
wire fft_input_cen  ;
wire [8:0] fft_input_addr ;
wire [31:0] fft_input_wdata;//高16位实部，低16位虚部，Q15格式
wire [31:0] fft_input_rdata;//高16位实部，低16位虚部，Q15格式

wire fft_output_wen  ;
wire fft_output_cen  ;
wire [8:0] fft_output_addr ;
wire [31:0] fft_output_wdata;//高16位实部，低16位虚部，Q15格式
wire [31:0] fft_output_rdata;//高16位实部，低16位虚部，Q15格式

reg busy,write,R_or_W;

wire ar_enter = s_arvalid & s_arready;
wire r_retire = s_rvalid & s_rready & s_rlast;
wire aw_enter = s_awvalid & s_awready;
wire w_enter  = s_wvalid & s_wready & s_wlast;
wire b_retire = s_bvalid & s_bready;

assign s_arready = ~busy & (!R_or_W| !s_awvalid);
assign s_awready = ~busy & ( R_or_W| !s_arvalid);

always@(posedge aclk) begin
    if(~aresetn) busy <= 1'b0;
    else if(ar_enter|aw_enter) busy <= 1'b1;
    else if(r_retire|b_retire) busy <= 1'b0;
end

reg [4 :0] buf_id;
reg [31:0] buf_addr;
reg [7 :0] buf_len;
reg [2 :0] buf_size;
reg [1 :0] buf_burst;
reg        buf_lock;
reg [3 :0] buf_cache;
reg [2 :0] buf_prot;

always@(posedge aclk) begin
    if(~aresetn) begin
        R_or_W      <= 1'b0;
        buf_id      <= 'b0;
        buf_addr    <= 'b0;
        buf_len     <= 'b0;
        buf_size    <= 'b0;
        buf_burst   <= 'b0;
        buf_lock    <= 'b0;
        buf_cache   <= 'b0;
        buf_prot    <= 'b0;
    end
    else
    if(ar_enter | aw_enter) begin
        R_or_W      <= ar_enter;
        buf_id      <= ar_enter ? s_arid   : s_awid   ;
        buf_addr    <= ar_enter ? s_araddr : s_awaddr ;
        buf_len     <= ar_enter ? s_arlen  : s_awlen  ;
        buf_size    <= ar_enter ? s_arsize : s_awsize ;
        buf_burst   <= ar_enter ? s_arburst: s_awburst;
        buf_lock    <= ar_enter ? s_arlock : s_awlock ;
        buf_cache   <= ar_enter ? s_arcache: s_awcache;
        buf_prot    <= ar_enter ? s_arprot : s_awprot ;
    end
end

always@(posedge aclk) begin
    if(~aresetn) write <= 1'b0;
    else if(aw_enter) write <= 1'b1;
    else if(ar_enter)  write <= 1'b0;
end

always@(posedge aclk) begin
    if(~aresetn) s_wready <= 1'b0;
    else if(aw_enter) s_wready <= 1'b1;
    else if(w_enter & s_wlast) s_wready <= 1'b0;
end

wire [31:0] rdata_d =   buf_addr[15:0] == `FFT_CONFIG_ADDR                ? fft_config                  :
                        buf_addr[15:11] == 5'h02                          ? fft_input_rdata             :
                        buf_addr[15:11] == 5'h04                          ? fft_output_rdata            :
                        32'd0;

always@(posedge aclk) begin
    if(~aresetn) begin
        s_rdata  <= 'b0;
        s_rvalid <= 1'b0;
        s_rlast  <= 1'b0;
    end
    else if(busy & !write & !r_retire)
    begin
        s_rdata <= rdata_d;
        s_rvalid <= 1'b1;
        s_rlast <= 1'b1; 
    end
    else if(r_retire)
    begin
        s_rvalid <= 1'b0;
    end
end

always@(posedge aclk) begin
    if(~aresetn) s_bvalid <= 1'b0;
    else if(w_enter) s_bvalid <= 1'b1;
    else if(b_retire) s_bvalid <= 1'b0;
end

assign s_rid   = buf_id;
assign s_bid   = buf_id;
assign s_bresp = 2'b0;
assign s_rresp = 2'b0;


//-------------------------------{fft}begin----------------------------//
reg   stb;
reg   sop_in;
wire   [15:0]  x_re;
wire   [15:0]  x_im;
wire  valid_out;
wire  sop_out;
wire  [15:0]  y_re;
wire  [15:0]  y_im;
reg   [8:0] fft_count;
wire  [8:0] fft_countcmp;
reg   hw_read_fft_input;
reg  fft_out_finish;

wire write_fft_config   = w_enter & (buf_addr[15:0]==`FFT_CONFIG_ADDR);
wire write_fft_input   = w_enter & (buf_addr[15:11] == 5'h02);

always @(posedge aclk) begin
    if(~aresetn)
        fft_config <= 32'h0;
    else if (write_fft_config)
        fft_config <= {27'h0,s_wdata[4:0]};
    else if(fft_out_finish)
        fft_config <= {27'h0,1'b1,fft_config[3:0]};
    else
        fft_config <= {27'h0,fft_config[4:1],1'b0};
end

always @(posedge aclk or negedge aresetn) begin
    if(~aresetn)
        sop_in <= 1'b0;
    else
        sop_in <= fft_config[0];
end

assign fft_countcmp =   ({9{fft_config[3:2] == 2'b00}} & 9'h3f) |
                        ({9{fft_config[3:2] == 2'b01}} & 9'h7f) |
                        ({9{fft_config[3:2] == 2'b10}} & 9'hff) |
                        ({9{fft_config[3:2] == 2'b11}} & 9'h1ff);

always @(posedge aclk or negedge aresetn) begin
    if(~aresetn)
        hw_read_fft_input <= 1'b0;
    else if (write_fft_config & s_wdata[0])
        hw_read_fft_input <= 1'b1;
    else if(fft_count == fft_countcmp)
        hw_read_fft_input <= 1'b0;
end

always @(posedge aclk or negedge aresetn) begin
    if(~aresetn)
        stb <= 1'b0;
    else
        stb <= hw_read_fft_input;
end

always @(posedge aclk or negedge aresetn) begin
    if(~aresetn)
        fft_count <= 9'h0;
    else if(hw_read_fft_input|valid_out)
        fft_count <= fft_count + 1'b1;
    else if(fft_count== fft_countcmp)
        fft_count <= 9'h0;
end

always @(posedge aclk or negedge aresetn) begin
    if(~aresetn)
        fft_out_finish <= 1'b0;
    else if(valid_out &(fft_count== fft_countcmp))
        fft_out_finish <= 1'b1;
    else
        fft_out_finish <= 1'b0;
end

assign fft_finish = fft_config[4];
assign x_re = stb ? fft_input_rdata[31:16] : 16'h0;
assign x_im = stb ? fft_input_rdata[15:0] : 16'h0;

fft_multimode  u_fft_multimode (
    .clk                        ( aclk                        ),
    .rst_n                      ( aresetn                     ),

    .inv                        ( fft_config[1]               ),
    .np                         ( fft_config[3:2]             ),
    .stb                        ( stb                         ),
    .sop_in                     ( sop_in                      ),
    .x_re                       ( x_re                        ),
    .x_im                       ( x_im                        ),

    .valid_out                  ( valid_out                   ),
    .sop_out                    ( sop_out                     ),
    .y_re                       ( y_re                        ),
    .y_im                       ( y_im                        )
);

assign axi_read_fft_input = ar_enter & (s_araddr[15:11] == 5'h02);
assign fft_input_addr = axi_read_fft_input ?  s_araddr[10:2] :
                        hw_read_fft_input ? fft_count :
                        (buf_addr[15:11] == 5'h02) ? buf_addr[10:2] :
                        9'h0;
assign fft_input_cen = ~(hw_read_fft_input | write_fft_input | axi_read_fft_input);
assign fft_input_wen = ~write_fft_input;
assign fft_input_wdata = s_wdata;

assign axi_read_fft_output = ar_enter & (s_araddr[15:11] == 5'h04);                        
assign fft_output_addr =    axi_read_fft_output ? s_araddr[10:2] :
                            valid_out ? fft_count :
                            9'h0;
assign fft_output_cen = ~(axi_read_fft_output | valid_out) ;
assign fft_output_wen = ~valid_out;
assign fft_output_wdata = {y_re,y_im};


fpga_sram_sp_maskoff #(
    .DATA_WIDTH ( 32 ),
    .ADDR_WIDTH ( 9  ))
 sramsp32x512_fftinput (
    .CLK                     ( aclk             ),
    .GWEN                    ( fft_input_wen    ),
    .CEN                     ( fft_input_cen    ),
    .A                       ( fft_input_addr   ),
    .D                       ( fft_input_wdata  ),
    .Q                       ( fft_input_rdata  )
);

fpga_sram_sp_maskoff #(
    .DATA_WIDTH ( 32 ),
    .ADDR_WIDTH ( 9  ))
 sramsp32x512_fftoutput (
    .CLK                     ( aclk             ),
    .GWEN                    ( fft_output_wen   ),
    .CEN                     ( fft_output_cen   ),
    .A                       ( fft_output_addr  ),
    .D                       ( fft_output_wdata ),
    .Q                       ( fft_output_rdata )
);
//--------------------------------{fft}end-----------------------------//

endmodule
