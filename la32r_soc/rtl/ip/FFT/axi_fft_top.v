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

module axi_fft_top (
    input           aclk,
    input           aresetn,
    input           fft_clk,
    input           fft_resetn,

    output          fft_finish,//电平中断

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
    output          s_wready,
    output [4 :0]   s_bid,
    output [1 :0]   s_bresp,
    output          s_bvalid,
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
    output [31:0]   s_rdata,
    output [1 :0]   s_rresp,
    output          s_rlast,
    output          s_rvalid,
    input           s_rready
);

//fft axi
//ar
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
//r
wire [4 :0] fft_rid    ;
wire [31:0] fft_rdata  ;
wire [1 :0] fft_rresp  ;
wire        fft_rlast  ;
wire        fft_rvalid ;
wire        fft_rready ;
//aw
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
//w
wire [31:0] fft_wdata  ;
wire [3 :0] fft_wstrb  ;
wire        fft_wlast  ;
wire        fft_wvalid ;
wire        fft_wready ;
//b
wire [4 :0] fft_bid    ;
wire [1 :0] fft_bresp  ;
wire        fft_bvalid ;
wire        fft_bready ;

wire        fft_finish_tmp;

Axi_CDC  u_Axi_CDC_fft (
    .axiInClk                ( aclk                  ),
    .axiInRstn               ( aresetn               ),
    .axiOutClk               ( fft_clk               ),
    .axiOutRstn              ( fft_resetn            ),

    .axiIn_awvalid           ( s_awvalid             ),
    .axiIn_awaddr            ( s_awaddr              ),
    .axiIn_awid              ( s_awid                ),
    .axiIn_awlen             ( s_awlen               ),
    .axiIn_awsize            ( s_awsize              ),
    .axiIn_awburst           ( s_awburst             ),
    .axiIn_awlock            ( s_awlock              ),
    .axiIn_awcache           ( s_awcache             ),
    .axiIn_awprot            ( s_awprot              ),
    .axiIn_wvalid            ( s_wvalid              ),
    .axiIn_wdata             ( s_wdata               ),
    .axiIn_wstrb             ( s_wstrb               ),
    .axiIn_wlast             ( s_wlast               ),
    .axiIn_bready            ( s_bready              ),
    .axiIn_arvalid           ( s_arvalid             ),
    .axiIn_araddr            ( s_araddr              ),
    .axiIn_arid              ( s_arid                ),
    .axiIn_arlen             ( s_arlen               ),
    .axiIn_arsize            ( s_arsize              ),
    .axiIn_arburst           ( s_arburst             ),
    .axiIn_arlock            ( s_arlock              ),
    .axiIn_arcache           ( s_arcache             ),
    .axiIn_arprot            ( s_arprot              ),
    .axiIn_rready            ( s_rready              ),
    .axiOut_awready          ( fft_awready           ),
    .axiOut_wready           ( fft_wready            ),
    .axiOut_bvalid           ( fft_bvalid            ),
    .axiOut_bid              ( fft_bid               ),
    .axiOut_bresp            ( fft_bresp             ),
    .axiOut_arready          ( fft_arready           ),
    .axiOut_rvalid           ( fft_rvalid            ),
    .axiOut_rdata            ( fft_rdata             ),
    .axiOut_rid              ( fft_rid               ),
    .axiOut_rresp            ( fft_rresp             ),
    .axiOut_rlast            ( fft_rlast             ),

    .axiIn_awready           ( s_awready             ),
    .axiIn_wready            ( s_wready              ),
    .axiIn_bvalid            ( s_bvalid              ),
    .axiIn_bid               ( s_bid                 ),
    .axiIn_bresp             ( s_bresp               ),
    .axiIn_arready           ( s_arready             ),
    .axiIn_rvalid            ( s_rvalid              ),
    .axiIn_rdata             ( s_rdata               ),
    .axiIn_rid               ( s_rid                 ),
    .axiIn_rresp             ( s_rresp               ),
    .axiIn_rlast             ( s_rlast               ),
    .axiOut_awvalid          ( fft_awvalid           ),
    .axiOut_awaddr           ( fft_awaddr            ),
    .axiOut_awid             ( fft_awid              ),
    .axiOut_awlen            ( fft_awlen             ),
    .axiOut_awsize           ( fft_awsize            ),
    .axiOut_awburst          ( fft_awburst           ),
    .axiOut_awlock           ( fft_awlock            ),
    .axiOut_awcache          ( fft_awcache           ),
    .axiOut_awprot           ( fft_awprot            ),
    .axiOut_wvalid           ( fft_wvalid            ),
    .axiOut_wdata            ( fft_wdata             ),
    .axiOut_wstrb            ( fft_wstrb             ),
    .axiOut_wlast            ( fft_wlast             ),
    .axiOut_bready           ( fft_bready            ),
    .axiOut_arvalid          ( fft_arvalid           ),
    .axiOut_araddr           ( fft_araddr            ),
    .axiOut_arid             ( fft_arid              ),
    .axiOut_arlen            ( fft_arlen             ),
    .axiOut_arsize           ( fft_arsize            ),
    .axiOut_arburst          ( fft_arburst           ),
    .axiOut_arlock           ( fft_arlock            ),
    .axiOut_arcache          ( fft_arcache           ),
    .axiOut_arprot           ( fft_arprot            ),
    .axiOut_rready           ( fft_rready            )
);

axi_fft_wrap  u_axi_fft_wrap (
    .aclk                    ( fft_clk          ),
    .aresetn                 ( fft_resetn       ),
    .s_awid                  ( fft_awid         ),
    .s_awaddr                ( fft_awaddr       ),
    .s_awlen                 ( fft_awlen        ),
    .s_awsize                ( fft_awsize       ),
    .s_awburst               ( fft_awburst      ),
    .s_awlock                ( fft_awlock       ),
    .s_awcache               ( fft_awcache      ),
    .s_awprot                ( fft_awprot       ),
    .s_awvalid               ( fft_awvalid      ),
    .s_wid                   ( fft_wid          ),
    .s_wdata                 ( fft_wdata        ),
    .s_wstrb                 ( fft_wstrb        ),
    .s_wlast                 ( fft_wlast        ),
    .s_wvalid                ( fft_wvalid       ),
    .s_bready                ( fft_bready       ),
    .s_arid                  ( fft_arid         ),
    .s_araddr                ( fft_araddr       ),
    .s_arlen                 ( fft_arlen        ),
    .s_arsize                ( fft_arsize       ),
    .s_arburst               ( fft_arburst      ),
    .s_arlock                ( fft_arlock       ),
    .s_arcache               ( fft_arcache      ),
    .s_arprot                ( fft_arprot       ),
    .s_arvalid               ( fft_arvalid      ),
    .s_rready                ( fft_rready       ),

    .fft_finish              ( fft_finish_tmp   ),
    .s_awready               ( fft_awready      ),
    .s_wready                ( fft_wready       ),
    .s_bid                   ( fft_bid          ),
    .s_bresp                 ( fft_bresp        ),
    .s_bvalid                ( fft_bvalid       ),
    .s_arready               ( fft_arready      ),
    .s_rid                   ( fft_rid          ),
    .s_rdata                 ( fft_rdata        ),
    .s_rresp                 ( fft_rresp        ),
    .s_rlast                 ( fft_rlast        ),
    .s_rvalid                ( fft_rvalid       )
);

reg fft_finish_tmp_r;
always @(posedge fft_clk or negedge fft_resetn) begin
    if(~fft_resetn) begin
        fft_finish_tmp_r <= 1'b0;
    end
    else begin
        fft_finish_tmp_r <= fft_finish_tmp;
    end
end

(* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0] fft_finish_r;
always @(posedge aclk or negedge aresetn) begin
    if(~aresetn) begin
        fft_finish_r <= 2'b0;
    end
    else begin
        fft_finish_r <= {fft_finish_r[0],fft_finish_tmp_r};
    end
end
assign fft_finish = fft_finish_r[1];

endmodule
