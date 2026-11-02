module dma_top(

    input         aclk,
    input         aresetn,

    output        dma_finish,

    //slave port
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
    input           s_rready,

    //master port
    //ar
    output [3 :0] m_arid   ,
    output [31:0] m_araddr ,
    output [7 :0] m_arlen  ,
    output [2 :0] m_arsize ,
    output [1 :0] m_arburst,
    output        m_arlock ,
    output [3 :0] m_arcache,
    output [2 :0] m_arprot ,
    output        m_arvalid,
    input         m_arready,
    //r
    input  [3 :0] m_rid    ,
    input  [31:0] m_rdata  ,
    input  [1 :0] m_rresp  ,
    input         m_rlast  ,
    input         m_rvalid ,
    output        m_rready ,
    //aw
    output [3 :0] m_awid   ,
    output [31:0] m_awaddr ,
    output [7 :0] m_awlen  ,
    output [2 :0] m_awsize ,
    output [1 :0] m_awburst,
    output        m_awlock ,
    output [3 :0] m_awcache,
    output [2 :0] m_awprot ,
    output        m_awvalid,
    input         m_awready,
    //w
    output [31:0] m_wdata  ,
    output [3 :0] m_wstrb  ,
    output        m_wlast  ,
    output        m_wvalid ,
    input         m_wready ,
    //b
    input  [3 :0] m_bid    ,
    input  [1 :0] m_bresp  ,
    input         m_bvalid ,
    output        m_bready  

);

//-------------------------------{slave reg port}begin----------------------------//
    reg [31:0] CTRL,SADDR,DADDR,LENGTH;

    reg busy,write,R_or_W;
    reg s_wready;

    wire ar_enter = s_arvalid & s_arready;
    wire r_retire = s_rvalid & s_rready & s_rlast;
    wire aw_enter = s_awvalid & s_awready;
    wire w_enter  = s_wvalid & s_wready & s_wlast;
    wire b_retire = s_bvalid & s_bready;

    assign s_arready = ~busy & (!R_or_W| !s_awvalid);
    assign s_awready = ~busy & ( R_or_W| !s_arvalid);

    always@(posedge aclk)
        if(~aresetn) busy <= 1'b0;
        else if(ar_enter|aw_enter) busy <= 1'b1;
        else if(r_retire|b_retire) busy <= 1'b0;

    reg [3 :0] buf_id;
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

    always@(posedge aclk)
        if(~aresetn) write <= 1'b0;
        else if(aw_enter) write <= 1'b1;
        else if(ar_enter)  write <= 1'b0;

    always@(posedge aclk)
        if(~aresetn) s_wready <= 1'b0;
        else if(aw_enter) s_wready <= 1'b1;
        else if(w_enter & s_wlast) s_wready <= 1'b0;


    reg [31:0] s_rdata;
    reg s_rvalid,s_rlast;
    wire [31:0] rdata_d =   buf_addr[15:0] ==  16'h0     ? CTRL         : 
                            buf_addr[15:0] ==  16'h4     ? SADDR        : 
                            buf_addr[15:0] ==  16'h8     ? DADDR        : 
                            buf_addr[15:0] ==  16'hc     ? LENGTH       : 
                            32'd0;

    always@(posedge aclk)begin
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

    reg s_bvalid;
    always@(posedge aclk)   begin
        if(~aresetn) s_bvalid <= 1'b0;
        else if(w_enter) s_bvalid <= 1'b1;
        else if(b_retire) s_bvalid <= 1'b0;
    end
    assign s_rid   = buf_id;
    assign s_bid   = buf_id;
    assign s_bresp = 2'b0;
    assign s_rresp = 2'b0;


    wire write_reg_en[3:0];
    assign write_reg_en[0] = w_enter & (buf_addr[15:0]==16'h0);
    assign write_reg_en[1] = w_enter & (buf_addr[15:0]==16'h4);
    assign write_reg_en[2] = w_enter & (buf_addr[15:0]==16'h8);
    assign write_reg_en[3] = w_enter & (buf_addr[15:0]==16'hc);

    always @(posedge aclk) begin
        if(!aresetn) begin
            CTRL <= 32'h0;
        end
        else if (write_reg_en[0]) begin
            CTRL <= {29'h0,CTRL[2],s_wdata[1:0]};
        end
        else if(dma_finish) begin
            CTRL <= {29'h0,1'h1,CTRL[1:0]};
        end
        else begin
            CTRL <= {29'h0,CTRL[2],2'h0};
        end
    end

    always @(posedge aclk) begin
        if(!aresetn) begin
            SADDR <= 32'h0;
        end
        else if (write_reg_en[1]) begin
            SADDR <= {s_wdata[31:2],2'h0};
        end
    end

    always @(posedge aclk) begin
        if(!aresetn) begin
            DADDR <= 32'h0;
        end
        else if (write_reg_en[2]) begin
            DADDR <= {s_wdata[31:2],2'h0};
        end
    end

    always @(posedge aclk) begin
        if(!aresetn) begin
            LENGTH <= 32'h0;
        end
        else if (write_reg_en[3]) begin
            LENGTH <= s_wdata;
        end
    end
//-------------------------------{slave reg port}end----------------------------//

//---------------------------------{dma master}begin----------------------------//

dma_master  u_dma_master (
    .clk                     ( aclk           ),
    .resetn                  ( aresetn        ),

    .dma_en                  ( CTRL[0]       ),
    .dma_int_clr             ( CTRL[1]       ),
    .saddr                   ( SADDR         ),
    .daddr                   ( DADDR         ),
    .length                  ( LENGTH        ),
    .dma_finish              ( dma_finish    ),

    .arready                 ( m_arready       ),
    .rid                     ( m_rid           ),
    .rdata                   ( m_rdata         ),
    .rresp                   ( m_rresp         ),
    .rlast                   ( m_rlast         ),
    .rvalid                  ( m_rvalid        ),
    .awready                 ( m_awready       ),
    .wready                  ( m_wready        ),
    .bid                     ( m_bid           ),
    .bresp                   ( m_bresp         ),
    .bvalid                  ( m_bvalid        ),

    .arid                    ( m_arid          ),
    .araddr                  ( m_araddr        ),
    .arlen                   ( m_arlen         ),
    .arsize                  ( m_arsize        ),
    .arburst                 ( m_arburst       ),
    .arlock                  ( m_arlock        ),
    .arcache                 ( m_arcache       ),
    .arprot                  ( m_arprot        ),
    .arvalid                 ( m_arvalid       ),
    .rready                  ( m_rready        ),
    .awid                    ( m_awid          ),
    .awaddr                  ( m_awaddr        ),
    .awlen                   ( m_awlen         ),
    .awsize                  ( m_awsize        ),
    .awburst                 ( m_awburst       ),
    .awlock                  ( m_awlock        ),
    .awcache                 ( m_awcache       ),
    .awprot                  ( m_awprot        ),
    .awvalid                 ( m_awvalid       ),
    .wid                     (                 ),
    .wdata                   ( m_wdata         ),
    .wstrb                   ( m_wstrb         ),
    .wlast                   ( m_wlast         ),
    .wvalid                  ( m_wvalid        ),
    .bready                  ( m_bready        )
);


//---------------------------------{dma master}end------------------------------//

endmodule