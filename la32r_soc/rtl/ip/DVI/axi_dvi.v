module axi_dvi #
(
    parameter WIDTH = 12,    // hdata and vdata width (in bits)
    parameter HSIZE = 800,   // Horizontal size of visible area
    parameter HFP = 856,     // Horizontal front porch
    parameter HSP = 976,     // Horizontal sync pulse
    parameter HMAX = 1040,   // Horizontal total size
    parameter VSIZE = 600,   // Vertical size of visible area
    parameter VFP = 637,     // Vertical front porch
    parameter VSP = 643,     // Vertical sync pulse
    parameter VMAX = 666,    // Vertical total size
    parameter HSPP = 1,      // Horizontal sync pulse polarity (1 for positive)
    parameter VSPP = 1       // Vertical sync pulse polarity (1 for positive)
)
(
	input            s_awvalid,
	output           s_awready,
	input   [31:0]   s_awaddr,
	input   [4:0]    s_awid,
	input   [7:0]    s_awlen,
	input   [2:0]    s_awsize,
	input   [1:0]    s_awburst,
	input   [0:0]    s_awlock,
	input   [3:0]    s_awcache,
	input   [2:0]    s_awprot,
	input            s_wvalid,
	output           s_wready,
	input   [31:0]   s_wdata,
	input   [3:0]    s_wstrb,
	input            s_wlast,
	output           s_bvalid,
	input            s_bready,
	output  [4:0]    s_bid,
	output  [1:0]    s_bresp,
	input            s_arvalid,
	output           s_arready,
	input   [31:0]   s_araddr,
	input   [4:0]    s_arid,
	input   [7:0]    s_arlen,
	input   [2:0]    s_arsize,
	input   [1:0]    s_arburst,
	input   [0:0]    s_arlock,
	input   [3:0]    s_arcache,
	input   [2:0]    s_arprot,
	output           s_rvalid,
	input            s_rready,
	output  [31:0]   s_rdata,
	output  [4:0]    s_rid,
	output  [1:0]    s_rresp,
	output           s_rlast,
	
	output  video_clk,           // Video clock signal
    output  hsync,               // Horizontal sync signal
    output  vsync,               // Vertical sync signal
    output  data_enable,         // Data enable signal
    output  [2:0] video_red,      // Red color signal (3 bits)
    output  [2:0] video_green,    // Green color signal (3 bits)
    output  [1:0] video_blue,      // Blue color signal (2 bits)

	input            aclk,
	input            aresetn
);

    reg [31:0] DVI_RECT_DIR,DVI_RECT_L_W,DVI_SQU_DIR,DVI_SQU_R;

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
    wire [31:0] rdata_d =   buf_addr[15:0] ==  16'h0     ? DVI_RECT_DIR         : 
                            buf_addr[15:0] ==  16'h4     ? DVI_RECT_L_W         : 
                            buf_addr[15:0] ==  16'h8     ? DVI_SQU_DIR          : 
                            buf_addr[15:0] ==  16'hc     ? DVI_SQU_R            : 
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


//-------------------------------{dvi controller}begin----------------------------//
    wire write_reg_en[3:0];
    assign write_reg_en[0] = w_enter & (buf_addr[15:0]==16'h0);
    assign write_reg_en[1] = w_enter & (buf_addr[15:0]==16'h4);
    assign write_reg_en[2] = w_enter & (buf_addr[15:0]==16'h8);
    assign write_reg_en[3] = w_enter & (buf_addr[15:0]==16'hc);

    always @(posedge aclk) begin
        if(!aresetn) begin
            DVI_RECT_DIR <= 32'h0;
        end
        else if (write_reg_en[0]) begin
            DVI_RECT_DIR <= s_wdata;
        end
    end

    always @(posedge aclk) begin
        if(!aresetn) begin
            DVI_RECT_L_W <= 32'h0;
        end
        else if (write_reg_en[1]) begin
            DVI_RECT_L_W <= s_wdata;
        end
    end

    always @(posedge aclk) begin
        if(!aresetn) begin
            DVI_SQU_DIR <= 32'h0;
        end
        else if (write_reg_en[2]) begin
            DVI_SQU_DIR <= s_wdata;
        end
    end

    always @(posedge aclk) begin
        if(!aresetn) begin
            DVI_SQU_R <= 32'h0;
        end
        else if (write_reg_en[3]) begin
            DVI_SQU_R <= s_wdata;
        end
    end

    reg [WIDTH-1:0] hdata = 0;       // Horizontal position counter
	reg [WIDTH-1:0] vdata = 0;       // Vertical position counter
	
    wire hdata_in_range;
    wire vdata_in_range;
    
    wire hdata1_in_range;
    wire vdata1_in_range;
	
    always @(posedge aclk) begin
        if (hdata == (HMAX - 1))      // If horizontal counter reaches max
            hdata <= 0;               // Reset horizontal counter
        else
            hdata <= hdata + 1;       // Increment horizontal counter
    end

    // Vertical counter (vdata) logic
    always @(posedge aclk) begin
        if (hdata == (HMAX - 1)) begin
            if (vdata == (VMAX - 1))  // If vertical counter reaches max
                vdata <= 0;           // Reset vertical counter
            else
                vdata <= vdata + 1;   // Increment vertical counter
        end
    end

    // Horizontal sync signal generation (hsync)
    assign video_clk = aclk; // Example: using input clock as video clock
    assign hsync = ((hdata >= HFP) && (hdata < HSP)) ? HSPP : !HSPP;
    assign vsync = ((vdata >= VFP) && (vdata < VSP)) ? VSPP : !VSPP;
    assign data_enable = ((hdata < HSIZE) & (vdata < VSIZE));  
    
    assign hdata_in_range = (hdata > (DVI_RECT_DIR[31:16]-DVI_RECT_L_W[31:16])) && (hdata < (DVI_RECT_DIR[31:16]+DVI_RECT_L_W[31:16]));
    // Check if vdata is in the range specified by DVI_RECT_L_W
    assign vdata_in_range = (vdata > DVI_RECT_DIR[15:0]) && (vdata <(DVI_RECT_DIR[15:0]+DVI_RECT_L_W[15:0]));
    
    assign hdata1_in_range = (hdata > (DVI_SQU_DIR[31:16]-DVI_SQU_R[31:16])) && (hdata < (DVI_SQU_DIR[31:16]+DVI_SQU_R[31:16]));
    // Check if vdata is in the range specified by DVI_RECT_L_W
    assign vdata1_in_range = (vdata > (DVI_SQU_DIR[15:0]-DVI_SQU_R[15:0])) && (vdata <(DVI_SQU_DIR[15:0]+DVI_SQU_R[15:0]));
    
    // Set video output colors based on conditions
    assign video_red = ((hdata_in_range && vdata_in_range)||(hdata1_in_range && vdata1_in_range)) ? 3'b111 : 3'b0;
    assign video_green = 3'b0;
    assign video_blue = 2'b0;

//--------------------------------{dvi controller}end-----------------------------//


endmodule