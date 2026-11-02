module dma_master(
    input clk,
    input resetn,

    input dma_en,
    input dma_int_clr,
    input [31:0] saddr,
    input [31:0] daddr,
    input [31:0] length,
    output dma_finish,

    //AXI interface 
    //Read address channel
    output   [ 3:0] arid,
    output   [31:0] araddr,
    output   [ 7:0] arlen,
    output   [ 2:0] arsize,
    output   [ 1:0] arburst,
    output          arlock,
    output   [ 3:0] arcache,
    output   [ 2:0] arprot,
    output          arvalid,
    input           arready,
    //Read data channel
    input    [ 3:0] rid,
    input    [31:0] rdata,
    input    [ 1:0] rresp,
    input           rlast,
    input           rvalid,
    output          rready,
    //Write address channel
    output   [ 3:0] awid,
    output   [31:0] awaddr,
    output   [ 7:0] awlen,
    output   [ 2:0] awsize,
    output   [ 1:0] awburst,
    output          awlock,
    output   [ 3:0] awcache,
    output   [ 2:0] awprot,
    output          awvalid,
    input           awready,
    //Write data channel
    output   [ 3:0] wid,
    output   [31:0] wdata,
    output   [ 3:0] wstrb,
    output          wlast,
    output          wvalid,
    input           wready,
    //Write response channel
    input    [ 3:0] bid,
    input    [ 1:0] bresp,
    input           bvalid,
    output          bready

);

    reg         mem_req;
    reg         mem_we;
    reg[31:0]   mem_addr;
    reg[31:0]   mem_wdata;

    // 状态
    localparam  S_IDLE                      = 5'd0;
    localparam  S_INIT                      = 5'd1;
    localparam  S_READ                      = 5'd2;
    localparam  S_WRITE                     = 5'd3;
    localparam  S_JUDGE                     = 5'd4;
    
    reg[2:0]    state;
    reg[2:0]    next_state;

    // 存放读数据
    reg[31:0]   data_buffer[31:0];//一组32个字
    reg[4:0]    words_index;//组内的读写指针
    wire[5:0]   need_to_trans_words;//组内字数
    reg[4:0]    last_packet_words;//最后一组的字数
    reg[31:0]   remain_packet_count;//剩余组数
    reg[31:0]   all_packet_count;//总组数
    reg[31:0]   trans_mem_addr;
    reg[31:0]   trans_mem_wdata;

    wire        store_finish;
    wire        load_finish;

    /***********************BEGIN of FSM********************************/
    always @(posedge clk or negedge resetn) begin
        if(~resetn)
            state <= S_IDLE;
        else begin
            state <= next_state;
        end
    end

    //状态跳转控制
    always @(*) begin
        case(state)
            S_IDLE : begin
                if(dma_en && (length != 32'h0))
                    next_state = S_READ;
                else
                    next_state = S_IDLE;
            end
            S_READ : begin
                if(load_finish && (words_index == (need_to_trans_words - 6'h1)))
                    next_state = S_WRITE;
                else
                    next_state = S_READ;
            end
            S_WRITE : begin
                if (store_finish && (words_index == (need_to_trans_words - 6'h1)))
                    next_state = S_JUDGE;
                else
                    next_state = S_WRITE;
            end
            S_JUDGE : begin
                if(remain_packet_count == 32'h0)
                    next_state = S_IDLE;
                else
                    next_state = S_READ;
            end
            default     :begin
                next_state = S_IDLE;
            end
        endcase
    end

    always @(*) begin
        case(state)
            S_IDLE : begin
                mem_req = 1'h0;
                mem_we = 1'h0;
                mem_addr = 32'h0;
                mem_wdata = 32'h0;
            end
            S_READ : begin
                mem_req = 1'h1;
                mem_we = 1'h0;
                mem_addr = trans_mem_addr;
                mem_wdata = 32'h0;
            end
            S_WRITE : begin
                mem_req = 1'h1;
                mem_we = 1'h1;
                mem_addr = trans_mem_addr;
                mem_wdata = trans_mem_wdata;
            end
            default : begin
                mem_req = 1'h0;
                mem_we = 1'h0;
                mem_addr = 32'h0;
                mem_wdata = 32'h0;
            end
        endcase
    end

    wire length_add = |(length[4:0]);
    assign need_to_trans_words = ((remain_packet_count == 32'h1)&length_add) ? last_packet_words : 32'd32;

    always @ (posedge clk or negedge resetn) begin
        if (~resetn) begin
            remain_packet_count <= 32'h0;
        end else begin
            case (state)
                S_IDLE: begin
                    if(dma_en && (length != 32'h0))
                        remain_packet_count <= (length >> 5) + length_add;
                    else
                        remain_packet_count <= 32'h0;
                end
                S_WRITE: begin
                    if(store_finish && (words_index == (need_to_trans_words - 6'h1)))
                        remain_packet_count <= remain_packet_count - 32'h1;
                end
                default: begin
                    remain_packet_count <= remain_packet_count;
                end
            endcase
        end
    end

    always @ (posedge clk or negedge resetn) begin
        if (~resetn) begin
            all_packet_count <= 32'h0;
        end else begin
           if((state == S_IDLE) && dma_en && (length != 32'h0))
                all_packet_count <= (length >> 5) + length_add;
        end
    end

    always @ (posedge clk or negedge resetn) begin
        if (~resetn) begin
            last_packet_words <= 5'h0;
        end else begin
           if((state == S_IDLE) && dma_en && (length[4:0] != 5'h0))
                last_packet_words <= length[4:0];
        end
    end

    always @ (posedge clk or negedge resetn) begin
        if (~resetn) begin
            words_index <= 5'h0;
        end else begin
           if((state == S_READ) || (state == S_WRITE))begin
                if(store_finish || load_finish) begin
                    if(words_index == (need_to_trans_words - 6'h1))
                        words_index <= 5'h0;
                    else
                        words_index <= words_index + 5'h1;
                end
           end
           else begin
                words_index <= 5'h0;
           end
        end
    end

    always @ (posedge clk ) begin
        if((state == S_READ) && load_finish)
            data_buffer[words_index] <= rdata;
    end


    always @ (posedge clk or negedge resetn) begin
        if (~resetn) begin
            trans_mem_addr <= 32'h0;
        end else begin
            case (state)
                S_IDLE: begin
                    if(dma_en && (length != 32'h0))
                        trans_mem_addr <= saddr;
                end
                S_READ: begin
                    if(load_finish) begin
                        if(words_index == (need_to_trans_words - 6'h1))
                            trans_mem_addr <= daddr + ((all_packet_count - remain_packet_count)<<7);
                        else
                            trans_mem_addr <= trans_mem_addr + 32'h4;
                    end
                end
                S_WRITE: begin
                    if(store_finish) begin
                        trans_mem_addr <= trans_mem_addr + 32'h4;
                    end
                end
                S_JUDGE: begin
                    if(remain_packet_count != 32'h0)
                        trans_mem_addr <= saddr + ((all_packet_count - remain_packet_count)<<7);
                end
            endcase
        end
    end

    always @ (posedge clk or negedge resetn) begin
        if (~resetn) begin
            trans_mem_wdata <= 32'h0;
        end else begin
            if(state == S_WRITE)
                trans_mem_wdata <= data_buffer[words_index];
        end
    end

    reg dma_finish_reg;
    assign dma_finish = dma_finish_reg;

    always @ (posedge clk or negedge resetn) begin
        if (~resetn) begin
            dma_finish_reg <= 1'b0;
        end else begin
            if((state == S_JUDGE) && (remain_packet_count == 0)) begin
                dma_finish_reg <= 1'b1;
            end
            else if(dma_int_clr) begin
                dma_finish_reg <= 1'b0;
            end
        end
    end

memreq_axi  u_memreq_axi (
    .clk                     ( clk                ),
    .rst_n                   ( resetn             ),
    .mem_req                 ( mem_req            ),
    .mem_we                  ( mem_we             ),
    .mem_addr                ( mem_addr           ),
    .mem_wdata               ( mem_wdata          ),
    .mem_stb                 ( 1'h0               ),
    .arready                 ( arready            ),
    .rid                     ( rid                ),
    .rdata                   ( rdata              ),
    .rresp                   ( rresp              ),
    .rlast                   ( rlast              ),
    .rvalid                  ( rvalid             ),
    .awready                 ( awready            ),
    .wready                  ( wready             ),
    .bid                     ( bid                ),
    .bresp                   ( bresp              ),
    .bvalid                  ( bvalid             ),

    .store_finish            ( store_finish       ),
    .load_finish             ( load_finish        ),
    .arid                    ( arid               ),
    .araddr                  ( araddr             ),
    .arlen                   ( arlen              ),
    .arsize                  ( arsize             ),
    .arburst                 ( arburst            ),
    .arlock                  ( arlock             ),
    .arcache                 ( arcache            ),
    .arprot                  ( arprot             ),
    .arvalid                 ( arvalid            ),
    .rready                  ( rready             ),
    .awid                    ( awid               ),
    .awaddr                  ( awaddr             ),
    .awlen                   ( awlen              ),
    .awsize                  ( awsize             ),
    .awburst                 ( awburst            ),
    .awlock                  ( awlock             ),
    .awcache                 ( awcache            ),
    .awprot                  ( awprot             ),
    .awvalid                 ( awvalid            ),
    .wid                     ( wid                ),
    .wdata                   ( wdata              ),
    .wstrb                   ( wstrb              ),
    .wlast                   ( wlast              ),
    .wvalid                  ( wvalid             ),
    .bready                  ( bready             )
);

endmodule