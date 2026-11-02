`include "CNNConfig.vh"
`include "mycpu.h"

module CNNAccelerator(
    input                       clk,
    input                       rst,


    input                       lacc_flush,

    input                       lacc_req_valid,
    input [`LACC_OP_WIDTH-1: 0] lacc_req_command,
    input [6: 0]                lacc_req_imm,
    input [31: 0]               lacc_req_rj,
    input [31: 0]               lacc_req_rk,

    output                      lacc_rsp_valid,
    output [31: 0]              lacc_rsp_rdat,

    // wreq will also send valid sign
    output                      lacc_data_valid,
    input                       lacc_data_ready,
    output [31: 0]              lacc_data_addr,
    output                      lacc_data_read,
    output [31: 0]              lacc_data_wdata,
    output [1: 0]               lacc_data_size,

    input                       lacc_drsp_valid,
    input [31: 0]               lacc_drsp_rdata
);
    wire buffer_cmd_valid;
    wire buffer_cmd_ready;
    wire buffer_drsp_valid;
    wire buffer_req;
    wire [31: 0] buffer_cmd_addr;
    wire [`WINDOW_SIZE*32-1: 0] window;
    wire window_valid;
    reg data_req_buf;
    reg [`KERNEL_WIDTH-1: 0]            conf_kernel_width;
    reg [`KERNEL_WIDTH-1: 0]            conf_kernel_height;
    reg [$clog2(`BUFFER_DEPTH)-1: 0]    conf_buf_depth;
    reg [$clog2(`BUFFER_WIDTH)-1: 0]    conf_buf_width;
    reg [`STRIDE_WIDTH-1: 0]            conf_stride;
    reg [$clog2(`WEIGHT_SIZE)-1: 0]     conf_weight_num;
    reg [`KERNEL_WIDTH-1: 0]            conf_padding;
    reg [3: 0]                          conf_padding_valid;
    reg [1: 0]                          conf_weight_size;
    reg [1: 0]                          conf_buf_size;
    reg                                 conf_add_write;
    reg [15: 0]                         conf_buf_offset;
    reg [15: 0]                         conf_res_offset;
    reg [$clog2(`BUFFER_WIDTH)-1: 0]    conf_res_width;
    reg                                 conf_buf_refresh;
    wire [`KERNEL_SIZE: 0] kernel_width_vec, kernel_height_vec;
    reg conf_refresh;
    reg buf_offset_valid, res_offset_valid;
    reg conv_op;
    reg pool_op;
    reg act_op;
    wire conv_empty;
    wire pool_empty;
    wire [1: 0] lacc_data_source;
    wire        lacc_drsp_read;
    reg weight_cmd_end;
    genvar i;
// decode
    wire cmd_conf_buf;
    wire cmd_conf_res;
    wire cmd_conv;
    wire cmd_conf_offset;
    assign cmd_conf_buf      = lacc_req_command == 0;
    assign cmd_conf_res      = lacc_req_command == 1;
    assign cmd_conv          = lacc_req_command == 2;
    assign cmd_conf_offset   = lacc_req_command == 3;

    reg [31: 0] conf_res_addr [`WEIGHT_SIZE-1: 0];
    reg [31: 0] conf_bias [`WEIGHT_SIZE-1: 0];
    wire [`WEIGHT_SIZE*32-1: 0] conf_bias_w;

    
    wire conf_buf_valid = lacc_req_valid & cmd_conf_buf;

    always @(posedge clk)begin
        conf_refresh <= conf_buf_valid;
        if(rst)begin
            conf_add_write <= 0;
        end
        else if(conf_buf_valid)begin
            conf_add_write      <= lacc_req_rk[1];
        end
        if(conf_buf_valid)begin
            conf_buf_refresh    <= lacc_req_rk[0];
            conf_padding_valid  <= lacc_req_rk[2 +: 4];
            conf_weight_size    <= lacc_req_rk[6 +: 2];
            conf_buf_size       <= lacc_req_rk[8 +: 2];
            conf_kernel_width   <= lacc_req_rk[10 +: `KERNEL_WIDTH];
            conf_kernel_height  <= lacc_req_rk[13 +: `KERNEL_WIDTH];
            conf_buf_width      <= lacc_req_rk[16 +: $clog2(`BUFFER_WIDTH)];
            conf_buf_depth      <= lacc_req_rk[21 +: $clog2(`BUFFER_DEPTH)];
            conf_stride         <= lacc_req_rk[25 +: `STRIDE_WIDTH];
            conf_weight_num     <= lacc_req_rk[27 +: $clog2(`WEIGHT_SIZE)];
            conf_padding        <= lacc_req_rk[30 +: `KERNEL_WIDTH];

        end
        if(lacc_req_valid & cmd_conf_offset)begin
            conf_buf_offset     <= lacc_req_rj[15: 0];
            conf_res_offset     <= lacc_req_rk[15: 0];
            conf_res_width      <= lacc_req_rk[16 +: $clog2(`BUFFER_WIDTH)];
        end
        if(rst)begin
            buf_offset_valid <= 1'b0;
            res_offset_valid <= 1'b0;
        end
        else if(lacc_req_valid & cmd_conf_offset)begin
            buf_offset_valid <= |lacc_req_rj[15: 0];
            res_offset_valid <= |lacc_req_rk[15: 0];
        end
    end
    Decoder #(`KERNEL_SIZE+1) decoder_kernel_width(conf_kernel_width, kernel_width_vec);
    Decoder #(`KERNEL_SIZE+1) decoder_kernel_height(conf_kernel_height, kernel_height_vec);

// state
    parameter FSM_WIDTH = 2;
    parameter IDLE      = 'b0;
    parameter WEIGHT    = 'b1;
    parameter CONV      = 'd2;
    reg [FSM_WIDTH-1: 0] state_r;
    wire [FSM_WIDTH-1: 0] nxt_state;
    wire [FSM_WIDTH-1: 0] idle_state_n;
    wire state_en;
    wire idle_exit;
    wire weight_exit;
    wire conv_exit;
    wire window_finish;
    wire window_stall;
    wire res_buf_empty;

    wire state_idle     = state_r == IDLE;
    wire state_weight   = state_r == WEIGHT;
    wire state_conv     = state_r == CONV;

    assign state_en = idle_exit | weight_exit | conv_exit;
    assign idle_exit = state_idle & lacc_req_valid & cmd_conv;
    assign weight_exit = state_weight & weight_cmd_end & lacc_drsp_valid;
    assign conv_exit = state_conv & window_finish & res_buf_empty &
                       (conv_op & conv_empty | pool_op & pool_empty);
    assign idle_state_n = lacc_req_imm[0] ? WEIGHT : CONV;
    assign nxt_state = {FSM_WIDTH{idle_exit}} & idle_state_n |
                       {FSM_WIDTH{weight_exit}} & CONV |
                       {FSM_WIDTH{conv_exit}} & IDLE;

    always @(posedge clk)begin
        if(rst | lacc_flush)begin
            state_r <= IDLE;
        end
        else begin
            if(state_en) state_r <= nxt_state;
        end
    end

// weight control
    reg [31:0] weight_buf_r [`WEIGHT_SIZE*`WINDOW_SIZE-1: 0];
    wire [`WEIGHT_SIZE*`WINDOW_SIZE*32-1: 0] weight_buf_w;
    reg [$clog2(`WEIGHT_SIZE)-1: 0] weight_size, req_weight_size;
    reg [`KERNEL_WIDTH-1: 0] weight_row_idx, weight_col_idx;
    reg [`KERNEL_WIDTH-1: 0] req_weight_row_idx, req_weight_col_idx;
    reg [$clog2(`WEIGHT_SIZE*`WINDOW_SIZE)-1:0] weight_idx, weight_line_idx, weight_page_idx;
    wire [$clog2(`WEIGHT_SIZE*`WINDOW_SIZE)-1:0] weight_line_idx_n, weight_page_idx_n;

    reg [31: 0] weight_addr;
    reg [1: 0] weight_offset;
    wire [2: 0] weight_addr_cin;
    wire [31: 0] weight_data_in;
    wire weight_addr_en;
    wire [31: 0] nxt_weight_addr;
    wire weight_cmd_valid;
    wire weight_data_valid;
    wire weight_cmd_hsk = weight_cmd_valid & lacc_data_ready;

    assign weight_cmd_valid = state_weight & ~weight_cmd_end;
    assign weight_data_valid = state_weight & lacc_drsp_valid;
    assign weight_addr_en = idle_exit | weight_cmd_hsk;
    assign weight_addr_cin = {conf_weight_size[1], conf_weight_size[0], ~(|conf_weight_size)};
    assign nxt_weight_addr = {32{idle_exit}} & lacc_req_rj |
                             {32{weight_cmd_hsk}} & (weight_addr + weight_addr_cin);

    wire req_weight_size_end    = req_weight_size == conf_weight_num;
    wire req_weight_row_end     = req_weight_row_idx == conf_kernel_height - 1;
    wire req_weight_col_end     = req_weight_col_idx == conf_kernel_width - 1;
    wire weight_size_end        = weight_size == conf_weight_num;
    wire weight_row_end         = weight_row_idx == conf_kernel_height - 1;
    wire weight_col_end         = weight_col_idx == conf_kernel_width - 1;

    assign weight_line_idx_n    = weight_line_idx + `KERNEL_SIZE;
    assign weight_page_idx_n    = weight_page_idx + `WINDOW_SIZE;

    RDataGen weight_data_gen (conf_weight_size, weight_offset, lacc_drsp_rdata, weight_data_in);

    always @(posedge clk)begin
        if(weight_data_valid)begin
            weight_buf_r[weight_idx] <= weight_data_in;
        end
        if(weight_addr_en) weight_addr <= nxt_weight_addr;
        if(rst | idle_exit)begin
            weight_cmd_end <= 1'b0;
            weight_idx <= 0;
            weight_size <= 0;
            req_weight_size <= 0;
            weight_row_idx <= 0;
            weight_col_idx <= 0;
            req_weight_col_idx <= 0;
            req_weight_row_idx <= 0;
            weight_line_idx <= 0;
            weight_page_idx <= 0;
            weight_offset <= lacc_req_rj[1: 0];
        end
        else begin
            if(state_weight & weight_cmd_hsk & 
                req_weight_col_end & req_weight_row_end & req_weight_size_end) begin
                weight_cmd_end <= 1'b1;
            end
            if(weight_cmd_hsk)begin
                req_weight_col_idx <= req_weight_col_end ? 0 : req_weight_col_idx + 1;
                if(req_weight_col_end)begin
                    req_weight_row_idx <= req_weight_row_end ? 0 : req_weight_row_idx + 1;
                    if(req_weight_row_end)begin
                        req_weight_size <= req_weight_size + 1;
                    end
                end

            end
            if(weight_data_valid)begin
                weight_offset <= weight_offset + weight_addr_cin;
                weight_idx <= weight_col_end & weight_row_end ? weight_page_idx_n :
                              weight_col_end ? weight_line_idx_n : weight_idx + 1;
                weight_col_idx <= weight_col_end ? 0 : weight_col_idx + 1;
                if(weight_col_end)begin
                    weight_row_idx <= weight_row_end ? 0 : weight_row_idx + 1;
                    weight_line_idx <= weight_row_end ? weight_page_idx_n : weight_line_idx_n;
                    if(weight_row_end)begin
                        weight_size <= weight_size + 1;
                        weight_page_idx <= weight_page_idx_n;
                    end
                end
            end
        end
    end

// input buffer

    wire [1: 0] buffer_cmd_size;
    assign buffer_cmd_ready = lacc_data_ready;
    assign buffer_drsp_valid = lacc_drsp_valid & data_req_buf;
    assign buffer_req = weight_exit | idle_exit & ~lacc_req_imm[0];
    CNNBuffer buffer(
        .clk(clk),
        .rst(rst),
        .kernel_width_i(conf_kernel_width),
        .kernel_height_i(conf_kernel_height),
        .buffer_width_i(conf_buf_width),
        .buffer_depth_i(conf_buf_depth),
        .stride_i(conf_stride),
        .padding_i(conf_padding),
        .padding_valid_i(conf_padding_valid),
        .buf_size_i(conf_buf_size),
        .buf_refresh_i(conf_buf_refresh),
        .conf_addr_valid(conf_buf_valid),
        .conf_addr(lacc_req_rj),
        .conf_offset_valid(buf_offset_valid),
        .conf_offset(conf_buf_offset),
        .req(buffer_req),
        .req_final(conv_exit),
        .lacc_data_addr(buffer_cmd_addr),
        .lacc_data_size(buffer_cmd_size),
        .lacc_data_valid(buffer_cmd_valid),
        .lacc_data_ready(buffer_cmd_ready),
        .lacc_drsp_valid(buffer_drsp_valid),
        .lacc_drsp_rdata(lacc_drsp_rdata),
        .window(window),
        .window_valid(window_valid),
        .window_finish(window_finish),
        .window_stall(window_stall)
    );

// conv
    wire [`WEIGHT_SIZE*32-1: 0] conv_data;
    wire [`WEIGHT_SIZE-1: 0] conv_valid;
    wire res_buf_stall;
    wire [31: 0] pool_data;
    wire pool_valid;
    wire conv_window_valid;
    wire pool_window_valid;
    wire window_stall_conv;
    wire window_stall_pool;
    reg [`POOL_MODE_WIDTH-1: 0] pool_mode;

    assign conv_window_valid = window_valid & conv_op;
    wire conv_act_valid = act_op & ~conf_add_write;
generate
    for(i=0; i<`WEIGHT_SIZE*`WINDOW_SIZE; i=i+1)begin
        assign weight_buf_w[i*32 +: 32] = weight_buf_r[i];
    end
    for(i=0; i<`WEIGHT_SIZE; i=i+1)begin
        assign conf_bias_w[i*32 +: 32] = conf_bias[i];
    end
endgenerate
    CNNConv conv(
        .clk(clk),
        .rst(rst),
        .act_valid(conv_act_valid),
        .bias(conf_bias_w),
        .conf_refresh(conf_refresh),
        .kernel_height(kernel_height_vec[`KERNEL_SIZE: 1]),
        .kernel_width(kernel_width_vec[`KERNEL_SIZE: 1]),
        .window_valid(conv_window_valid),
        .window(window),
        .weight(weight_buf_w),
        .conv_data(conv_data),
        .conv_valid(conv_valid),
        .conv_empty(conv_empty),
        .window_stall(window_stall_conv),
        .stall(res_buf_stall)
    );

    assign pool_window_valid = window_valid & pool_op;
    always @(posedge clk)begin
        if(idle_exit) pool_mode <= lacc_req_rj[0 +: `POOL_MODE_WIDTH];
    end
    CNNPool pool(
        .clk(clk),
        .rst(rst),
        .conf_refresh(conf_refresh),
        .kernel_height(kernel_height_vec[`KERNEL_SIZE: 1]),
        .kernel_width(kernel_width_vec[`KERNEL_SIZE: 1]),
        .stall(res_buf_stall),
        .pool_mode(pool_mode),
        .window_valid(pool_window_valid),
        .window(window),
        .window_stall(window_stall_pool),
        .pool_data(pool_data),
        .pool_valid(pool_valid),
        .pool_empty(pool_empty)
    );

    assign window_stall = window_stall_conv | window_stall_pool;

    always @(posedge clk)begin
        if(rst)begin
            conv_op <= 0;
            pool_op <= 0;
            act_op  <= 0;
        end
        else if(lacc_req_valid & cmd_conv)begin
            conv_op <= lacc_req_imm[0];
            pool_op <= lacc_req_imm[1];
            act_op  <= lacc_req_imm[2];
        end
    end

// result buffer
    parameter WEIGHT_WIDTH = $clog2(`WEIGHT_SIZE);
    wire res_cmd_valid;
    wire res_cmd_ready;
    wire res_cmd_hsk;
    wire res_addr_we;
    wire res_wieght_en;
    wire [WEIGHT_WIDTH-1: 0] res_addr_widx;
    wire [31: 0] res_addr_wdata;
    wire [31: 0] res_cmd_addr;
    wire [31: 0] res_cmd_addr_n4;
    wire [15: 0] res_cmd_addr_cin;
    wire [31: 0] res_cmd_wdata;
    reg [WEIGHT_WIDTH-1: 0] res_weight_idx;
    wire [`WEIGHT_SIZE-1: 0] res_weight_vec;
    wire [`WEIGHT_SIZE-1: 0] res_buf_valid;
    wire [`WEIGHT_SIZE-1: 0] res_buf_full;
    wire [`WEIGHT_SIZE*32-1: 0] res_buf_rdata;
    wire [31: 0]                res_buf_rdata_sel;
    wire [31: 0]                res_buf_raddr_sel;
    wire [`WEIGHT_SIZE*32-1: 0] res_buf_wdata;
    wire [`WEIGHT_SIZE-1: 0] res_buf_en;
    reg  [`WEIGHT_SIZE-1: 0] conf_res_buf_valid;
    wire         res_buf_read_req;
    wire         res_buf_write_req;
    wire [31: 0] res_buf_awrite_addr;
    wire [31: 0] res_buf_awrite_data;
    wire         res_buf_awrite_req;
    wire  [31: 0] res_buf_awrite_addr_n;
    wire  [31: 0] res_buf_awrite_data_n;
    wire [31: 0] nxt_res_buf_awrite_data;
    wire [31: 0] nxt_res_drsp_add_data;
    wire [31: 0] nxt_res_drsp_rdata_relu;
    parameter RES_IDX_W = $clog2(`BUFFER_WIDTH);
    reg [`WEIGHT_SIZE*RES_IDX_W-1: 0] res_buf_idx;
    wire [RES_IDX_W-1: 0] res_buf_idx_n;

    assign res_buf_read_req = (|(res_weight_vec & res_buf_valid)) & conf_add_write;
    assign res_buf_write_req = (|(res_weight_vec & res_buf_valid)) & ~conf_add_write |
                                res_buf_awrite_req;

    wire res_buf_addr_valid;
    wire res_drsp_rdata_en = lacc_drsp_valid & lacc_data_source[1] & lacc_drsp_read;
    SplitReg #(64) split_res_buf_addr(clk, rst, conf_add_write & res_cmd_hsk & ~res_buf_awrite_req, {res_buf_rdata_sel, res_buf_raddr_sel}, res_buf_addr_valid,
                                        res_drsp_rdata_en, {res_buf_awrite_data, res_buf_awrite_addr});
    SplitReg #(64) split_awrite_info(clk, rst, res_drsp_rdata_en, {nxt_res_buf_awrite_data, res_buf_awrite_addr}, res_buf_awrite_req,
                                        res_cmd_hsk & res_buf_awrite_req, {res_buf_awrite_data_n, res_buf_awrite_addr_n});

    Decoder #(`WEIGHT_SIZE) decoder_res_weight(res_weight_idx, res_weight_vec);
    assign res_cmd_valid    = res_buf_read_req | res_buf_write_req;
    assign res_cmd_ready    = ~buffer_cmd_valid & lacc_data_ready;
    assign res_cmd_hsk      = res_cmd_valid & res_cmd_ready;
    assign res_cmd_addr     = res_buf_awrite_req ? res_buf_awrite_addr_n : res_buf_raddr_sel;
    assign res_buf_rdata_sel = res_buf_rdata[res_weight_idx*32 +: 32];
    assign res_buf_raddr_sel = conf_res_addr[res_weight_idx];
    assign res_cmd_wdata    = res_buf_awrite_req ? res_buf_awrite_data_n : res_buf_rdata_sel;
    assign res_buf_empty    = ~(|res_buf_valid) & ~res_buf_addr_valid & ~res_buf_awrite_req;
    assign res_buf_stall    = |res_buf_full;
    wire   res_buf_idx_max  = (res_buf_idx[res_weight_idx*RES_IDX_W +: RES_IDX_W] == conf_res_width);
    assign res_cmd_addr_cin = res_buf_idx_max & res_offset_valid ? conf_res_offset + 4 : 4;
    assign res_cmd_addr_n4  = res_cmd_addr + res_cmd_addr_cin;
    assign res_buf_idx_n    = res_buf_idx[res_weight_idx*RES_IDX_W +: RES_IDX_W] + 1;
    assign res_buf_en       = conf_res_buf_valid & (conv_valid | pool_valid);

    assign res_addr_widx = {WEIGHT_WIDTH{lacc_req_valid & cmd_conf_res}} & lacc_req_rk[WEIGHT_WIDTH-1:0] |
                           {WEIGHT_WIDTH{res_cmd_hsk}} & res_weight_idx;
    assign res_addr_we   = lacc_req_valid & cmd_conf_res & lacc_req_imm[0] | res_cmd_hsk & ~res_buf_awrite_req;
    assign res_addr_wdata = {32{lacc_req_valid & cmd_conf_res}} & lacc_req_rj |
                            {32{res_cmd_hsk}} & res_cmd_addr_n4;  
    assign res_weight_en = res_cmd_hsk & (pool_op | conv_op) & ~res_buf_awrite_req;

    assign nxt_res_drsp_add_data = lacc_drsp_rdata + res_buf_awrite_data;
    assign nxt_res_buf_awrite_data = act_op ? nxt_res_drsp_rdata_relu : nxt_res_drsp_add_data;

    Relu resp_buf_relu (nxt_res_drsp_add_data, nxt_res_drsp_rdata_relu);

    always @(posedge clk)begin
        if(conf_refresh) conf_res_buf_valid <= (1 << (conf_weight_num+1)) - 1;
        if(res_addr_we) conf_res_addr[res_addr_widx] <= res_addr_wdata;
        if(rst | idle_exit)begin
            res_weight_idx <= 0;
            res_buf_idx <= 0;
        end
        else begin
            if(res_weight_en)begin
                res_weight_idx <= res_weight_idx == conf_weight_num ? 0 : res_weight_idx + 1;
                res_buf_idx[res_weight_idx*RES_IDX_W +: RES_IDX_W] <= res_buf_idx_max ? 0 : res_buf_idx_n;
            end
        end
        if(lacc_req_valid & cmd_conf_res & (|lacc_req_imm[2: 1])) begin
            conf_bias[res_addr_widx] <= {32{~lacc_req_imm[2]}} & lacc_req_rj;
        end
    end

generate
    for(i=0; i<`WEIGHT_SIZE; i=i+1)begin : gen_res_buf
        reg [`RES_BUF_SIZE*32-1: 0] res_buf;
        reg [$clog2(`RES_BUF_SIZE)-1: 0] head, tail;
        reg hdir, tdir;
        wire equal = head == tail;
        wire dir_xor = hdir ^ tdir;
        wire full = equal & dir_xor;
        wire empty = equal & ~dir_xor;
        assign res_buf_valid[i] = ~empty;
        assign res_buf_rdata[i*32 +: 32] = res_buf[head*32 +: 32];
        assign res_buf_full[i]  = full;
        if(i == 0)begin
            assign res_buf_wdata[i*32 +: 32] = {32{conv_valid[i]}} & conv_data[i*32 +: 32] |
                                      {32{pool_valid}} & pool_data;
        end
        else begin
            assign res_buf_wdata[i*32 +: 32] = conv_data[i*32 +: 32];
        end
        always @(posedge clk)begin
            if(state_conv & res_buf_en[i] & ~res_buf_stall)begin
                res_buf[tail*32 +: 32] <= res_buf_wdata[i*32 +: 32];
            end

            if(rst)begin
                head <= 0;
                tail <= 0;
                hdir <= 1'b0;
                tdir <= 1'b0;
            end
            else begin
                if(state_conv & res_buf_en[i] & ~res_buf_stall)begin
                    tail <= tail + 1;
                    if(&tail)begin
                        tdir <= ~tdir;
                    end
                end
                if(res_cmd_hsk & res_weight_vec[i] & ~res_buf_awrite_req)begin
                    head <= head + 1;
                    if(&head)begin
                        hdir <= ~hdir;
                    end
                end
            end
        end
    end
endgenerate

    always @(posedge clk)begin
        data_req_buf <= buffer_cmd_valid;
    end
    assign lacc_rsp_valid = cmd_conf_buf | conv_exit | cmd_conf_res | cmd_conf_offset;
    assign lacc_rsp_rdat  = 0;

    assign lacc_data_valid = weight_cmd_valid | buffer_cmd_valid | res_cmd_valid;
    assign lacc_data_addr = weight_cmd_valid ? weight_addr :
                            buffer_cmd_valid ? buffer_cmd_addr : res_cmd_addr;
    assign lacc_data_read = ~(res_cmd_valid & ~buffer_cmd_valid & (~conf_add_write | res_buf_awrite_req));
    assign lacc_data_wdata = res_cmd_wdata;
    assign lacc_data_size = buffer_cmd_valid ? buffer_cmd_size : 2'b10;

    wire lacc_source_valid, drsp_read_valid;
    SplitReg #(2) split_lacc_source(clk, rst, lacc_data_valid & lacc_data_ready, {res_cmd_valid & ~buffer_cmd_valid, buffer_cmd_valid}, lacc_source_valid,
                                         lacc_drsp_valid, lacc_data_source);
    SplitReg #(1) split_drsp_read(clk, rst, lacc_data_valid & lacc_data_ready, lacc_data_read, drsp_read_valid,
                                        lacc_drsp_valid, lacc_drsp_read);


endmodule