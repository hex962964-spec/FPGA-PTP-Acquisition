`include "CNNConfig.vh"


module CNNBuffer(
    input                               clk,
    input                               rst,

    input [`KERNEL_WIDTH-1: 0]          kernel_width_i,
    input [`KERNEL_WIDTH-1: 0]          kernel_height_i,
    input [$clog2(`BUFFER_DEPTH)-1: 0]  buffer_depth_i,
    input [$clog2(`BUFFER_WIDTH)-1: 0]  buffer_width_i,
    input [`STRIDE_WIDTH-1: 0]          stride_i,
    input [`KERNEL_WIDTH-1: 0]          padding_i,
    input [3: 0]                        padding_valid_i,
    input [1: 0]                        buf_size_i,
    input                               buf_refresh_i,

    input                               conf_addr_valid,
    input [31: 0]                       conf_addr,
    input                               conf_offset_valid,
    input [15: 0]                       conf_offset,

    input                               req,
    input                               req_final, // op finish

    output                              lacc_data_valid,
    output [31: 0]                      lacc_data_addr,
    output [1: 0]                       lacc_data_size,
    input                               lacc_data_ready,
    input                               lacc_drsp_valid,
    input [31: 0]                       lacc_drsp_rdata,

    output [`WINDOW_SIZE-1: 0][31: 0]   window,
    output                              window_valid,
    output                              window_finish,
    input                               window_stall
);
    reg [$clog2(`BUFFER_WIDTH): 0]      idx_x;
    reg [`BUFFER_DEPTH-1: 0]            vec_y;
    reg                                 last;
    reg [$clog2(`BUFFER_WIDTH): 0]      req_idx_x;
    reg [$clog2(`BUFFER_DEPTH)-1: 0]    req_idx_y;
    reg [31: 0]                         buf_addr;
    
    reg [`KERNEL_SIZE-1: 0]             kernel_fill;
    reg [$clog2(`BUFFER_WIDTH): 0]      kernel_idx_x;
    reg [$clog2(`BUFFER_DEPTH): 0]      kernel_idx_y;
    reg                                 kernel_data_req_end;
    wire [$clog2(`BUFFER_DEPTH)-1: 0]   idx_y;
    wire                                kernel_data_req;
    wire                                kernel_idx_x_max;
    wire                                kernel_padding_x_ov;
    wire [31: 0]         window_rdata [`KERNEL_SIZE-1: 0];
    wire [2: 0]                         data_cin;
// state control
    parameter FSM_WIDTH = 1;
    parameter BUFFER_SIZE = `BUFFER_WIDTH * `BUFFER_DEPTH;
    parameter IDLE      = 'b0;
    parameter REQ       = 'b1;
    reg [FSM_WIDTH-1: 0] state_r;
    wire [FSM_WIDTH-1: 0] nxt_state;
    wire state_en;
    wire state_idle = state_r == IDLE;
    wire state_req  = state_r == REQ;
    wire idle_exit  = state_idle & req;
    wire req_exit   = state_req & req_final;

    assign state_en = idle_exit | req_exit;
    assign nxt_state = {FSM_WIDTH{idle_exit}} & REQ |
                       {FSM_WIDTH{req_exit}} & IDLE;    

    always @(posedge clk)begin
        if(rst)begin
            state_r <= IDLE;
        end else begin
            if(state_en) state_r <= nxt_state;
        end
    end

// channel control
    reg                         cmd_valid_r;
    reg                         cmd_read_r;
    wire                        cmd_valid_last;
    wire                        cmd_hsk;
    wire                        req_idx_x_max;

    assign cmd_valid_last   = state_req & lacc_data_ready & 
                              (req_idx_x_max & (req_idx_y == buffer_depth_i));
    assign cmd_hsk          = lacc_data_ready & cmd_valid_r;

    always @(posedge clk)begin
        if(rst)begin
            cmd_valid_r <= 1'b0;
        end 
        else begin
            if(idle_exit & buf_refresh_i)begin
                cmd_valid_r <= 1'b1;
            end
            else if(cmd_valid_last) begin
                cmd_valid_r <= 1'b0;
            end
        end
    end
    
    assign lacc_data_valid   = cmd_valid_r;

// idx control
    wire [$clog2(`BUFFER_WIDTH): 0] remain_buf_col;
    wire [15: 0] buf_addr_cin;
    wire [1: 0] buf_data_size;
    wire [1: 0] buf_data_offset;
    wire [2: 0] lacc_data_size_cin;
    wire [2: 0] idx_x_cin;
    reg [$clog2(`BUFFER_WIDTH): 0] buffer_width_n;

    wire idx_x_max = idx_x + idx_x_cin >= buffer_width_n;
    assign remain_buf_col = buffer_width_n - req_idx_x;
    assign req_idx_x_max = req_idx_x + lacc_data_size_cin >= buffer_width_n;

    wire [31: 0] nxt_buf_addr;
    wire [31: 0] nxt_req_addr;
    wire buf_addr_en = conf_addr_valid | cmd_hsk;
    wire remain_col_ge4 = (|remain_buf_col[$clog2(`BUFFER_WIDTH): 2]);
    wire remain_col_le2 = ~remain_col_ge4 & ~remain_buf_col[1];
    assign lacc_data_size_cin = {lacc_data_size[1], lacc_data_size[0], ~(|lacc_data_size)};
    assign lacc_data_size = {{~buf_addr[1] & ~buf_addr[0] & remain_col_ge4},
                           {buf_addr[1] & ~buf_addr[0] & ~remain_col_le2}};
    assign buf_addr_cin = conf_offset_valid & req_idx_x_max ? conf_offset + lacc_data_size_cin : 
                            lacc_data_size_cin;
    assign nxt_req_addr = buf_addr + buf_addr_cin;
    assign nxt_buf_addr = {32{conf_addr_valid}} & conf_addr |
                          {32{cmd_hsk}} & nxt_req_addr;
    assign lacc_data_addr = buf_addr;
    wire data_size_valid, data_offset_valid;
    SplitReg #(2) split_data_size(clk, rst, cmd_hsk, lacc_data_size, data_size_valid, lacc_drsp_valid, buf_data_size);
    SplitReg #(2) split_data_offset(clk, rst, cmd_hsk, lacc_data_addr[1: 0], data_offset_valid, lacc_drsp_valid, buf_data_offset);
    assign idx_x_cin = {buf_data_size[1], buf_data_size[0], ~(|buf_data_size)};

    always @(posedge clk)begin
        buffer_width_n <= buffer_width_i + 1;
        if(buf_addr_en) buf_addr <= nxt_buf_addr;
        if(rst | req)begin
            idx_x <= 0;
            vec_y <= 1;
            last  <= ~buf_refresh_i;
            req_idx_x <= 0;
            req_idx_y <= 0;
        end
        else begin
            if(cmd_hsk)begin
                req_idx_x <= req_idx_x_max ? 0 : req_idx_x + buf_addr_cin;
                if(req_idx_x_max)begin
                    req_idx_y <= req_idx_y + 1;
                end
            end
            if(lacc_drsp_valid)begin
                idx_x <= idx_x_max ? 0 : idx_x + idx_x_cin;
                if(idx_x_max) begin
                    vec_y <= {vec_y[`BUFFER_DEPTH-2: 0], vec_y[`BUFFER_DEPTH-1]};
                    last <= vec_y[buffer_depth_i];
                end
            end
        end
    end

// buffer control
    wire [31: 0] rdata [`BUFFER_DEPTH-1: 0];
    wire [4*2-1: 0] drsp_ridx;
    wire [`BUFFER_WIDTH*4-1: 0] buf_wvecs;
    wire [`BUFFER_WIDTH-1: 0] buf_wvec;
    wire [31: 0] drsp_rdata_sft;
    wire [31: 0] drsp_wdata_normal;
    wire [4: 0]  drsp_sfamt;

    assign buf_wvec = buf_wvecs[`BUFFER_WIDTH-1: 0] |
                      {`BUFFER_WIDTH{|buf_data_size}} & buf_wvecs[`BUFFER_WIDTH +: `BUFFER_WIDTH] |
                      {`BUFFER_WIDTH{buf_data_size[1]}} & buf_wvecs[`BUFFER_WIDTH*2 +: `BUFFER_WIDTH] |
                      {`BUFFER_WIDTH{buf_data_size[1]}} & buf_wvecs[`BUFFER_WIDTH*3 +: `BUFFER_WIDTH];
    assign drsp_sfamt = {buf_data_offset, 3'b0};
    assign drsp_rdata_sft = lacc_drsp_rdata >> drsp_sfamt;

    genvar i, j;
generate
    for(i=0; i<4; i=i+1)begin
        assign drsp_ridx[i*2 +: 2] = i - idx_x[1: 0];
        wire [1: 0] sel_idx = drsp_ridx[i*2 +: 2];
        assign drsp_wdata_normal[i*8 +: 8] = drsp_rdata_sft[sel_idx * 8 +: 8];
        wire [$clog2(`BUFFER_WIDTH)-1: 0] widx;
        assign widx = idx_x + i;
        Decoder #(`BUFFER_WIDTH) decoder_widx (widx, buf_wvecs[i*`BUFFER_WIDTH +: `BUFFER_WIDTH]);
    end
    for(i=0; i<`BUFFER_DEPTH; i=i+1)begin : gen_buffer
        reg [31: 0] buffer_row [`BUFFER_WIDTH/4-1: 0];
        assign rdata[i] = buffer_row[kernel_idx_x[$clog2(`BUFFER_WIDTH)-1: 2]];
        for(j=0; j<`BUFFER_WIDTH/4; j=j+1)begin
            wire [31: 0] buf_wmask = {{8{buf_wvec[j*4+3]}}, {8{buf_wvec[j*4+2]}}, {8{buf_wvec[j*4+1]}}, {8{buf_wvec[j*4]}}};
            always @(posedge clk)begin
                if(lacc_drsp_valid & vec_y[i] & (|buf_wvec[j*4 +: 4]))begin
                    buffer_row[j] <= drsp_wdata_normal & buf_wmask;
                end
            end
        end
    end
endgenerate

// window gen
    reg [$clog2(`BUFFER_DEPTH): 0] buffer_depth_n;
    Encoder #(`BUFFER_DEPTH) encoder_idx_y (vec_y, idx_y);
    wire [`STRIDE_WIDTH-1: 0] kernel_fill_shift_idx;
    wire [`KERNEL_SIZE-1: 0]        window_col;
    reg [$clog2(`BUFFER_WIDTH): 0] buffer_width_max;
    reg kernel_next_line;
    reg [1: 0] data_offset;
    wire [2: 0] kernel_idx_x_cin;

    wire [`KERNEL_WIDTH-1: 0] kernel_fill_all_idx;
    assign kernel_fill_all_idx = kernel_width_i - 1;
    wire kernel_fill_all = kernel_fill[kernel_fill_all_idx];

    assign kernel_fill_shift_idx    = {`STRIDE_WIDTH{kernel_fill_all}} & stride_i;

    assign kernel_data_req = state_req & ~kernel_data_req_end & ~window_stall &
                            ((idx_y > kernel_idx_y) | last | (idx_y == kernel_idx_y) & (kernel_idx_x + data_cin <= idx_x));
    assign kernel_idx_x_max = kernel_idx_x  + kernel_idx_x_cin == buffer_width_max;
    assign kernel_padding_x_ov = kernel_idx_x > buffer_width_i;

    wire [$clog2(`BUFFER_WIDTH): 0] padding_expand, padding_sub;
    assign padding_expand = padding_i;
    assign padding_sub = {$clog2(`BUFFER_WIDTH)+1{padding_valid_i[0]}} & ((~padding_expand) + 1);


    assign data_cin         = {buf_size_i[1], buf_size_i[0], ~(|buf_size_i)};
    assign kernel_idx_x_cin = kernel_padding_x_ov ? 1 : data_cin;

    always @(posedge clk)begin
        buffer_depth_n <= buffer_depth_i + 1;
        buffer_width_max <= (buffer_width_n + ({`KERNEL_WIDTH{padding_valid_i[1]}} & padding_i));
        kernel_next_line <= kernel_data_req & kernel_idx_x_max;
        if(rst | req)begin
            kernel_fill <= 0;
            kernel_idx_x <= padding_sub;
            kernel_idx_y <= kernel_height_i - 1 - ({`KERNEL_WIDTH{padding_valid_i[2]}} & padding_i);
            kernel_data_req_end <= 1'b0;
            data_offset <= 0;
        end 
        else begin
            if(kernel_data_req)begin
                kernel_fill <= (((kernel_fill & {`KERNEL_SIZE{~kernel_next_line}}) >> kernel_fill_shift_idx) | window_col);
                data_offset <= kernel_idx_x[$clog2(`BUFFER_WIDTH)] | kernel_idx_x_max ? 0 : data_offset + data_cin;
                kernel_idx_x <= kernel_idx_x_max ? padding_sub : kernel_idx_x + kernel_idx_x_cin;
                if(kernel_idx_x_max)begin
                    kernel_idx_y <= kernel_idx_y + stride_i;
                    if(kernel_idx_y >= buffer_depth_n - stride_i + ({`KERNEL_WIDTH{padding_valid_i[3]}} & padding_i))begin
                        kernel_data_req_end <= 1'b1;
                    end
                end
            end
        end
    end

    wire [`KERNEL_SIZE-1: 0] kernel_row_ov;
generate
    for(i=0; i<`KERNEL_SIZE; i=i+1)begin : gen_window_rdata
        wire [$clog2(`BUFFER_DEPTH): 0] window_ridx;
        assign window_ridx = kernel_idx_y - kernel_height_i + 1 + i;
        wire [$clog2(`BUFFER_DEPTH)-1: 0] window_ridx_valid;
        assign window_ridx_valid        = window_ridx[$clog2(`BUFFER_DEPTH)-1: 0];
        assign kernel_row_ov[i]         = window_ridx > buffer_depth_i;
        assign window_rdata[i] = rdata[window_ridx_valid];
    end
endgenerate


    reg [31: 0]      window_r [`WINDOW_SIZE-1: 0];
    wire [`WINDOW_SIZE*32-1: 0] window_r_w;
    reg [$clog2(`KERNEL_SIZE)-1: 0] window_col_idx;

    Decoder #(`KERNEL_SIZE) decoder_window_col(window_col_idx, window_col);

    wire [`KERNEL_SIZE-1: 0] kernel_width_mask, kernel_height_mask;
    assign kernel_width_mask = (1 << kernel_width_i) - 1;
    assign kernel_height_mask = (1 << kernel_height_i) - 1;

generate
    for(i=0; i<`KERNEL_SIZE; i=i+1)begin : gen_window
        wire [31: 0] window_rdata_in, window_wdata_v;
        RDataGen data_gen(buf_size_i, data_offset, window_rdata[i], window_rdata_in);
        assign window_wdata_v = {32{~kernel_padding_x_ov & ~kernel_row_ov[i]}} & window_rdata_in;
        for(j=0; j<`KERNEL_SIZE; j=j+1)begin : gen_window_row
            wire window_fill_en = kernel_data_req & (kernel_fill_all | ~kernel_fill_all & window_col[j]) | req;
            wire [31: 0] window_wdata;

            wire [$clog2(`WINDOW_SIZE)-1: 0] shift_idx;
            
            assign window_r_w[(i*`KERNEL_SIZE+j)*32 +: 32] = window_r[i*`KERNEL_SIZE+j];
            assign shift_idx = i * `KERNEL_SIZE + j + stride_i;
            assign window_wdata = ~kernel_height_mask[i] | ~kernel_width_mask[j] | req ? 0 :
                                  kernel_fill_all & ~window_col[j] ? window_r[shift_idx] : 
                                                                    window_wdata_v;

            always @(posedge clk)begin
                if(window_fill_en) window_r[(i * `KERNEL_SIZE + j)] <= window_wdata;
            end
        end
    end
endgenerate

    wire [`KERNEL_WIDTH-1: 0] kernel_fill_idx;
    wire [`KERNEL_WIDTH-1: 0] nxt_window_col_idx, window_col_idx_n;
    reg kernel_data_req_n;
    assign kernel_fill_idx = kernel_width_i > 1 ? kernel_width_i - 2 : 0;
    assign window_col_idx_n = window_col_idx + 1;
    assign nxt_window_col_idx = kernel_idx_x_max ? 0 :
                              (window_col_idx == kernel_width_i - 1) ? window_col_idx_n - stride_i : window_col_idx_n;
    always @(posedge clk)begin
        if(~window_stall)begin
            kernel_data_req_n <= kernel_data_req;
        end
        if(rst)begin
            window_col_idx <= 0;
        end
        else begin 
            if(kernel_data_req)begin
                window_col_idx <= nxt_window_col_idx;
            end
        end
    end
    assign window           = window_r_w;
    assign window_valid     = kernel_data_req_n & kernel_fill_all;
    assign window_finish    = kernel_data_req_end;
endmodule