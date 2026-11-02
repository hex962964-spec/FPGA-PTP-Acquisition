`ifndef __CNNCONFIG_VH__
`define __CNNCONFIG_VH__

`define KERNEL_SIZE 3
`define KERNEL_WIDTH $clog2(`KERNEL_SIZE)
`define STRIDE_WIDTH 2
`define WINDOW_SIZE (`KERNEL_SIZE * `KERNEL_SIZE)
`define BUFFER_DEPTH 8
`define BUFFER_WIDTH 32 // int8
`define WEIGHT_SIZE 3
`define RES_BUF_SIZE 8
`define MAX_STRIDE_WIDTH 2

`define POOL_MODE 3
`define POOL_MODE_WIDTH $clog2(`POOL_MODE)

`endif