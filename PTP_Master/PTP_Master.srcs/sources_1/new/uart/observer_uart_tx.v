`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: observer_uart_tx
// -----------------------------------------------------------------------------
// Purpose:
//   Simple 8N1 UART transmitter with ready/valid byte input.
//////////////////////////////////////////////////////////////////////////////////
module observer_uart_tx #(
    parameter integer CLK_FREQ_HZ = 50000000,
    parameter integer BAUD_RATE   = 115200
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] s_data,
    input  wire       s_valid,
    output wire       s_ready,

    output reg        uart_tx,
    output wire       busy
);

    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;
    localparam integer DIV_WIDTH = 16;

    reg [DIV_WIDTH-1:0] baud_cnt_r;
    reg [3:0]           bit_idx_r;
    reg [9:0]           shifter_r;
    reg                 busy_r;

    assign s_ready = !busy_r;
    assign busy    = busy_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt_r <= {DIV_WIDTH{1'b0}};
            bit_idx_r  <= 4'd0;
            shifter_r  <= 10'h3FF;
            busy_r     <= 1'b0;
            uart_tx    <= 1'b1;
        end else begin
            if (!busy_r) begin
                uart_tx <= 1'b1;
                if (s_valid) begin
                    // start bit + 8 data bits LSB first + stop bit
                    shifter_r  <= {1'b1, s_data, 1'b0};
                    bit_idx_r  <= 4'd0;
                    baud_cnt_r <= CLKS_PER_BIT - 1;
                    busy_r     <= 1'b1;
                    uart_tx    <= 1'b0;
                end
            end else begin
                if (baud_cnt_r == {DIV_WIDTH{1'b0}}) begin
                    shifter_r <= {1'b1, shifter_r[9:1]};
                    bit_idx_r <= bit_idx_r + 4'd1;
                    uart_tx   <= shifter_r[1];
                    baud_cnt_r <= CLKS_PER_BIT - 1;

                    if (bit_idx_r == 4'd9) begin
                        busy_r  <= 1'b0;
                        uart_tx <= 1'b1;
                    end
                end else begin
                    baud_cnt_r <= baud_cnt_r - 1'b1;
                end
            end
        end
    end

endmodule
`default_nettype wire
