module fpga_sram_sp #(
    parameter AW = 16,
    parameter Init_File = "none"
)
(
    input  wire          CLK,
    input  wire [AW-1:0] ADDR,
    input  wire [31:0]   WDATA,
    input  wire [3:0]    WREN,
    input  wire          CS,
    output wire [31:0]   RDATA
);

    localparam DEPTH = (1 << AW);

    (* ram_style = "block" *) reg [31:0] BRAM [0:DEPTH-1];
    reg [31:0] rdata_r;

    wire [3:0] write_enable = WREN & {4{CS}};
    wire       read_enable  = CS & ~(|WREN);

    initial begin
        if (Init_File != "none") begin
            $readmemb(Init_File, BRAM);
        end
    end

    always @(posedge CLK) begin
        if (CS) begin
            if (|write_enable) begin
                if (write_enable[0]) BRAM[ADDR][7:0]   <= WDATA[7:0];
                if (write_enable[1]) BRAM[ADDR][15:8]  <= WDATA[15:8];
                if (write_enable[2]) BRAM[ADDR][23:16] <= WDATA[23:16];
                if (write_enable[3]) BRAM[ADDR][31:24] <= WDATA[31:24];
            end
            else if (read_enable) begin
                rdata_r <= BRAM[ADDR];
            end
        end
    end

    assign RDATA = rdata_r;

endmodule
