module sramsp_maskoff #(
  parameter DATA_WIDTH = 16,
  parameter ADDR_WIDTH = 8
) ( 
  input CLK,
  input GWEN,
  input CEN,
  input [ADDR_WIDTH-1:0] A,
  input [DATA_WIDTH-1:0] D,  
  output [DATA_WIDTH-1:0] Q
);

  reg [DATA_WIDTH-1:0] mem [0:(1 << ADDR_WIDTH)-1];  
  reg [DATA_WIDTH-1:0] Q_reg;

  always @(posedge CLK) begin
    if ((! CEN) && (! GWEN )) begin 
      mem[A] <= D; 
    end 
  end  
  always @(posedge CLK) begin
    if ((! CEN) && GWEN) begin 
      Q_reg <= mem[A]; 
    end 
    else begin
      Q_reg <= {DATA_WIDTH{1'bx}};
    end
  end

  assign Q = Q_reg;

endmodule

module fpga_sram_sp_maskoff #(
  parameter DATA_WIDTH = 16,
  parameter ADDR_WIDTH = 8
) ( 
  input CLK,
  input GWEN,
  input CEN,
  input [ADDR_WIDTH-1:0] A,
  input [DATA_WIDTH-1:0] D,  
  output [DATA_WIDTH-1:0] Q
);

    localparam V_STYLE = "block";
    localparam P_STYLE =    (V_STYLE == "ultra")        ? "uram" :
                            (V_STYLE == "distributed")  ? "select_ram" :
                            "block_ram";

    (*ram_style = V_STYLE*)reg [DATA_WIDTH-1:0] mem [0:(1 << ADDR_WIDTH)-1]/*synthesis syn_ramstyle=P_STYLE*/;
        
    reg [DATA_WIDTH-1:0] Q_reg;

    always @(posedge CLK) begin
        if ((! CEN) && (! GWEN )) begin 
            mem[A] <= D; 
        end 
    end  
    always @(posedge CLK) begin
        if ((! CEN) && GWEN) begin 
            Q_reg <= mem[A]; 
        end
    end

    assign Q = Q_reg;

endmodule