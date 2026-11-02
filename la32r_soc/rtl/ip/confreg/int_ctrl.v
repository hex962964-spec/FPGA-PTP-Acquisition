module one_int_gen(  
    input clk,  
    input resetn,  
   
    input int_en,  
    input int_edga,  
    input int_pol,  
    input int_in,  
    input int_clr,  
    input int_set,  
    output int_out  
);  
    reg int_delay;  
    reg int_lt;  
   
    always @(posedge clk) begin  
        if(~resetn)  
            int_delay <= 1'b0;  
        else  
            int_delay <= int_in;  
    end  
   
    always @(posedge clk) begin  
        if(~resetn)  
            int_lt <= 1'b0;  
        else if(int_in != int_delay && int_in == int_pol) int_lt <= 1'b1;  
        else if(int_set) int_lt <= 1'b1;  
        else if(int_clr) int_lt <= 1'b0;  
    end  
   
    wire int_inv = ~(int_pol ^ int_in);  
    wire level_int = int_edga ? int_lt : int_inv;  
    assign int_out = int_en & level_int;  
   
endmodule  
   
module int_reg #(parameter N=4)  
(  
    input clk,  
    input resetn,  
   
    input [N-1:0] int_en,  
    input [N-1:0] int_edga,  
    input [N-1:0] int_pol,  
    input [N-1:0] int_in,  
    input [N-1:0] int_clr,  
    input [N-1:0] int_set,  
    output [N-1:0]int_out  
);  

genvar i;  
generate for(i=0;i<N;i=i+1) begin: generate_int_reg  
    one_int_gen u_one_int_gen(  
        .clk   (clk),  
        .resetn(resetn),  
        .int_en(int_en[i]),  
        .int_edga(int_edga[i]),  
        .int_pol(int_pol[i]),  
        .int_in(int_in[i]),  
        .int_clr(int_clr[i]),  
        .int_set(int_set[i]),  
        .int_out(int_out[i])  
    );
end
endgenerate

endmodule