module Pipeline_Reg #(
    parameter WIDTH = 32
)(
    input wire clk,
    input wire rst,
    input wire stall,
    input wire flush,
    input wire [WIDTH-1:0] d,
	
    output reg [WIDTH-1:0] q
);

    always @(posedge clk or posedge rst) 
	begin
        if(rst || flush) 
		begin
            q <= {WIDTH{1'b0}};
        end 
		
		else if(!stall) 
		begin
            q <= d;
        end
    end
	
endmodule
