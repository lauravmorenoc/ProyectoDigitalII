module clk_enable(input clk,
output reg clk_enable=0, 
output reg [7:0]cam_data=8'b01100000); //divisor de frecuencias
	 
  parameter maxCount = 99999999; //reloj de entrada de 100MHz
  
  // (entrada/deseada)-1 (así es como funciona)
  
  parameter N = 27; // depende de la cantidad de bits que requiera maxCount
  
	 reg [N-1:0]cnt = 0;
	 
	 always @(posedge clk)
	 
		begin
		   if(cnt == maxCount) 
			begin
				cnt <= 0;
              clk_enable=~clk_enable;
			end
			else
			begin
				cnt <= cnt + 1'b1;
			end			
		end		
endmodule 