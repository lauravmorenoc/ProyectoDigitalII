// Módulo de envío UART - ARDUINO

module arduinoUART(input clk, rst, ardSignal,
input [2:0]Tx_data,
output Tx_serial);

wire Tx_active,Tx_done;
reg Tx_DV, lastArduino;
reg [7:0]Tx_byte;

uart_tx #(.CLKS_PER_BIT(5209), .N(13)) uart(.i_Clock(clk), // input
.rst(rst),                  // input
.i_Tx_DV(Tx_DV),            // input
.i_Tx_Byte(Tx_byte),        // input
.o_Tx_Active(Tx_active),    // output
.o_Tx_Serial(Tx_serial),    // output
.o_Tx_Done(Tx_done));       // output

always @(posedge (clk))
begin
  if(rst)
    begin
	   Tx_byte=8'b0;
		Tx_DV=1'b0;
		lastArduino=ardSignal;
    end
  else 
    begin
	   Tx_byte={5'd1,Tx_data};
	   if(ardSignal&(~lastArduino)&(~Tx_active)) Tx_DV=1'b1;
		else Tx_DV=1'b0;
		lastArduino=ardSignal;
	 end 
end

endmodule 