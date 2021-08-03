module main(input [7:0]camera_data, 
input pclk, href, vsync, clk, reset, arduinoSignal,
output [2:0]color_code,
output reg xclk = 1'b0,
output Tx_data, arduino_output);

wire read_color;
wire [14:0]pixel_data;
wire reset_color;
wire image_select;
wire [2:0]RAM_data, data_out;
wire [12:0]addr_in, addr_out;
wire begin_frame;
wire regwrite;
wire pclk_aux;

pclk_driver driver(
      .clk(clk),
      .rst(~reset),
      .pclk_in(pclk),
      .pclk_out(pclk_aux));

pixel_catcher UU1(.rst(~reset),
.addr_in(addr_in), 
.pclk(pclk_aux), 
.vsync(vsync), 
.href(href),
.cam_data(camera_data),
.reset_color(reset_color), 
.read_color(read_color),
.pixel_data(pixel_data),
.image_select(image_select),
.begin_frame(begin_frame));

color_finder UU2(.pixel_data(pixel_data),
.regwrite(regwrite),
.clk(clk), 
.read_color(read_color), 
.rst(reset_color),
.final_code(color_code), 
.RAM_data(RAM_data));

buffer_ram_dp RAM(.clk_w(clk),
.addr_in(addr_in),
.data_in(RAM_data),
.regwrite(regwrite),
.clk_r(clk),
.addr_out(addr_out),
.data_out(data_out));

image_sender im_send(.clk(clk),
.rst(~reset),
.begin_frame(begin_frame),
.image_select(image_select),
.pixel_data(data_out),
.addr_out(addr_out),
.Tx_data(Tx_data));

arduinoUART arUART(.clk(clk), 
.rst(~reset), 
.ardSignal(arduinoSignal),
.Tx_data(color_code),
.Tx_serial(arduino_output));

// Cambiar a la mitad
  always @ ( posedge(clk) )
  begin
    //if (divider_31_25M == 1'b1)
    //begin
      xclk = !xclk;
      //divider_31_25M = 1'b0;
    //end
    //else
    //begin
     // divider_31_25M = divider_31_25M + 1'b1;
    //end
  end

endmodule