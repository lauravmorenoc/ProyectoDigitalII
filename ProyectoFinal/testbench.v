`timescale 1ns / 1ps
module testbench();

reg clk, href, vsync, reset, pclk, frame_start;
reg [7:0]cam_data;
wire [2:0]color_code;
wire xclk;

reg [4:0]pclk_counter;
reg [14:0]href_counter;
reg [19:0]vsync_counter;
reg first_frame=0;
reg last_vsync_tb;

parameter cam1=8'b01100000, cam2=8'b10101010;

main U1(.camera_data(cam_data), 
.pclk(pclk),
.href(href),
.vsync(vsync),
.clk(clk),
.reset(reset), 
.color_code(color_code),
.xclk(xclk));

initial
  begin
    $dumpfile("dump.vcd"); $dumpvars;
	 
	 clk=0;
	 href=0;
	 pclk=0;
	 vsync=1;
	 reset=1;
	 vsync_counter=0;
	 href_counter=6'b0;
	 pclk_counter=0;
	 first_frame=0;
	 frame_start=0;
	 cam_data=8'b01100000;
	 #100
	 reset=0;
	 #2560
	 reset=1;
	 #2560;
	 vsync=0;
	 #3054000000
	 
	 
	 
	 $finish;
  end

  always
  begin
     #10; 
	  clk = ~clk;
	  if((~vsync)&last_vsync_tb)begin
		 first_frame=1;
		 frame_start=1;
		 href=0;
	  end
	  last_vsync_tb=vsync;
  end
  
  always@(posedge pclk)begin
  
    if(frame_start)begin
	   if(href_counter==15'd26656)begin
		href_counter=0;
		href=1;
		frame_start=0;
		end else begin
	   	href_counter=href_counter+1'b1;
			vsync_counter=vsync_counter+1;
		end
	 end
  
    if(first_frame&(~frame_start))begin
	 
	 // href
	   if((href_counter==14'd17120)&(~href))begin
		  href=~href;
		  href_counter=0;
		end else begin
		  if((href_counter==14'd128)&href)begin
		    href=~href;
			 href_counter=0;
		  end
		end
		
		// vsync
	   if((vsync_counter==20'd794976)&(~vsync))begin
		  vsync=~vsync;
		  vsync_counter=0;
		end else begin
		  if((vsync_counter==20'd4704)&vsync)begin
		    vsync=~vsync;
			 vsync_counter=0;
		  end
		end
		
		
	   href_counter=href_counter+1;
		vsync_counter=vsync_counter+1;
	 end
  end
  
  always @(xclk) begin
	 if(pclk_counter==5'd32) begin
	   pclk=~pclk;
		if(~pclk)cam_data=cam_data+2'b11;
		pclk_counter=0;
	 end
	 pclk_counter=pclk_counter+1;
  end

endmodule 