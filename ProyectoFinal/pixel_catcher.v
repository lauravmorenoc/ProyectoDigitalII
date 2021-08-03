module pixel_catcher(
  input rst, pclk, vsync, href,
  input [7:0]cam_data,
  output reg read_color, reset_color,
  output reg [14:0] pixel_data,
  output reg begin_frame,
  output reg image_select,
  output [12:0]addr_in);
  
  reg last_vsync_state;
  reg [6:0] pixel_data_aux;
  reg [1:0]state;
  reg [12:0]addr_cnt;
  
  assign addr_in = addr_cnt + (13'd3072 & {13{image_select}});
  
  parameter BEGIN=0, CATCH_FIRST_BYTE=2'd1, CATCH_SECOND_BYTE=2'd2;
  
  always @ (posedge (pclk)) 
    begin 
     if(rst) begin 
       state=BEGIN;
       last_vsync_state=0;
       pixel_data_aux=7'd0;
       pixel_data=15'b0;
		 reset_color=1'b1;
       read_color=1'b0;
		 image_select<=0;
		 begin_frame=1'b0;
     end 
     else
       begin 
         case(state)
           BEGIN:
             begin 
               pixel_data_aux=7'd0;
               pixel_data=15'b0;
               read_color=1'b0;
					reset_color=0;
					image_select<=image_select;
               if((~vsync)&last_vsync_state) state= CATCH_FIRST_BYTE; 
               else
				     begin	
					    state=BEGIN;
						 if(vsync)begin_frame=1'b1; 
						 if(vsync&(~last_vsync_state))
						   begin
						   	image_select<=~image_select; // NEW
								addr_cnt=13'b0;
								begin_frame=1'b1;
							end
					  end
               last_vsync_state=vsync;
             end 
           
           CATCH_FIRST_BYTE:
             begin 
               if((~vsync)&&href)
                 begin 
					    begin_frame=1'b0;
                   pixel_data_aux=cam_data[6:0];
                   pixel_data<=pixel_data;
                   read_color=1'b0;
						 addr_cnt=addr_cnt+1'b1;
						 reset_color=0;
                   state=CATCH_SECOND_BYTE;
						 last_vsync_state=vsync;
						 image_select<=image_select;
                 end 
               else 
                 begin 
                   if(vsync)
                     begin 
							  begin_frame=1'b1;
                       reset_color=1'b1;
                       state=BEGIN;
                       read_color=1'b0;
							  if(~last_vsync_state)
						       begin
						   	   image_select<=~image_select; // NEW
								   addr_cnt=13'b0;
							    end
							  last_vsync_state=vsync;
                     end 
                   else
						   begin
							  begin_frame=1'b0;
						     state=CATCH_FIRST_BYTE;
							  last_vsync_state=vsync;
							  image_select<=image_select;
						   end
                 end 
             end 
           CATCH_SECOND_BYTE:
             begin 
               if((~vsync)&&href) 
                 begin 
					    begin_frame=1'b0;
					    reset_color=0;
                   pixel_data={pixel_data_aux, cam_data};
                   last_vsync_state=vsync;
                   read_color=1'b1;
						 image_select<=image_select;
                   state=CATCH_FIRST_BYTE;
                 end 
               else
                 begin 
                   if(vsync)
                     begin 
							  begin_frame=1'b1;
                       reset_color=1;
                       state=BEGIN;
                       read_color=1'b0;
							  if(~last_vsync_state)
						       begin
						   	   image_select<=~image_select; // NEW
								   addr_cnt=13'b0;
							    end
							  last_vsync_state=vsync;
                     end 
                   else
						 begin
						   begin_frame=1'b0;
						   state=CATCH_FIRST_BYTE;
							reset_color<=0;
							read_color<=0;
							image_select<=image_select;
							last_vsync_state=vsync;
						 end
                 end 
             end 
		   endcase
     end 
    end 
  
endmodule 