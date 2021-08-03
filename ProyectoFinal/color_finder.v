module color_finder(input [14:0]pixel_data,
       input clk,read_color, rst,
       output reg [2:0]final_code=3'b111,
		 output reg regwrite,
		 output reg [2:0]RAM_data);
  
  parameter BEGIN=0, READ_RGB=2'd1, CHOOSE_COLOR=2'd2, WAIT=2'd3;
  parameter BLACK=3'b000, BLUE=3'b001, GREEN=3'b010, CYAN=3'b011, RED=3'b100, VIOLET=3'b101, YELLOW=3'b110, WHITE=3'b111;
  
  reg [14:0] black,blue,green,cyan,red,violet,yellow,white;
  reg [1:0]state;
  reg [2:0]pixel_RGB, color_code=3'b111;
  reg color_done;
    
  always @ (posedge clk) begin
    if(rst)begin
      state=BEGIN;
      {black,blue,green,cyan,red,violet,yellow,white}<=120'b0;
		final_code=color_code;
      color_code=color_code; // default frame color is white 
      pixel_RGB<=3'b111; // default pixel color is white 
		color_done=0;
		regwrite=0;
		RAM_data=3'b0;
    end
    else
      begin
        case(state)
		    BEGIN:
			   begin
				  state=READ_RGB;
              {black,blue,green,cyan,red,violet,yellow,white}<=120'b0;
              color_code<=color_code; // default frame color is white 
              final_code<=final_code;
				  pixel_RGB<=3'b111; // default pixel color is white 
		        color_done=0;
				  regwrite=0;
				  RAM_data=3'b0;
			   end
          READ_RGB:
            begin
              if(read_color&(~color_done))
              begin 
              
				  /*
                if((pixel_data[14])||(pixel_data[13])) pixel_RGB[2]=1'b1;
                else pixel_RGB[2]=1'b0;
                if((pixel_data[9])||(pixel_data[8])) pixel_RGB[1]=1'b1;
                else pixel_RGB[1]=1'b0;
                if((pixel_data[4])||(pixel_data[3])) pixel_RGB[0]=1'b1;
                else pixel_RGB[0]=1'b0;
               */           
					 if(pixel_data[14]) pixel_RGB[2]=1'b1; //Rojo //65432R 10765G 43210B
                else pixel_RGB[2]=1'b0;
					 
                if(pixel_data[9]) pixel_RGB[1]=1'b1;  //Verde
                else pixel_RGB[1]=1'b0;
					 
                if(pixel_data[4]) pixel_RGB[0]=1'b1;  //Azul
                else pixel_RGB[0]=1'b0;
					
					RAM_data=pixel_RGB;
					regwrite=1;      /// VA EN UNO 01111R 11111G 11111B --> 1R 1G 1B
					//RAM_data=pixel_data;
					
                case (pixel_RGB)
                  BLACK:
                    begin
                      if(~(&black)) black<=black+15'd1;
                    end
                  BLUE:
                    begin
                      if(~(&blue)) blue<=blue+15'd1;
                    end
                  GREEN:
                    begin
                      if(~(&green)) green<=green+15'd1;
                    end
                  CYAN:
                    begin
                      if(~(&cyan)) cyan<=cyan+15'd1;
                    end
                  RED:
                    begin
                      if(~(&red)) red<=red+15'd1;
                    end
                  VIOLET:
                    begin
                      if(~(&violet)) violet<=violet+15'd1;
                    end
                  YELLOW:
                    begin
                      if(~(&yellow)) yellow<=yellow+15'd1;
                    end
                  default:
                    begin
                      if(~(&white)) white<=white+15'd1;
                    end
					  endcase
                  
					 color_done=0;
                state=CHOOSE_COLOR;
              end 
              else state=READ_RGB;
            end
          CHOOSE_COLOR:
            begin
              if((black>blue)&&(black>green)&&(black>cyan)&&(black>red)&&(black>violet)&&(black>yellow)&&(black>white)) color_code<=WHITE;
              else if((blue>black)&&(blue>green)&&(blue>cyan)&&(blue>red)&&(blue>violet)&&(blue>yellow)&&(blue>white)) color_code<=VIOLET;
              else if((green>black)&&(green>blue)&&(green>cyan)&&(green>red)&&(green>violet)&&(green>yellow)&&(green>white)) color_code<=RED;
              else if((cyan>black)&&(cyan>blue)&&(cyan>green)&&(cyan>red)&&(cyan>violet)&&(cyan>yellow)&&(cyan>white)) color_code<=GREEN;
              else if((red>black)&&(red>blue)&&(red>green)&&(red>cyan)&&(red>violet)&&(red>yellow)&&(red>white)) color_code<=BLUE;
              else if((violet>black)&&(violet>blue)&&(violet>green)&&(violet>cyan)&&(violet>red)&&(violet>yellow)&&(violet>white)) color_code<=YELLOW;
              else if((yellow>black)&&(yellow>blue)&&(yellow>green)&&(yellow>cyan)&&(yellow>red)&&(yellow>violet)&&(yellow>white)) color_code<=CYAN;
              else color_code<=BLACK;
				  color_done=1'b1;
              pixel_RGB<=pixel_RGB;
				  RAM_data<=RAM_data;
				  final_code<=final_code;
				  state=WAIT;
            end
			 WAIT:
				begin
				regwrite=0;      /// NEW
				pixel_RGB<=3'b111;
				final_code<=final_code;
				  if(~read_color)
				  begin
				    color_done=1'b0;
					 state<=READ_RGB;
					 RAM_data<=RAM_data;
				  end
				  else state<=WAIT;
				end
				default:
				state=BEGIN;
		   endcase
      end
          
  end
          
endmodule