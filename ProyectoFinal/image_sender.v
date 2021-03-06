//Este código es propiedad y fué realizado por nuestros compañeros Diego Figueroa, Ferdy Larrotta y Edwin Medina en sú proyecto "ov7670_captureimage"
module image_sender (
  input clk,rst,begin_frame,image_select,
  input [2:0] pixel_data, //[5:0] pixel_data //change
  output [12:0] addr_out, //[15:0] addr_out //change
  output Tx_data);


parameter W_BEGIN_FRAME = 0, EVALUATE_END = 1,READ_PIXEL_DATA = 2, SEND_FB = 3,SEND_SB = 4, W_DONE = 5, END_FRAME = 6;

wire Tx_active,Tx_done;

reg send_byte;
reg byte_select;
reg [2:0] state;
reg [7:0] Tx_byte;
reg [12:0] addr_out_aux; //[15:0] addr_out_aux
reg [15:0] pixel_data_aux; //[15:0] pixel_data_aux

assign addr_out = addr_out_aux + (13'd3072 & {13{!image_select}}); //(15'd3072 & {15{!image_select}});



uart_tx image_sender (
      .i_Clock(clk),
      .i_Tx_DV(send_byte),
      .rst(rst),
      .i_Tx_Byte(Tx_byte),
      .o_Tx_Active(Tx_active),
      .o_Tx_Serial(Tx_data),
      .o_Tx_Done(Tx_done));


always @ ( posedge clk ) begin

  if (rst)
  begin
    state <= W_BEGIN_FRAME;
    send_byte <= 1'b0;
    Tx_byte <= 8'd0;
    pixel_data_aux <= 16'd0;
    byte_select <= 1'b0;
    addr_out_aux <= 13'd0;
  end
  else
  begin
    case (state)
      W_BEGIN_FRAME:
      begin
        send_byte <= 1'b0;
        Tx_byte <= 8'd0;
        pixel_data_aux <= 16'd0;
        byte_select <= 1'b0;
        addr_out_aux <= 13'd0;
        if (begin_frame) state <= EVALUATE_END;
        else state <= W_BEGIN_FRAME;
      end
      EVALUATE_END:
      begin
        send_byte <= 1'b0;
        byte_select <= 1'b0;
        addr_out_aux <= addr_out_aux;
        if (addr_out_aux == 13'd3072)
        begin
          pixel_data_aux <= 16'hFFFF;
          Tx_byte <= 8'hFF;
          state <= END_FRAME;
        end
        else
        begin
          pixel_data_aux <= {13'b0,pixel_data};
			 
          Tx_byte <= 8'd0;
          state <= READ_PIXEL_DATA;
        end
      end
      READ_PIXEL_DATA:
      begin
        send_byte <= send_byte;
        Tx_byte <= Tx_byte;
        pixel_data_aux <= {13'b0,pixel_data};
        byte_select <= byte_select;
        addr_out_aux <= addr_out_aux;
        state <= SEND_FB;
      end
      SEND_FB:
      begin
        send_byte <= 1'b1;
        Tx_byte <= pixel_data_aux [15:8];
        pixel_data_aux <= pixel_data_aux;
        state <= W_DONE;
        byte_select <= 1'b0;
        addr_out_aux <= addr_out_aux;
      end
      SEND_SB:
      begin
        send_byte <= 1'b1;
        if (pixel_data_aux[7:0] == 8'hFF) Tx_byte <= {pixel_data_aux [7:1],1'b0};
        else Tx_byte <= pixel_data_aux [7:0];
        pixel_data_aux <= pixel_data_aux;
        state <= W_DONE;
        byte_select <= 1'b1;
        addr_out_aux <= addr_out_aux + 1'b1;
      end
      END_FRAME:
      begin
        send_byte <= 1'b1;
        Tx_byte <= 8'hFF;
        pixel_data_aux <= 16'd0;
        state <= W_BEGIN_FRAME;
        byte_select <= 1'b1;
        addr_out_aux <= 13'd0;
      end
      W_DONE:
      begin
        send_byte <= 1'b0;
        Tx_byte <= 8'd0;
        pixel_data_aux <= pixel_data_aux;
        byte_select <= byte_select;
        addr_out_aux <= addr_out_aux;
        if (Tx_done)
        begin
          if (byte_select == 1'b0) state <= SEND_SB;
          else state <= EVALUATE_END;
        end
        else state <= W_DONE;
        end
      default:
      begin
        state <= W_BEGIN_FRAME;
        send_byte <= 1'b0;
        Tx_byte <= 8'd0;
        pixel_data_aux <= 16'd0;
        byte_select <= 1'b0;
        addr_out_aux <= addr_out_aux;
      end
    endcase
  end
end

endmodule 
