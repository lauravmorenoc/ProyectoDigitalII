
module pclk_driver (
  input rst,pclk_in,clk,
  output reg pclk_out);


  parameter DETECT_RISING_EDGE = 1'b0, WAIT = 1'b1;

  reg state = DETECT_RISING_EDGE;

  always @ ( posedge(clk) ) begin

    if ( rst )
    begin
      state = DETECT_RISING_EDGE;
      pclk_out = 1'b0;
    end

    case (state)
      DETECT_RISING_EDGE:
      begin
        if ( pclk_in )
        begin
          pclk_out = 1'b1;
          state = WAIT;
        end
        else
        begin
          pclk_out = 1'b0;
          state = DETECT_RISING_EDGE;
        end
      end
      WAIT:
      begin
        pclk_out = 1'b0;
        if ( pclk_in )
        begin
          state = WAIT;
        end
        else
        begin
          state = DETECT_RISING_EDGE;
        end
      end
      default:
      begin
      pclk_out = 1'b0;
      state = DETECT_RISING_EDGE;
      end
    endcase
  end


endmodule
