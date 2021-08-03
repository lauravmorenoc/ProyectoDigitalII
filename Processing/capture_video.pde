//Este código es propiedad y fué realizado por nuestros compañeros Diego Figueroa, Ferdy Larrotta y Edwin Medina en sú proyecto "ov7670_captureimage"

import processing.serial.*;

Serial myPort;    // The serial port
byte [] image = new byte [6145]; //cambiar (64*48*2)+1=(horiz*vert*2)+1 // para 1 byte por pixel: (horiz*vert)+1
int end_frame = 0xFF;      // frame end flag

float newWidth = 64; //cambiar
float newHeight = 48; //cambiar
float dx,dy;
boolean update = false;
void setup() {
  // List all the available serial ports:
  //printArray(Serial.list());
  // Open the port you are using at the rate you want:
  //myPort = new Serial(this, Serial.list()[2], 115200); // Antes 8
  myPort = new Serial(this, "COM6", 115200);
  myPort.bufferUntil(end_frame);
  
  size(640, 480); // Que sean múltiplos de la resolución
  dx = width/newWidth;
  dy = height/newHeight;
  background(0);
  noStroke();
}

void draw() {
  if (update){
    if (image!=null){
      display(image);
      update = false;
    }
  }
}

void serialEvent(Serial p) {
  p.readBytes(image);
  //print(image); // Colocado
  update = true;
}

// Original: Diego


/*
void display(byte[] image) {
    
  int px = 0;
  int FB,SB,pixel_Data;
  float R,G,B;
  for (int i=0; i<newHeight; i++) {
    for (int j=0; j<newWidth; j++) {
      FB = image[px] & 0xff; 
      SB = image[px+1] & 0xff; 
      pixel_Data = (FB<<8) + SB;

      
      R = (pixel_Data>>10) & 0x0000001F;
      R = map(R,0,31,0,255); 
      print(R);

      G = (pixel_Data>>5)  & 0x0000001F; 
      G = map(G,0,31,0,255); 
      print(G);

      B = (pixel_Data>>0)  & 0x0000001F; 
      B = map(B,0,31,0,255); 
      print(B);
      print("-");

      fill(R,G,B);
      rect(j*width/newWidth, i*height/newHeight, dx, dy);
      
        px += 2; 
    }
  }
}
*/


// Modificada


void display(byte[] image) {
  
  int two_pixels_read=0;
  
  int px = 0;
  int FB,SB,pixel_Data;
  float R,G,B;
  for (int i=0; i<newHeight; i++) {
    for (int j=0; j<newWidth; j++) {
      FB = image[px] & 0xff; 
      SB = image[px+1] & 0xff; 
      pixel_Data = (FB<<8) + SB;
           
      
      // Para RGB 111, 2pixeles x 2bytes: [0000000000,R1,G1,B1,R2,G2,B2]
    //  pixel_Data=image[px] & 0xff;  // No hay necesidad de tomar 2 bytes
      
     R = (pixel_Data>>2) & 0x00000001;
     print(R);
      R = map(R,0,1,0,255);

    G = (pixel_Data>>1) & 0x00000001;
    print(G);
 
   G = map(G,0,1,0,255);

    B = (pixel_Data>>0) & 0x00000001;
    print(B);
    print("-");

    B = map(B,0,1,0,255);
      fill(R,G,B);
      rect(j*width/newWidth, i*height/newHeight, dx, dy);
      
    //  if(two_pixels_read==1){
        px += 2; 
    //    two_pixels_read=0;
    //  }
    }
  }
}
