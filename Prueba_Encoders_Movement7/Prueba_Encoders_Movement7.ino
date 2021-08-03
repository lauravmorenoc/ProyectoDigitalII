#include <SoftwareSerial.h>
SoftwareSerial BT1(A1,A0); // RX, TX recordar que se cruzan

int Motor_1_Adel = 11; //Motor 1 es el motor que está a la derecha
int Motor_1_Atras = 10;
int Motor_2_Adel = 6;
int Motor_2_Atras = 5;
bool Detener_Motor_1 = false; //True para detener
bool Detener_Motor_2 = false; //True para detener
bool Direccion_Motor_1 = true; //True para ir adelante, False atras
bool Direccion_Motor_2 = true; //True para ir adelante, False atras
double Encoder_Der = 2;
double Encoder_Izq = 3;
int i = 0;
bool state_change = false;
int Max_delay;
float Total_ran;
unsigned long tiempoAnterior;
double tiempoActual;
double W_Der;
double W_Izq;
double Wheels_diameter;
double Wheels_perimeter;
double Car_Velocity;
double Car_W_angle;
double Car_distance_X;
double Car_angle;
double Par_bef_mov=0;

// Variables de función send_10_pulse();
int datoAPP=0;
bool tenCentDone=LOW;
int dataFPGA=0;
int giroInfo=0; // 0 no gira, 1 gira a izquierda, 2 gira a derecha

// Variable de salida para la FPGA
const int arduinoSignal=4; // PIN PARA FPGA -> CAMBIAR SEGUN PIN LIBRE

int k=0;

////////
int distance_residue=0; // Dado que Car_distance_X se actualiza, distance_residue guarda la distancia adicional a los 10 cm
float Car_distance_residue=0;
int counter=0;

int ranura_Izq;
boolean ranura_Ant_Izq;
double vueltas_Izq;
double vueltas_Izq_Ant;
boolean turnstate_Izq;

int ranura_Der;
boolean ranura_Ant_Der;
double vueltas_Der;
double vueltas_Der_Ant;
boolean turnstate_Der;

int State=0;
int Last_state=0;
//int U1=0;
//int U2=0;
int E=0;
unsigned long T_max=180000; // 3 segundos

///////////////////// VARIABLES SERVO Y ULTRASONIDO //////////////////////

// Servo
#include <Servo.h>
Servo servo;
int angle = 0;
int servoPin=7;
bool action=LOW;

// Ultrasonido U1 (al frente)
int Trig1=12;
int Echo1=13;
//int led1=8;  // LED para demostración, luego solo será una señal
int read1=0;

// Ultrasonido U2 (lado izquierdo)
int Trig2=8;
int Echo2=9;
//int led2=9;  // LED para demostración, luego solo será una señal
int read2=0;

//////////////////////////////////////////////////////////////////////////

void setup() {

  // Servo + ultrasonidos
  pinMode(Trig1,OUTPUT);
  pinMode(Echo1, INPUT);

  pinMode(Trig2,OUTPUT);
  pinMode(Echo2, INPUT);

  pinMode(arduinoSignal, OUTPUT);

  servo.attach(servoPin);
  servo.write(angle);
  ///////////////////////
  
  //iniciamos el puerto serie
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  BT1.begin(9600);
  Serial.begin(9600);

  ranura_Izq = (int)(0);
  ranura_Ant_Izq = (boolean)(false);
  Total_ran = (float)(40.0);
  vueltas_Izq = (double)(0.0);
  vueltas_Izq_Ant = (double)(0.0);
  turnstate_Izq = (boolean)(false);

  ranura_Der = (int)(0);
  ranura_Ant_Der = (boolean)(false);
  vueltas_Der = (double)(0.0);
  vueltas_Der_Ant = (double)(0.0);
  turnstate_Der = (boolean)(false);

  tiempoAnterior = (unsigned long) (0);
  tiempoActual = (double) (0.0);
  W_Der = (double) (0.0);
  W_Izq = (double) (0.0);
  Wheels_diameter = (double) (0.065); //Diametro en metros
  Wheels_perimeter = (double) (0.2042); //Perimetro en metros
  Car_Velocity = (double) (0.0);
  Car_W_angle = (double) (0.0);
  Car_distance_X = (double) (0.0);
  Car_angle = (double) (0.0);

  Max_delay = (int)(5);
}

unsigned long tiempoInicial = micros();

/////////////////////////////////////////////////////////FUNCIONES DE MOVIMIENTO///////////////////////////////////////////////////////////
void Motor_Derecha(int PWM_Motor_1_Adel, int PWM_Motor_1_Atras, bool Detener) {
  if (Detener == true) {
    PWM_Motor_1_Adel = 0;
    PWM_Motor_1_Atras = 0;
    analogWrite(Motor_1_Adel, PWM_Motor_1_Adel);
    analogWrite(Motor_1_Atras, PWM_Motor_1_Atras);
    return;
  }else{
    analogWrite(Motor_1_Adel, PWM_Motor_1_Adel);
    analogWrite(Motor_1_Atras, PWM_Motor_1_Atras);
    return;
    }
}
void Motor_Izquierda(int PWM_Motor_2_Adel, int PWM_Motor_2_Atras, bool Detener) {
  if (Detener == true) {
    PWM_Motor_2_Adel = 0;
    PWM_Motor_2_Atras = 0;
    analogWrite(Motor_2_Adel, PWM_Motor_2_Adel);
    analogWrite(Motor_2_Atras, PWM_Motor_2_Atras);
    return;
  }else{
    analogWrite(Motor_2_Adel, PWM_Motor_2_Adel);
    analogWrite(Motor_2_Atras, PWM_Motor_2_Atras);
    return;
    }
}
void Avance(bool direccion){
  if(direccion==true){
       int PWM_Motor_1_Adel=255;
       int PWM_Motor_1_Atras=0;
       int PWM_Motor_2_Adel=255;
       int PWM_Motor_2_Atras=0;
       bool Detener = false;
       Direccion_Motor_1 = true;
       Direccion_Motor_2 = true;
       Motor_Derecha(PWM_Motor_1_Adel, PWM_Motor_1_Atras, Detener);
       Motor_Izquierda(PWM_Motor_2_Adel, PWM_Motor_2_Atras, Detener);
    }else{
       int PWM_Motor_1_Adel=0;
       int PWM_Motor_1_Atras=255;
       int PWM_Motor_2_Adel=0;
       int PWM_Motor_2_Atras=255;
       bool Detener = false;
       Direccion_Motor_1 = false;
       Direccion_Motor_2 = false;
       Motor_Derecha(PWM_Motor_1_Adel, PWM_Motor_1_Atras, Detener);
       Motor_Izquierda(PWM_Motor_2_Adel, PWM_Motor_2_Atras, Detener);
      }
  }
void Giro(bool direccion){
  if(direccion==true){
       int PWM_Motor_1_Adel=255;
       int PWM_Motor_1_Atras=0;
       int PWM_Motor_2_Adel=0;
       int PWM_Motor_2_Atras=255;
       bool Detener = false;
       Direccion_Motor_1 = true;
       Direccion_Motor_2 = false;
       Motor_Derecha(PWM_Motor_1_Adel, PWM_Motor_1_Atras, Detener);
       Motor_Izquierda(PWM_Motor_2_Adel, PWM_Motor_2_Atras, Detener);
    }else{
       int PWM_Motor_1_Adel=0;
       int PWM_Motor_1_Atras=255;
       int PWM_Motor_2_Adel=255;
       int PWM_Motor_2_Atras=0;
       bool Detener = false;
       Direccion_Motor_1 = false;
       Direccion_Motor_2 = true;
       Motor_Derecha(PWM_Motor_1_Adel, PWM_Motor_1_Atras, Detener);
       Motor_Izquierda(PWM_Motor_2_Adel, PWM_Motor_2_Atras, Detener);
      }
  }
int Accionar_motores (bool accionar, double magnitud, double Car_distance_X, double Car_angle, double Par_bef_mov, bool detener ){ //Accionar : true-avance; false-giro
  if(detener==true){
    Motor_Derecha(0,0,true);
    Motor_Izquierda(0,0,true);
    return 1;
  }
  if(accionar==true){//Avance
    bool direccion=true;
    if(magnitud<0){
      direccion=false;
    }
    if((direccion==true)&&((Car_distance_X-Par_bef_mov)>magnitud)){
      Motor_Derecha(0,0,true);
      Motor_Izquierda(0,0,true);
      return 1;
      }
    if((direccion==false)&&(Car_distance_X-Par_bef_mov<magnitud)){
      Motor_Derecha(0,0,true);
      Motor_Izquierda(0,0,true);
      return 1;
      }
    Avance(direccion);
    }
   else{
    bool direccion=true;
    if(magnitud<0){
      direccion=false;
    }
    if((direccion==true)&&((Car_angle-Par_bef_mov)>magnitud)){
      Motor_Derecha(0,0,true);
      Motor_Izquierda(0,0,true);
      return 1;
      }
    if((direccion==false)&&((Car_angle-Par_bef_mov)<magnitud)){
      Motor_Derecha(0,0,true);
      Motor_Izquierda(0,0,true);
      return 1;
      }
    Giro(direccion); 
    }
   return 0;
  } 

///////////////////////////////////////////////////////FUNCION DE ODOMETRIA///////////////////////////////////////////////////////////////
void Odometria (unsigned long tiempoInicial){
    Encoder_Der = digitalRead(2);
    Encoder_Izq = digitalRead(3);
  if (Encoder_Izq != ranura_Ant_Izq) {
   ranura_Ant_Izq = Encoder_Izq;
   ranura_Izq = ranura_Izq + 1;
   vueltas_Izq = ranura_Izq / Total_ran;
  }
  if (vueltas_Izq != turnstate_Izq) {
    turnstate_Izq = vueltas_Izq;
  }

  if (Encoder_Der != ranura_Ant_Der) {
    ranura_Ant_Der = Encoder_Der;
    ranura_Der = ranura_Der + 1;
    vueltas_Der = ranura_Der / Total_ran;
  }
  if (vueltas_Der != turnstate_Der) {
    turnstate_Der = vueltas_Der;
  }

  if (i == 0) {
    tiempoActual = micros() - tiempoInicial;
    i=i+1;
  } else {
    tiempoActual = micros() - tiempoAnterior;
  }
  tiempoAnterior = micros();

  W_Izq = (vueltas_Izq - vueltas_Izq_Ant) * 2 * 3.1416 / (tiempoActual * 0.000001); //Velocidad en rad/s
  W_Der = (vueltas_Der - vueltas_Der_Ant) * 2 * 3.1416 / (tiempoActual * 0.000001); //Velocidad en rad/s

  if (Direccion_Motor_1 == false) {
    W_Der = -W_Der;
  }
  if (Direccion_Motor_2 == false) {
    W_Izq = -W_Izq;
  }

  Car_Velocity = (Wheels_diameter * W_Izq + Wheels_diameter * W_Der) / 2; //Velocidad del robot en x [m/s]
  Car_W_angle = (-Wheels_diameter * W_Izq + Wheels_diameter * W_Der) / ( Wheels_perimeter); //Velocidad de rotación del robot en sentido contrario a manecillas [rad/s]

  Car_distance_X = Car_distance_X + Car_Velocity * tiempoActual * 0.000001; //[m]
  Car_angle = Car_angle + Car_W_angle * tiempoActual * 0.000001; //[rad]

  vueltas_Der_Ant = vueltas_Der;
  vueltas_Izq_Ant = vueltas_Izq;
  return 0;
}

////////////////////////////////////////////////////////MAQUINA DE ESTADOS////////////////////////////////////////////////////////////////
int State_Machine(int Last_state, int U1, int U2, int E, unsigned long T_max){
  int Next_state=0;
  switch (Last_state){
      case 0:
        Next_state=1;
        state_change=true;
      break;

      case 1:
        if(millis()>=T_max){
          Next_state=2;
          state_change=true;
          break;
        }else if((read1==0)&&(read2==0)&&(E==0)){
          Next_state=1;
          break;
        }else if((read1==1)||(read2!=0)||(E==1)){
          Next_state=3;
          state_change=true;
          break;
        }
       break; 

      case 2:
        Next_state=2;
      break;

      case 3:
        if(E==1){
          Next_state=4;
          state_change=true;
          break;
        }else if((read1==1)&&((read2==0)||(read2==2))&&(E==0)){
          Next_state=5;
          state_change=true;
          break;
        }else if((read1==1)&&(read2==1)&&(E==0)){
          Next_state=7;
          state_change=true;
          break;
        }else if((read1==0)&&(read2==1)&&(E==0)){
          Next_state=8;
          state_change=true;
          break;
        }else if((read1==0)&&(read2==2)&&(E==0)){
          Next_state=9;
          state_change=true;
          break;
        }
      break;

      case 4:
        Next_state=1;
        state_change=true;
      break;

      case 5:
        if(E==1){
          Next_state=6;
          state_change=true;
        break;
        }else{
          Next_state=5;
          break;  
        }
      break;

      case 6:
        if(E==1){
          Next_state=1;
          state_change=true;
          break;
        }else{
          Next_state=6;
          break;
          }
      break;

      case 7:
        if(E==1){
            Next_state=1;
            state_change=true;
            break;
        }else{
            Next_state=7;
        break;
            }
      break;

      case 8:
      if(E==1){
          Next_state=10;
          state_change=true;
          break;
      }else{
          Next_state=8;
          break;
      }
      break;

      case 9:
      if(E==1){
          Next_state=10;
          state_change=true;
          break;
        }else{
          Next_state=9;
          break;
          }
      break;

      case 10:
      if(E==1){
          Next_state=1;
          state_change=true;
          break;
        }else{
          Next_state=10;
          break;
          }
      break;   
    }
  return Next_state;
  }

///////////////////////////////////////////////////// ACCIONES DE MOVIMIENTO//////////////////////////////////////////////////////////////
void Mente_de_todo(int State){
  
  switch(State){

    case 1:

      E=0;
      //Espacio para llamar a funciones de ultrasonido
      U1();
      U2();
      if(state_change==true){ //Es la primera vez que se va a mover el carro
        Par_bef_mov=Car_distance_X; 
        state_change=false;
        if(Last_state==7){
            giroInfo=1;
          }
          else if (Last_state==5){
            giroInfo=2;
          } 
          else {
            giroInfo=0;
          }
      }
      if((Last_state==6)||(Last_state==10)){
        E=1;
      }else{
        E=Accionar_motores (true,0.1,Car_distance_X,Car_angle,Par_bef_mov,false); 
      }
      Odometria (tiempoInicial);
    break;

    case 2:
      state_change=false;
      E=Accionar_motores (true,0,Car_distance_X,Car_angle,Par_bef_mov,true);// Detiene los motores
    break;
    
    case 3:
      //BT -> Car_distance_X-Par_bef_mov
      if(state_change==true){ //Es la primera vez que se va a mover el carro
        Par_bef_mov=0; 
        state_change=false;
      }
      k=Accionar_motores (true,0,Car_distance_X,Car_angle,Par_bef_mov,true);
      Odometria (tiempoInicial);
      //**No se si llamar aqui los ultrasonidos otravez aca, como para tener la informacion actualizada lo más posible o si es suficiente con llamarlos en el 1
    break;

    case 4: 
      if(state_change==true){ //Es la primera vez que se va a mover el carro
        moveRadar(); 
        //delay(5000);
        Par_bef_mov=0; 
        state_change=false;
      }
      //BT(01)
      //BT(Pulso a FPGA)
      //Acá va el codigo para mover el servomotor y decirle a la FPGA que puede tomar la foto (Sistema de cámara)
      Odometria (tiempoInicial);
    break;

    case 5:
    if(state_change==true){ //Es la primera vez que se va a mover el carro
      Par_bef_mov=Car_angle; 
      state_change=false;
    }
    E=0;
    E=Accionar_motores (false,-3.1416/2,Car_distance_X,Car_angle,Par_bef_mov,false);
    Odometria (tiempoInicial);
    break;

    case 6:
    if(state_change==true){ //Es la primera vez que se va a mover el carro
      Par_bef_mov=Car_distance_X; 
      state_change=false;
      if (Last_state==5){
            giroInfo=2;
      } 
    }
    E=0;
    E=Accionar_motores (true,0.1,Car_distance_X,Car_angle,Par_bef_mov,false);       //CAMBIAR QUE DISTANCIA ES
    Odometria (tiempoInicial);
    break;

    case 7:
    if(state_change==true){ //Es la primera vez que se va a mover el carro
      Par_bef_mov=Car_angle; 
      state_change=false;
    }
    E=0;
    E=Accionar_motores (false,3.1416/2,Car_distance_X,Car_angle,Par_bef_mov,false);
    Odometria (tiempoInicial);
    break;

    case 8:
    if(state_change==true){ //Es la primera vez que se va a mover el carro
      Par_bef_mov=Car_angle; 
      state_change=false;
    }
    E=0;
    E=Accionar_motores (false,3.1416/12,Car_distance_X,Car_angle,Par_bef_mov,false);
    Odometria (tiempoInicial);
    break;

    case 9:
    if(state_change==true){ //Es la primera vez que se va a mover el carro
      Par_bef_mov=Car_angle; 
      state_change=false;
    }
    E=0;
    E=Accionar_motores (false,-3.1416/12,Car_distance_X,Car_angle,Par_bef_mov,false);
    Odometria (tiempoInicial);
    break;

    case 10:
    if(state_change==true){ //Es la primera vez que se va a mover el carro
      Par_bef_mov=Car_distance_X; 
      state_change=false;
    }
    E=0;
    E=Accionar_motores (true,0.15,Car_distance_X,Car_angle,Par_bef_mov,false);
    Odometria (tiempoInicial);
    break; 

  }
}

void U1(){

  long duracion;
  long distancia;
  digitalWrite(Trig1, LOW); 
  delayMicroseconds(4); 
  digitalWrite(Trig1, HIGH);// Envía onda acústica, 8 pulsos de 40uHz
  delayMicroseconds(10);
  digitalWrite(Trig1, LOW);
  
  duracion=pulseIn(Echo1, HIGH); // El pin Echo queda positivo hasta que recibe la onda acústica, luego se apaga. Este pin lee el tiempo que demora.
  duracion=duracion/2; // Solo necesitamos el tiempo de ida, no de ida y vuelta -> tener en cuenta después que el carro se mueve (despreciable en realidad)
  distancia=duracion/29; // distancia a la cual se encuentra el objeto en cm (regla de tres con velocidad de sonido)

  if(distancia<20){
    //digitalWrite(led1, HIGH);
    read1=1;
  }else{
   //digitalWrite(led1, LOW);
   read1=0;
  }
    
}

void U2(){

  int lower_limit=10; //in cm
  int upper_limit=15;;  //in cm
  long duracion;
  long distancia;
  digitalWrite(Trig2, LOW); 
  delayMicroseconds(4); 
  digitalWrite(Trig2, HIGH);// Envía onda acústica, 8 pulsos de 40uHz
  delayMicroseconds(10);
  digitalWrite(Trig2, LOW);
  
  duracion=pulseIn(Echo2, HIGH); // El pin Echo queda positivo hasta que recibe la onda acústica, luego se apaga. Este pin lee el tiempo que demora.
  duracion=duracion/2; // Solo necesitamos el tiempo de ida, no de ida y vuelta -> tener en cuenta después que el carro se mueve (despreciable en realidad)
  distancia=duracion/29; // distancia a la cual se encuentra el objeto en cm (regla de tres con velocidad de sonido)

  if((distancia>=lower_limit) && (distancia<=upper_limit)){
    //digitalWrite(led2, LOW);
    read2=0;
  }else
  if(distancia<lower_limit){
   //digitalWrite(led2, HIGH);
   read2=2;
  } else
  if(distancia>upper_limit){
    //digitalWrite(led2, HIGH);
   read2=1;
  }
    
}

void moveRadar(){
  int dataFPGA1=0;
  int dataFPGA2=0;
  for(angle=0; angle<180; angle++){
  
  servo.write(angle);
  delay(5);
  if(angle==179){
    delay(2000); // Delay de tomar la foto (puede aumentar)
  }
}

digitalWrite(arduinoSignal, HIGH);


  if (BT1.available() > 0) {
    // read the incoming byte:
    dataFPGA1 = BT1.read(); // Lee FPGA
    Serial.println(dataFPGA1-8, BIN);
  }

  digitalWrite(arduinoSignal, LOW);
  

for(angle=180; angle>0; angle--){
  servo.write(angle);
  delay(5);
  if(angle==1){
    delay(2000); // Delay de tomar la foto (puede aumentar)
  }
  }

  digitalWrite(arduinoSignal, HIGH);


  if (BT1.available() > 0) {
    // read the incoming byte:
    dataFPGA2 = BT1.read(); // Lee FPGA
        Serial.println(dataFPGA2-8, BIN);
  }

  digitalWrite(arduinoSignal, LOW);

  if((dataFPGA1-8)!=0){
    dataFPGA=dataFPGA1;
    } else if((dataFPGA2-8)!=0){
      dataFPGA=dataFPGA2;
      } else{
        dataFPGA=8;
        }

}

void send_10_pulse(){
  Car_distance_residue=Car_distance_X-(counter*0.10);
  if((Car_distance_residue+distance_residue)>=0.10){ 
    counter=counter+1;
   // Serial.println("10 cm rec");
   // Serial.print(",   ");
    tenCentDone=HIGH;
    distance_residue=((Car_distance_residue+distance_residue)-0.10);
    } else {
        tenCentDone=LOW;
      }
    
   if((dataFPGA-8)==0){ //cambiar depronto
      if(tenCentDone){
        datoAPP+=64;
        BT1.println(datoAPP);
        } else if(giroInfo==1) { //izquierda
          datoAPP+=32;
          BT1.println(datoAPP);
          giroInfo=0;
        } else if(giroInfo==2){ //derecha
          BT1.println(0);
          giroInfo=0;
          }
    } else {
      datoAPP+=dataFPGA;
      if(tenCentDone){
        datoAPP+=64;
        BT1.println(datoAPP);
        } else if(giroInfo==1){
          datoAPP+=32;
          giroInfo=0;
          BT1.println(datoAPP);
          } else if (giroInfo==2){
            BT1.println(0); //datoAPP
            giroInfo=0;
            }
      }
    datoAPP=0;
    tenCentDone=LOW;
}

void loop() {

send_10_pulse();
State=State_Machine(Last_state,U1, U2, E, T_max);
Mente_de_todo(State);
Odometria (tiempoInicial);
Last_state=State;
  //Recibe información de FPGA y modifica informacion a enviar por BT
/*

    digitalWrite(arduinoSignal, HIGH);

  if (BT1.available() > 0) {
    // read the incoming byte:
    dataFPGA = BT1.read(); // Lee FPGA
    Serial.println(dataFPGA-8, BIN);
  }
    digitalWrite(arduinoSignal, LOW);
*/
}
