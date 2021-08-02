# Proyecto de la materia Electrónica Digital II: Explorador de un laberinto.

## Integrantes
Johan Higuera (jhiguera@unal.edu.co)
Laura Moreno (lamorenoca@unal.edu.co)
Ana Espinosa (aespinosaj@unal.edu.co)
Daniel Maldonado (dfmaldonadob@unal.edu.co)


El proyecto consiste en el diseño de un sistema capaz de recorrer un laberinto, enviar información de su trayectoria a un dispositivo en el cual se puede visualizar, y captar información del color de las paredes de su entorno, que también debe visualizarse junto a la trayectoria.

## Implementos
Para la realización de este proyecto se usaron los siguientes implementos:
- Tarjeta Arduino UNO
- Tarjeta FPGA Altera EP4CE6E22C8N
- Montura con bases de acrílico, 2 ruedas fijas y 2 ruedas tipo castor.
- 2 motores DC.
- 2 encoders ref. HC-020K
- 2 sensores ultrasonid ref. HC-SR04.
- Servomotor ref. SG90.
- Módulo bluetooth ref. HC-06.
- Puente H ref. HG7881.
- Módulo de cámara ref. OV7670.

## Particionamiento
El proyecto se separó en tres sub-sistemas "independientes" entre sí, distribuidos entre el hardware (FPGA) y el software de propósito general (Arduino) de la siguiente manera:

- Software: El Arduino UNO será el encargado de realizar todos los procesos relacionados con el sistema del movimiento y, además, se encargará de la transmisión de la información del recorrido para ser visualizada en un ordenador. A continuación se presentan los periféricos a su cargo y su función en el explorador.
  * Motores: Dos motorreductores a cada lado de la base del vehículo que servirán para movilizarlo. Estos motores contarán con alimentación externa (la tarjeta Arduino UNO puede no tener la suficiente potencia para accionarlos).
  * Encoders Ópticos: Dos encoders ópticos que obtendrán la información de posición angular de cada una de las llantas, con lo cual se puede realizar una correcta odometría del robot.
  * Recepción de la información de los sensores de ultrasonido: Uno de los sensores capta la distancia que hay entre el explorador y la pared de enfrente con el fin de detenerlo / realizar un giro. El segundo registra la distancia del explorador con la pared izquierda con el fin de reorientar su trayectoria en caso de que no sea paralela al camino del laberinto.
  * Recepción de información proveniente de la FPGA: Uso del puerto serial para recibir información de la FPGA relacionada con los datos obtenidos de la cámara.
  * Envío de información al ordenador: Se usará el módulo bluetooth para enviar la información necesaria al dispositivo que dibujará la trayectoria recorrida.
  * Control de giro del servomotor: Se hará girar el servomotor de un lado al otro, lo cual servirá para que la cámara que va sujetada a él pueda tomar fotos tanto en el lado derecho como en el lado izquierdo del robot y de este modo detectar las minas de color en las paredes laterales. 

- Hardware: Se encargará del sistema de la cámara, que tomará fotografías cada X cantidad de tiempo, captando una imagen a su derecha y a su izquierda cada vez. También se encargará de una parte del sistema de envío de datos, pues debe enviar la información de la cámara a la tarjeta Arduino.


## Manejo y adaptación de periféricos
A continuación se indica el manejo que se dio a los periféricos:

### Periféricos: sistema de movimiento
La velocidad de giro de cada motor es manejada por medio de señales PWM. Estas señales llegan a un módulo puente H, que genera una tensión dependiente del pulso PWM entrante que es enviada a las entradas del motor. El robot debe ser capaz de girar 90° o 180°, además de avanzar hacia adelante y atrás. Para cumplir con ello, se realizaron diversos “módulos” que permiten el control de la rotación de las llantas y de este mismo modo, el control del movimiento del robot. Dichos módulos se listan y explican a continuación.

#### Motores
- Accionar Motores

Para efectuar el movimiento, es importante diferenciar dos acciones que debe realizar el vehículo: el giro y el avance, pues la rotación escogida para las ruedas motorizadas del robot es diferente en cada uno de los casos. Para ello se construyó el diagrama del módulo que gestiona estas 2 acciones, además de tener el mando sobre la detención total del movimiento. Este diagrama se muestra en la figura 1.

![DiagramaJohan](https://user-images.githubusercontent.com/42346349/127759413-13e373c7-3690-4ee8-923e-aaee70407e53.jpeg)

Figura 1: Diagrama de flujo de la función Accionar Motores

Código fuente del módulo:

```cpp
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
```

- Giro

Es el encargado de enviar el comando de giro a las ruedas. Para ello toma la variable dirección y a partir de ella asigna el sentido de giro de cada rueda para lograr la acción de giro deseada. Si la dirección de movimiento es positiva, se pretende girar en sentido de las manecillas del reloj y para ello la llanta izquierda deberá avanzar adelante y la llanta derecha deberá ir hacia atrás. Si la dirección es negativa, las llantas y el giro se orientarán de manera contraria a la mencionada anteriormente. A continuación, se muestra el diagrama de bloques y el código fuente del módulo.
![Giro](https://user-images.githubusercontent.com/32202871/127775892-5adb3817-2b8d-45e9-ba94-ead2e777abe0.jpeg)

Figura 2:  Diagrama de flujo de la función Giro

```cpp
void Giro(bool direccion){
  if(direccion==true){
       int PWM_Motor_1_Adel=120;
       int PWM_Motor_1_Atras=0;
       int PWM_Motor_2_Adel=0;
       int PWM_Motor_2_Atras=150;
       bool Detener = false;
       Direccion_Motor_1 = true;
       Direccion_Motor_2 = false;
       Motor_Derecha(PWM_Motor_1_Adel, PWM_Motor_1_Atras, Detener);
       Motor_Izquierda(PWM_Motor_2_Adel, PWM_Motor_2_Atras, Detener);
    }else{
       int PWM_Motor_1_Adel=0;
       int PWM_Motor_1_Atras=120;
       int PWM_Motor_2_Adel=150;
       int PWM_Motor_2_Atras=0;
       bool Detener = false;
       Direccion_Motor_1 = false;
       Direccion_Motor_2 = true;
       Motor_Derecha(PWM_Motor_1_Adel, PWM_Motor_1_Atras, Detener);
       Motor_Izquierda(PWM_Motor_2_Adel, PWM_Motor_2_Atras, Detener);
      }
  }
```
- Avance

Genera el llamado a los módulos de las ruedas para el avance dependiendo de la dirección de este. Si la dirección es positiva, se realiza una asignación de avance a ambas ruedas, y si la dirección es negativa, se genera una asignación de retroceso.

![Avance](https://user-images.githubusercontent.com/32202871/127775910-d673da84-3564-4f49-99d8-c228b977dee2.jpeg)


Figura 3: Diagrama de flujo de la función Avance

```cpp
void Avance(bool direccion){
  if(direccion==true){
       int PWM_Motor_1_Adel=100;
       int PWM_Motor_1_Atras=0;
       int PWM_Motor_2_Adel=150;
       int PWM_Motor_2_Atras=0;
       bool Detener = false;
       Direccion_Motor_1 = true;
       Direccion_Motor_2 = true;
       Motor_Derecha(PWM_Motor_1_Adel, PWM_Motor_1_Atras, Detener);
       Motor_Izquierda(PWM_Motor_2_Adel, PWM_Motor_2_Atras, Detener);
    }else{
       int PWM_Motor_1_Adel=0;
       int PWM_Motor_1_Atras=120;
       int PWM_Motor_2_Adel=0;
       int PWM_Motor_2_Atras=150;
       bool Detener = false;
       Direccion_Motor_1 = false;
       Direccion_Motor_2 = false;
       Motor_Derecha(PWM_Motor_1_Adel, PWM_Motor_1_Atras, Detener);
       Motor_Izquierda(PWM_Motor_2_Adel, PWM_Motor_2_Atras, Detener);
      }
  }
```

- Motor_Derecha

Este módulo es el encargado final de asignar los pulsos PWM al motor de la llanta derecha.
![Motor_derecha](https://user-images.githubusercontent.com/32202871/127775885-91d3e85c-0cbf-4faa-95e6-41d9839c2fe5.jpeg)


Figura 4: Diagrama de flujo de la función Motor_Derecha

```cpp
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
```

- Motor_Izquierda

Este módulo tiene una similitud alta con el módulo anterior, asignando al motor izquierdo la tasa de PWM que debe tener para el movimiento correcto según la dirección que ha sido ordenado por algún módulo de mayor jerarquía.
![Motor_izquierda](https://user-images.githubusercontent.com/32202871/127775879-f2736798-1af1-4583-86ca-71d3c3f5da5e.jpeg)

Figura 5: Diagrama de flujo de la función Motor_Izquierda

```cpp
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
```

#### Encoders

Son los sensores encargados de registrar el giro de las llantas para conocer la distancia que ha recorrido el robot, a partir de un modelo cinemático que fue construido a partir de las condiciones particulares del vehículo, pero que no será mostrado en el presente documento por su extensión. La información proveniente de los encoders es usada y procesada en un módulo llamado “Odometría”. Cada 40 cambios de pulso de los encoders se genera una vuelta completa de la llanta, cantidad que se va acumulando para ver la cantidad de vueltas que cada llanta ha dado y calcular su velocidad angular. Con la velocidad angular de cada llanta y el modelo cinemático construido se puede obtener la velocidad del robot en sus componentes lineal y angular. Realizando una integración de las anteriores velocidades puede tenerse la información de la posición del robot (Variables Car_Distance_X y Car_Angle), en coordenadas polares. Este módulo debe ser actualizado la mayor cantidad de veces posibles, puesto que cómo se mencionó debe observar el cambio en la señal de los encoders lo cual puede suceder muy rápido, generando así errores de medición.

![DiagramaJohan(3)](https://user-images.githubusercontent.com/42346349/127759489-941f1c7e-7545-42e7-a5cc-d149e21333ed.jpeg)

Figura 6:  Diagrama de flujo de la función de Odometría

```cpp
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
```

#### Sensores ultrasonido
Son dispositivos encargados de generar una onda acústica que viaja desde el dispositivo con una dirección especifica y, que al encontrar un obstáculo, se reflejan en forma de eco y son captadas al regresar por el recibidor del dispositivo, que genera una señal eléctrica en respuesta. Dado que conocemos la velocidad a la que viajan las ondas sonoras en el aire (340 m/s aproximadamente), podemos obtener una medida indirecta de la distancia a la cual se encuentra el objeto del sensor contabilizando el tiempo que tarda el sensor en recibir la señal de eco desde el objeto. En esta ocasión, se han dispuesto dos sensores de referencia HC-SR04, uno al frente de forma que el explorador pueda detenerse antes de chocar con un obstáculo frontal, y uno a su izquierda de forma que la distancia lateral le permite reorientar su movimiento, es decir, si se aleja de la pared lateral por encima de un límite establecido, el explorador deberá orientarse levemente hacia su izquierda pues esto indica que se está alejando y no se mueve en la dirección del camino que debe seguir. Análogamente, si se acerca a la pared lateral por encima de un segundo límite establecido, el explorador deberá reorientarse levemente hacia la derecha pues su movimiento no es paralelo a la trayectoria deseada.

![image](https://user-images.githubusercontent.com/42346349/127583296-5b63e354-d69a-4f8a-88c3-2bfdf84ceba7.png)

Figura 7: Imágen del ultrasonido utilizado

Los pines Vcc y GND son los encargados de la alimentación del sensor, que funciona correctamente con 5 voltios, por lo cual pueden ser conectados a la salida de 5 V de la tarjeta Arduino UNO. 

El pin trig es el encargado de activar la señal acústica, y la detección del su eco se hace por medio de la lectura del pin Echo, que permanece prendido desde que se envía la onda acústica hasta que es detectada por el receptor. Suponiendo que la velocidad con la que el explorador se acerca al obstáculo es mucho menor que la velocidad del sonido, lo cual es cierto en nuestro caso, el tiempo en el que la onda tarda en llegar desde el sensor hasta el obstáculo será la mitad del tiempo detectado, y la distancia a la que se encuentra será este tiempo multiplicado por la velocidad del sonido especificada anteriormente.
#### Servomotor
Los servomotores etulizan un sistema de control preciso de una posición angular y lineal, esto se consigue usando la posicion actual del aspa. Se eleigieron este tipo de sensores ya que el entorno de desarrollo integrado (IDE) de Arduino, el nombre de dicho modulo es el servomotor SG90.
La librería en Arduino que permite el manejo del servomotor es la librería Servo.h, de la cual se usó la función write(), que tiene como argumento el ángulo en grados que debe recorrer el aspa desde su posición actual.
Para nuestro proyecto el servo debe permitirlw al explorador realizar un movimiento giratorio de la cámara de modo que pueda tomar imagenes a cada lado del explorador, con el fin de detectar las "minas" ubicadas en las paredes del laberinto. El tiempo establecido para capatar la imagen es de 2s; dado que el movimiento del servo es un poco brusco, se realizó una función que suavisa este movimiento dandole indicaciones de moverse un grado cada 5 milisegundos, para que se mueva un total de 180 grados, se tome la pimera imagen, y luego vuelva a recorrer esta misma distancia angular hasta volver a su posición inicial, para tomar la segunda imagen. El código utilizado se muestra a continuación.

```cpp
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
  

for(angle=180; angle>0; angle--){
  servo.write(angle);
  delay(5);
  if(angle==1){
    delay(2000); // Delay de tomar la foto (puede aumentar)
  }
  }

}
```

### Periféricos: sistema de cámara
#### Cámara
Inicialmente se realizó la configuración de este periférico, con ayuda del protocolo SCCB y el código de Muhammad Yaseen para realizar la configuración desde Arduino, disponible en el link https://gist.github.com/muhammadyaseen/75490348a4644dcbc70f. Este código envía a una dirección de registro de la cámara el valor que se desea colocar en este, un ejemplo sencillo de modificación de registros de la cámara es:
```cpp
WriteOV7670(0x12, 0x80);
```
En el anterior trozo de código se modifica el registro con dirección hexadecimal 12, y se le asigna el valor hexadecimal 80.
La úbicación del llamado a esta función dentro del código de configuración de cámara es dentro de la función "set_cam_RGB565_QCIF()".

La cámara se configuró para que adquiriera una resolución de 64x48 píxeles, a 1 frame por segundo (fps), con codifcación RGB555. A continuación se muestra la función set_cam_RGB565_QCIF() con los registros modificados allí.

```cpp
 WriteOV7670(0x12, 0x80); //reset
 
 WriteOV7670(0x12, 0x04); //COM07 Cambiado
 WriteOV7670(0x40, 0xF0); //COM15: Set RGB 555
 WriteOV7670(0x11, 0x1F);// CLKR Cambiado para 2fps
 WriteOV7670(0x0C, 0x08); //COM3: Enable Scaler
 WriteOV7670(0x3E, 0x08); //COM14: Cambiado para manual scaling
 WriteOV7670(0x70, 0x28); //SCALING_XSC: Cambiado
 WriteOV7670(0x71, 0x28); //SCALING_YSC: Cambiado
 WriteOV7670(0x72, 0x33); //SCALING_DCWCTR: Cambiado por down sampling 
 WriteOV7670(0x73, 0x03); //SCALING_PCLK_DIV: Cambiado
 WriteOV7670(0xA2, 0x10); //SCALING_PCLK_DELAY: Cambiado
```

Posterior a la realización de la configuración de la cámara se realizó el código para obtener los datos. Se realizaron dos módulos principales que realizaban la obtención de cada píxel y el calculo del color del frame. La explicación de cada módulo se explica a continuación. 

#### pixel_catcher:
Este modulo se encarga de obtener la información de cada pixel. Dado que se escogió una codificación de pixel RGB555, el píxel completo se envía en dos ciclos del PCLK, así pues este módulo se encarga de verificar todas las condiciones para obtener un píxel y además de ello, con ayuda de su máquina de estados interna, obtiene el pixel completo y autoriza al módulo color_finder a realizar sú labor. A continuación puede verse el diagrama de flujo de funcionamiento de dicho módulo y posteriormente el código con el cual fué implementado.

![pixel_catcher](https://user-images.githubusercontent.com/42346349/127759736-659c2da3-d546-43a5-b53f-317a892bdd65.jpeg)

Figura 8: Diagrama de flujo de funcionamiento del módulo píxel catcher

```verilog
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
```

#### color_finder:
Este módulo, cuando obtiene la autorización para funcionar por parte del pixel_catcher, toma el píxel tomado anteriormente y lo reduce de RGB555 a RGB111, para ello toma el bit más significativo de cada componente R, G ó B y lo asigna al nuevo píxel con resuloción RGB111, luego, se puede verificar cual es el color del píxel (Existen 8 posbilidades, para 8 colores escogidos) y se tienen registros a los cuales se les aumenta en una unidad cuando se detecta que el color corresponde al identificador del registro. Al finalizar el frame se realiza una comparación de todos los registros para ver cual es el mayor y este se toma cómo el color del frame y se muestra en la señal color_code.
A continuación se muestra el diagrama de flujo de este módulo y el código de verilog con el cual se implementó.

![color_finder](https://user-images.githubusercontent.com/42346349/127759766-b5584411-7fb8-4931-b7e8-15e18124fa94.jpeg)

Figura 8: Diagrama de flujo del módulo color_finder

```verilog
module color_finder(input [14:0]pixel_data,
       input clk,read_color, rst,
       output reg [2:0]color_code=3'b111,
		 output reg regwrite,
		 output reg [14:0]RAM_data);
  
  parameter BEGIN=0, READ_RGB=2'd1, CHOOSE_COLOR=2'd2, WAIT=2'd3;
  parameter BLACK=3'b000, BLUE=3'b001, GREEN=3'b010, CYAN=3'b011, RED=3'b100, VIOLET=3'b101, YELLOW=3'b110, WHITE=3'b111;
  
  reg [14:0] black,blue,green,cyan,red,violet,yellow,white;
  reg [1:0]state;
  reg [2:0]pixel_RGB;
  reg color_done;
    
  always @ (posedge clk) begin
    if(rst)begin
      state=BEGIN;
      {black,blue,green,cyan,red,violet,yellow,white}<=120'b0;
      color_code<=color_code; // default frame color is white 
      pixel_RGB<=3'b111; // default pixel color is white 
		color_done=0;
		regwrite=0;
		RAM_data=15'b0;
    end
    else
      begin
        case(state)
		    BEGIN:
			   begin
				  state=READ_RGB;
              {black,blue,green,cyan,red,violet,yellow,white}<=120'b0;
              color_code<=color_code; // default frame color is white 
              pixel_RGB<=3'b111; // default pixel color is white 
		        color_done=0;
				  regwrite=0;
				  RAM_data=15'b0;
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
					if(pixel_data[14]) pixel_RGB[2]=1'b1;
                else pixel_RGB[2]=1'b0;
                if(pixel_data[9]) pixel_RGB[1]=1'b1;
                else pixel_RGB[1]=1'b0;
                if(pixel_data[4]) pixel_RGB[0]=1'b1;
                else pixel_RGB[0]=1'b0;
					
					regwrite=1;      /// VA EN UNO
					RAM_data=pixel_data;
					
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
                  
					 color_done<=0;
                state=CHOOSE_COLOR;
              end 
              else state=READ_RGB;
            end
          CHOOSE_COLOR:
            begin
              if((black>blue)&&(black>green)&&(black>cyan)&&(black>red)&&(black>violet)&&(black>yellow)) color_code<=WHITE;
              else if((blue>green)&&(blue>cyan)&&(blue>red)&&(blue>violet)&&(blue>yellow)) color_code<=VIOLET;
              else if((green>cyan)&&(green>red)&&(green>violet)&&(green>yellow)) color_code<=RED;
              else if((cyan>red)&&(cyan>violet)&&(cyan>yellow)) color_code<=GREEN;
              else if((red>violet)&&(red>yellow)) color_code<=BLUE;
              else if((violet>yellow)&&(violet>white)) color_code<=YELLOW;
              else if(yellow>white) color_code<=CYAN;
              else color_code<=BLACK;
				  color_done<=1'b1;
              pixel_RGB<=3'b111;
				  RAM_data<=RAM_data;
				  state=WAIT;
            end
			 WAIT:
				begin
				regwrite=0;      /// NEW
				  if(~read_color)
				  begin
				    color_done<=1'b0;
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

```

### Nota importante
Es ímportante mencionar que el módulo de verilog image_sender, incluído en el proyecto de cámara de quartus, junto con el código de processing usado para ver la imágen en el computador, no fueron realizados por nuestro grupo, sino por nuestros colegas Diego Figueroa, Ferdy Larrotta y Edwin Medina, en sú proyecto de verilog"ov7670_captureimage". Mediante esto se pudo verificar la baja calidad de la cámara y su sensibilidad a la luz.

Para trabajos futuros con esta cámara se sugiere realizar un estudio de los registros de configuración de la cámara y en especial aquellos que realizan control del color, puesto que talvez con ayuda de esto se logre obtener una mejor imágen, más clara y que permita ver correctamente el exterior.

### Periféricos: sistema de mapeo y envío
De otro lado también se contruyó una App en Appinventor, que a través de datos obtenidos por bluetooth lograba dibujar la trayectoria del explorador, para ello contaba principalmente con tres "módulos" bien definidos, uno que realizaba la obtención de datos en la App (Cómo dato curioso, AppInventor solo tomaba el dato correctamente sí se obtenía cómo texto, No cómo nnúmero de 1, 2 o 4 bits, para pasar dicho texto a número para usarlo luego, se dividió la variable de texto entre 1 y se asignó a una variable numérica), otro decodificaba esta información (Para enviar datos se usó un byte en el cual se codificaba la información de una manera específica, para ver más, observar el punto 5 de la documentación) y finalmente un módulo que tomaba los datos decodificados, y realizaba el dibujo de la trayectoria del robot. A continuación se muestra un diagrama sencillo del funcionamiento de la App.
![Diagrama App](https://user-images.githubusercontent.com/32202871/127775609-1aa96a6e-d1e0-43e0-aa4d-99fd055b89cf.jpeg)

XXXXXXX Diagrama de funcionamiento de la App

## Unión de sistemas y resultado final
Posteior a la realización de cada uno de los sistemas se procedió a realizar la integración de estos, dentro del Soc está el sistema de la cámara y el de movimiento, estos 2 sistemas se comunican por medio de las señales arduinoSignal (Señal que indica a la FPGA el momento en el cual el sistema de movimiento requiere el color del frame) y datoAPP (Señal a través de la cual la FPGA, por protocolo UART envía la información del frame detectado, color y "figura") y, el SoC se comunica con un sistema externo, el de Mapeo y envío, envío, nuestro SoC se comunica con este sistema a parir de la señal datoAPP, que es enviada por el sistema de movimiento a partir de protocolo UART por bluetooth, donde el sistema de mapeo y envío obtiene el dato enviado y realiza el dibujo de la trayectoria del explorador. A continuación puede verse un diagrama general de todos los sistemas de nuestro explorador, junto con los periféricos con los cuales cuenta.
![Union de sistemas](https://user-images.githubusercontent.com/32202871/127775644-5457d461-97a0-4da4-af2c-e7cc8eab0e04.jpeg)

XXXXXXX Diagrama de funcionamiento del explorador, unión de los sistemas en el SoC
