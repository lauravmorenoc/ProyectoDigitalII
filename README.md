# Proyecto de la materia Electrónica Digital II: Explorador de un laberinto.
El proyecto consiste en el diseño de un sistema capaz de recorrer un laberinto, enviar información de su trayectoria a un dispositivo en el cual se puede visualizar, y captar información del color de las paredes de su entorno, que también debe visualizarse junto a la trayectoria.

## Implementos
Para la realización de este proyecto se usaron los siguientes implementos:
- Tarjeta Arduino UNO
- Tarjeta FPGA Altera EP4CE6E22C8N
- Montura con bases de acrílico, 2 llantas fijas y 2 llantas movibles.
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

Figura 6:  Diagrama de flujo de la función de Odometría

```cp
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

Los pines Vcc y GND son los encargados de la alimentación del sensor, que funciona correctamente con 5 voltios, por lo cual pueden ser conectados a la salida de 5 V de la tarjeta Arduino UNO. 

El pin trig es el encargado de activar la señal acústica, y la detección del su eco se hace por medio de la lectura del pin Echo, que permanece prendido desde que se envía la onda acústica hasta que es detectada por el receptor. Suponiendo que la velocidad con la que el explorador se acerca al obstáculo es mucho menor que la velocidad del sonido, lo cual es cierto en nuestro caso, el tiempo en el que la onda tarda en llegar desde el sensor hasta el obstáculo será la mitad del tiempo detectado, y la distancia a la que se encuentra será este tiempo multiplicado por la velocidad del sonido especificada anteriormente.

