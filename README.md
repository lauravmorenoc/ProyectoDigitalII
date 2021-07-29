# Proyecto de la materia Electrónica Digital II: Explorador de un laberinto.
El proyecto consiste en el diseño de un sistema capaz de recorrer un laberinto, enviar información de su trayectoria a un dispositivo en el cual se puede visualizar, y captar información del color de las paredes de su entorno, que también debe visualizarse junto a la trayectoria.

# Implementos
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

# Particionamiento
El proyecto se separó en tres sub-sistemas "independientes" entre sí, distribuidos entre el hardware (FPGA) y el software de propósito general (Arduino) de la siguiente manera:
- Software: El Arduino UNO será el encargado de realizar todos los procesos relacionados con el sistema del movimiento y, además, se encargará de la transmisión de la información del recorrido para ser visualizada en un ordenador. A continuación se presentan los periféricos a su cargo y su función en el explorador.
  * Motores: Dos motorreductores a cada lado de la base del vehículo que servirán para movilizarlo. Estos motores contarán con alimentación externa (la tarjeta Arduino UNO puede no tener la suficiente potencia para accionarlos).
  * Encoders Ópticos: Dos encoders ópticos que obtendrán la información de posición angular de cada una de las llantas, con lo cual se puede realizar una correcta odometría del robot.
  * Recepción de la información de los sensores de ultrasonido: Uno de los sensores capta la distancia que hay entre el explorador y la pared de enfrente con el fin de detenerlo / realizar un giro. El segundo registra la distancia del explorador con la pared izquierda con el fin de reorientar su trayectoria en caso de que no sea paralela al camino del laberinto.
  * Recepción de información proveniente de la FPGA: Uso del puerto serial para recibir información de la FPGA relacionada con los datos obtenidos de la cámara.
  * Envío de información al ordenador: Se usará el módulo bluetooth para enviar la información necesaria al dispositivo que dibujará la trayectoria recorrida.
  * Control de giro del servomotor: Se hará girar el servomotor de un lado al otro, lo cual servirá para que la cámara que va sujetada a él pueda tomar fotos tanto en el lado derecho como en el lado izquierdo del robot y de este modo detectar las minas de color en las paredes laterales. 

- Hardware: Se encargará del sistema de la cámara, que tomará fotografías cada X cantidad de tiempo, captando una imagen a su derecha y a su izquierda cada vez. También se encargará de una parte del sistema de envío de datos, pues debe enviar la información de la cámara a la tarjeta Arduino.


# Manejo y adaptación de periféricos
A continuación se indica el manejo que se dio a los periféricos:

# Periféricos: sistema de movimiento
La velocidad de giro de cada motor es manejada por medio de señales PWM. Estas señales llegan a un módulo puente H, que genera una tensión dependiente del pulso PWM entrante que es enviada a las entradas del motor. El robot debe ser capaz de girar 90° o 180°, además de avanzar hacia adelante y atrás. Para cumplir con ello, se realizaron diversos “módulos” que permiten el control de la rotación de las llantas y de este mismo modo, el control del movimiento del robot. Dichos módulos se listan y explican a continuación.

- Accionar Motores
Para efectuar el movimiento, es importante diferenciar dos acciones que debe realizar el vehículo: el giro y el avance, pues la rotación escogida para las ruedas motorizadas del robot es diferente en cada uno de los casos. Para ello se construyó el diagrama del módulo que gestiona estas 2 acciones, además de tener el mando sobre la detención total del movimiento. Este diagrama se muestra en la figura 1.

![IMG_20191109_172336](https://user-images.githubusercontent.com/42346349/127579471-25eb564b-4f89-4962-840c-3c616430f884.jpg)


Figura 1: Diagrama de flujo de la función Accionar Motores



