#include <Servo.h> // Incluimos la librería para los servomotores
#include<math.h> // Incluimos la librería para los calculos matematicos

//************** Definicion motor CNC ********************

#define STEP 4      // pin STEP de A4988 a pin 4
#define DIR 5       // pin DIR de A4988 a pin 5
#define PIN_2 2     // Final de carrera 1
#define PIN_3 3     // Final de carrera 2

//********* Definicion de los servos **********************

#define pin_base 11   // Servo Base
#define pin_L1 10     // Servo Hombro
#define pin_L2 9     // Servo Codo
#define pin_Pnz 8   // Servo Pinza

//********** Definicion variables de los servos ***********

Servo base; // Crea objeto para la base
Servo L1;   // Crea objeto para el Hombro
Servo L2;   // Crea objeto para el Codo
Servo Pnz;  // Crea objeto para la pinza
int ang;

//********** Definicion variables de los servos ***********

const int buttonPin = 18;   // Pin del botón Start/Stop (Se utiliza el pin 18 pq el arduino mega tiene asignado ese boton como interrupcion)
int buttonState = HIGH;     // Estado inicial del botón (no pulsado)
int lastButtonState = HIGH; // Estado anterior del botón
bool isOn = false;          // Estado de encendido/apagado

//********** Definicion X e Y para el soltado ***********

float Soltado_X = 18;  // Posicion en la que soltara el objeto en X
float Soltado_Y = 3;    // Posicion en la que soltara el objeto en Y

volatile bool detener = false;  // Bandera para detener el motor
volatile int Pasos = 12182;     // Variable para almacenar el número de pasos

int estado = 0;         // Inicial: 0; Recogiendo = 1; Soltando = 2

//*********** Variables para el conteo del motor CNC ********

volatile int Pulsos = 0;  // Variable para almacenar el número de pulsos
int currentPosition = 0;  // Variable para almacenar la posición actual del motor
int targetPosition = 0;   // Variable para almacenar la posición objetivo
const float screwDistancePerRev = 15.0; // Distancia en cm que recorre el husillo
bool movRobot= false;     // Estado start/stop del robot


void setup() {
  Serial.begin(9600);

  // Declaración del boton Start/Stop como boton de interrupcion
  pinMode(buttonPin, INPUT_PULLUP); // Configura el pin del botón como entrada con resistencia pull-up interna
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonInterrupt, CHANGE); // Asigna la función de interrupción al botón
  
  // Declaración de los finales de carrera como boton de interrupcion
  pinMode(PIN_2, INPUT_PULLUP); // Final de carrera 1 pin 2 como entrada con pull-up
  pinMode(PIN_3, INPUT_PULLUP); // Final de carrera 2 pin 3 como entrada con pull-up
  //attachInterrupt(digitalPinToInterrupt(PIN_2), detenerConteo, RISING); // Interrupción para detener el conteo
  //attachInterrupt(digitalPinToInterrupt(PIN_3), cambiarSentido, RISING);// Interrupción para cambiar el sentido

  // Leemos el valor de los Pasos del motor CNC 
  Serial.println("Valor Pasos: "+String(Pasos));

  // Servos
  base.attach(pin_base); // Asociamos la variable base al pin base
  L1.attach(pin_L1);     // Asociamos la variable base al pin base
  L2.attach(pin_L2);     // Asociamos la variable base al pin base
  Pnz.attach(pin_Pnz);   // Asociamos la variable base al pin base

  // Variables motor CNC
  pinMode(STEP, OUTPUT);  // Paso del motor pin 4 como salida
  pinMode(DIR, OUTPUT);   // Direccion del motor pin 5 como salida

}

void loop() {
  if (movRobot==true){ //Miramos si se ha pulsado el boton Start
      if (isOn) {
        Serial.println("Start loop");
        /*int q1=45;
        int q4=45;
        int q7=45;
         move_axis( 1, q1);   // Movimiento de la base 
         move_axis( 2, q4);// Hombro
         move_axis( 3, q7);// Codo
        delay(5000);*/  
        estado_rbt();     
      }
      else {
        Serial.println("Stop loop");
      }
  }
}

//------------------------------------------------------------------------------------//
//-------------------------  Funcion estados del robot -------------------------------//
//------------------------------------------------------------------------------------//

void estado_rbt() 
{
  switch (estado) {
    //---------------- Iniciando el movimiento ---------------
    case 0: // Caso 0 poner el robot en posicion Home
      if (movRobot==true){
        Serial.println("------------------------------------------");
        axis_home();  // Ponemos el robot en posicion de HOME
        delay(500);
        estado = 1;
        break; 
      } 
    case 1: // Caso 1 recoger pieza
      if (movRobot==true){
      // Posicion de recoleccion alto
        Serial.println("Aproximacion recoleccion");
        inversa(27.57 , 0, 20.84); // inversa (x,y,z) Los valores se dan en cm y como maximo 2 decimales
        delay(1000); // Espera 1 segundo antes de detener el motor
        }
        if (movRobot==true){
      // Posicion de recoleccion
        Serial.println("Posicion recoleccion");
        inversa(18, 15, 5); // Los valores se dan en cm y como maximo 2 decimales
        delay(1000);

      // Cerrar pinza
        Pnz.write(90); // Indicamos el angulo de giro para cerrar la pinza

      // Posicion de recoleccion alto
        inversa(18, 15, 10); // Los valores se dan en cm y como maximo 2 decimales
        delay(1000);

        estado = 2;
        break;
      }  
    case 2: // Caso 2 soltando el objeto
      if (movRobot==true){
      // Posicion de soltado alta
        Serial.println("Aproximacion soltado");
        inversa(Soltado_X, Soltado_Y, 10);// Los valores se dan en cm y como maximo 2 decimales
        delay(1000);
        
      // Posicion de soltado baja
        Serial.println("Posicion soltado");
        inversa(Soltado_X, Soltado_Y, 5);// Los valores se dan en cm y como maximo 2 decimales
        delay(500);
        
      // Abriendo pinza
        Pnz.write(90); // Indicamos el angulo de giro para abrir la pinza
      
      // Posicion de soltado alta
        inversa(Soltado_X, Soltado_Y, 10);// Los valores se dan en cm y como maximo 2 decimales
        delay(1000);

      // Volver a posicion de Home y reinicio del loop 
        Serial.println("Posicion inicial");
        axis_home();
        delay(1000);

        estado = 0;
        break;
      }
  }
}

//------------------------------------------------------------------------------------//
//-------------------------  Posicion inicial ----------------------------------------//
//------------------------------------------------------------------------------------//

void axis_home()
{
  // Primero, mover el motor en un sentido hasta que se active el pin 3
    digitalWrite(DIR, HIGH);    // giro en un sentido
    while (digitalRead(PIN_3) == HIGH && movRobot) { // Si se pulsa el boton de paro o se ha llegado al final de carrera, el motor no se mueve
        digitalWrite(STEP, HIGH);
        delay(1);
        digitalWrite(STEP, LOW);
        delay(1);
    }
    currentPosition = 0; // Posicion actual sera 0
  // Posicion Home 
    inversa(18.0 , 0, 24.8); // (x,y,z) en cm
  // Abriendo pinza
    Pnz.write(90); // Indicamos el angulo de giro para abrir la pinza
    delay(100);
}

//------------------------------------------------------------------------------------//
//-------------------------  Control velocidad Servos --------------------------------//
//------------------------------------------------------------------------------------//

void moveServo(Servo *servo, int endAngle, int time = 100) //Funcion para mover los servos con una cierta velocidad determinada
{
  unsigned long moveStartTime = millis(); 
  unsigned long servoRead = servo->read(); // Almacenamos la posicion en la que esta el servo
  
  // Anulem autoadaptatiu i fem graus fixes
  time = (abs(endAngle - servoRead), 0, 180, 0, time); // Hacemos el escalado de tiempo donde si el angulo de 0 a 180 tarda 0,5s pues si fuese 90 tendria que tardar 0.25s
  while ((millis() - moveStartTime) < time && movRobot) { 
    unsigned long progress = millis() - moveStartTime;
    long angle = map(progress, 0, time, servoRead, endAngle);
    servo->write(angle); // Escribimos el angulo del servo
  }
}

//------------------------------------------------------------------------------------//
//-------------------------  Control Servos ------------------------------------------//
//------------------------------------------------------------------------------------//

void move_axis(int servoId, float position)
{
  if (servoId == 1) // Servo Base
  { 
    //base.write(position);
    //delay(100);
    moveServo(&base, position); //Definimos el servomotor a mover y el angulo 
  }
  //--------------------------------------------------------
  if (servoId == 2) // Servo Hombro
  { /*L1.write(position);
    delay(100);*/
    moveServo(&L1, position);//, 500);// Podemos modificar la velocidad a la que tenemos por defecto
  }
  //--------------------------------------------------------
  if (servoId == 3) // Servo Codo
  { /*L2.write(position);
    delay(100);*/
    moveServo(&L2, position);
  }
}

//------------------------------------------------------------------------------------//
//-------------------------  Control Motor CNC ---------------------------------------//
//------------------------------------------------------------------------------------//

void moveMotor(float distanceCM) {  // moveMotor(5); // Ejemplo: Mover 5 cm
  targetPosition = (distanceCM*10) / screwDistancePerRev * Pasos; // Transformacion de cm a pasos
  // Mostramos la distancia propuesta en pulsos
  Serial.println("Distancia propuesta: "+String(targetPosition)+" cm");
  // Mostramos la distancia actual en pulsos
  Serial.println("Distancia Actual: "+String(currentPosition)+" pulsos");
  Pulsos = targetPosition-currentPosition; // Calculamos cuantos pulsos hemos de avanzar o retroceder
  Serial.println("Pulsos a realizar: "+String(Pulsos));

  if (Pulsos > 0) {
    digitalWrite(DIR, LOW); // Avanza
    Serial.println("Distancia Actual: "+String(currentPosition));
    for (int i = 0; i < Pulsos; i++) {
      if (digitalRead(PIN_2) == HIGH && movRobot){ // Si se pulsa el boton de paro o se ha llegado al final de carrera, el motor no se mueve
        digitalWrite(STEP, HIGH);
        delayMicroseconds(700); // Ajusta el tiempo de acuerdo a tu motor
        digitalWrite(STEP, LOW);
        delayMicroseconds(700); // Ajusta el tiempo de acuerdo a tu motor
        currentPosition=currentPosition+1; // Actualizamos la posicion 
      }
      else{// En caso que se pulse el boton de paro salimos
        return;
      }
    }
  } else if(Pulsos < 0){
    digitalWrite(DIR, HIGH); // Retrocede
    Pulsos = -Pulsos; // Convierte pulsos a positivos
    Serial.println("Distancia Actual: "+String(currentPosition)+" cm");
    for (int i = 0; i < Pulsos; i++) {
      if (digitalRead(PIN_3) == HIGH && movRobot){  // Si se pulsa el boton de paro o se ha llegado al final de carrera, el motor no se mueve
        digitalWrite(STEP, HIGH);
        delayMicroseconds(700); // Ajusta el tiempo de acuerdo a tu motor
        digitalWrite(STEP, LOW);
        delayMicroseconds(700); // Ajusta el tiempo de acuerdo a tu motor
        currentPosition=currentPosition-1;// Actualizamos la posicion 
      }
      else{ // En caso que se pulse el boton de paro salimos
        return;
      }
    }
  }
  else {
      // Si estamos en la posición objetivo
      Serial.println("Ya estás en la posición requerida");
      return;
  }
}

//------------------------------------------------------------------------------------//
//-------------------------  Calculo cinematica inversa ------------------------------//
//------------------------------------------------------------------------------------//

void inversa(float x1, float y1, float z1)
{
  // Definición de constantes del robot
  const float d1 = 0.0345;
  const float d2 = 0.0525;
  const float d3 = 0.06045;
  const float d4 = 0.135;
  const float d5 = 0.147;
  const float d6 = 0.03331;

  // Definicion de variables
  float L1;
  float theta1;
  float theta2;
  float theta3;

  float x = x1/100;  
  float y = y1/100;  
  float z = z1/100;  
  L1=y;

  float x_4, y_4, z_4, h, q1, q2, q3, q4, q6, q7, axis_escale;

  //Si el valor de L1 es menor a 0 muestra un error y sale de la funcion
  if (L1 < 0) { 
    Serial.println("fuera del limite");
    return;
  } else if (L1 <= 0.150 || L1 == 0) {
    L1 = y;
    Serial.println("dentro del limite 1");
    float ny_EE = 0;
    q1 = atan2(ny_EE, x) * 180.0 / PI; //Convertimos de radianes a grados

    x_4 = x - d6;
    y_4 = ny_EE;
    z_4 = z - (d2 + d3);
    h = sqrt(x_4 * x_4 + z_4 * z_4);
    q2 = atan(z_4 / x_4) * 180.0 / PI;
    q3 = acos((h * h + d4 * d4 - d5 * d5) / (2 * h * d4)) * 180.0 / PI;
    q4 = q2 + q3;
    q6 = acos((-h * h + d4 * d4 + d5 * d5) / (2 * d5 * d4)) * 180.0 / PI;
    q7 = -(180.0 - (q6 + q2 + q3)) - q4;

  } else if (L1 > 0.150) {
    Serial.println("Dentro del límite 2");
    L1 = 0.150 - d1;
    float ny_EE = y - 0.150;
    float c = sqrt(x * x + ny_EE * ny_EE);
    q1 = atan2(ny_EE, x) * 180.0 / PI;

//Encuentra los valores para la posición del elemento final x, y, z
    x_4 = x - (d6 * cos(q1));
    y_4 = ny_EE - (d6 * sin(q1));
    z_4 = z - (d2 + d3);
    
//Calculamos la diagonal del centro al punto
    h = sqrt(x_4 * x_4 + z_4 * z_4);
    q2 = atan(z_4 / c) * 180.0 / PI;
    q3 = acos((h * h + d4 * d4 - d5 * d5) / (2 * h * d4)) * 180.0 / PI;
    q4 = q2 + q3;
    q6 = acos((-h * h + d4 * d4 + d5 * d5) / (2 * d5 * d4)) * 180.0 / PI;
    q7 = -(180.0 - (q6 + q2 + q3)) - q4;
  } 

  Serial.println("Valor q1: "+String(q1));
  Serial.println("Valor q2: "+String(-q4+90));
  Serial.println("Valor q3: "+String(-q7-90));
  Serial.println("Valor q7: "+String(180-q7));
  //Movemos los ejes
  moveMotor(L1);        // Patin
  move_axis( 1, q1+90);   // Movimiento de la base 
  move_axis( 2, 180-q4);// Hombro
  axis_escale= 90+(q6-q4);
  Serial.println("Escalado: "+String(axis_escale));
  Serial.println("Valor q7: "+String(180+q7));
  move_axis( 3, axis_escale);//180+q7);//axis_escale);// Codo
}

//------------------------------------------------------------------------------------//
//-------------------------  Final carrera detener conteo ----------------------------//
//------------------------------------------------------------------------------------//
// Modificar esto para que lo que haga sea parar el motor y q solo deje funcionar para el otro sentido
/*void detenerConteo() {
  //detener = true; // Establecer la bandera de detener el conteo de pulsos+
  movRobot = false;
  isOn = false;
}

void cambiarSentido() {
  //detachInterrupt(digitalPinToInterrupt(PIN_3)); // Deshabilitar la interrupción para cambiar el sentido
  
}*/

//------------------------------------------------------------------------------------//
//-------------------------  Boton Star/Stop -----------------------------------------//
//------------------------------------------------------------------------------------//

void buttonInterrupt() {
  // Esta función se ejecutará cuando se pulse el botón
  int buttonState = digitalRead(buttonPin);
  
  // Comprueba si el botón ha cambiado de estado (se ha pulsado o liberado)
  if (buttonState != lastButtonState) {
    if (buttonState == LOW) {
      isOn = !isOn; // Invierte el estado de encendido/apagado al pulsar el botón
      if (isOn) {
        Serial.println("Start");
        movRobot=true;
      } else {
        Serial.println("Stop");
        movRobot=false;
      }
    }
      delay(50); // Pequeño retardo para evitar rebotes
  }  
  // Actualiza el estado anterior del botón
  lastButtonState = buttonState;
}