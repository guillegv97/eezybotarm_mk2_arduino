#include <Servo.h> // Incluimos la librería para los servomotores
#include<math.h> // Incluimos la librería para los calculos matematicos
#define REPEAT_CAL true //Establecer REPEAT_CAL en true en lugar de false para ejecutar la calibración nuevamente.

//************** Definicion motor CNC ********************

#define STEP 4      // pin STEP de A4988 a pin 4
#define DIR 5       // pin DIR de A4988 a pin 5
#define PIN_2 2     // Final de carrera 1
#define PIN_3 3     // Final de carrera 2

//********* Definicion de los servos **********************

#define pin_base 8   // Servo Base
#define pin_L1 9     // Servo Hombro
#define pin_L2 10     // Servo Codo
#define pin_Pnz 11   // Servo Pinza

//********** Definicion variables de los servos ***********

Servo base; // Crea objeto para la base
Servo L1;   // Crea objeto para el Hombro
Servo L2;   // Crea objeto para el Codo
Servo Pnz;  // Crea objeto para la pinza
int ang;

//********** Definicion variables de los servos ***********

const int buttonPin = 18;   // Pin del botón Start/Stop
int buttonState = HIGH;     // Estado inicial del botón (no pulsado)
int lastButtonState = HIGH; // Estado anterior del botón
bool isOn = false;          // Estado de encendido/apagado


float L = 1.5;          // Largo del objeto, Valor en centimetros 
float a = 30;           // Ancho del objeto; Valor en centimetros
float Soltado_X = 280;  // Posicion en la que soltara el objeto en X
float Soltado_Y = 0;    // Posicion en la que soltara el objeto en Y

volatile int Pasos = 0;         // Variable para almacenar el número de pasos
volatile bool detener = false;  // Bandera para detener el motor
bool calibrateDevice = true;	// Se ha de calibrar el dispositivo

int estado = 0;         // Reposo: 0; Recogiendo = 1; Soltando = 2

//*********** Variables para el conteo del motor CNC ********
volatile int Pulsos = 0;  // Variable para almacenar el número de pulsos
int currentPosition = 0;  // Variable para almacenar la posición actual del motor
int targetPosition = 0;   // Variable para almacenar la posición objetivo
const float screwDistancePerRev = 15.0; // Distancia en cm que recorre el husillo
bool movRobot= false;     // Estado start/stop del robot

void setup() {
  Serial.begin(9600);
  // Declaración del boton como boton de interrupcion
  pinMode(buttonPin, INPUT_PULLUP); // Configura el pin del botón como entrada con resistencia pull-up interna
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonInterrupt, CHANGE); // Asigna la función de interrupción al botón

  // Leemos el valor de los Pasos del motor CNC 
  Serial.println("Valor Pasos: "+String(Pasos));

  // Servos
  base.attach(pin_base);
  L1.attach(pin_L1);
  L2.attach(pin_L2);
  Pnz.attach(pin_Pnz);

  // Variables motor CNC
  pinMode(STEP, OUTPUT);  // Paso del motor pin 4 como salida
  pinMode(DIR, OUTPUT);   // Direccion del motor pin 5 como salida
  pinMode(PIN_2, INPUT_PULLUP); // Final de carrera 1 pin 2 como entrada con pull-up
  pinMode(PIN_3, INPUT_PULLUP); // Final de carrera 2 pin 3 como entrada con pull-up
  attachInterrupt(digitalPinToInterrupt(PIN_2), detenerConteo, FALLING); // Interrupción para detener el conteo
  attachInterrupt(digitalPinToInterrupt(PIN_3), cambiarSentido, FALLING);// Interrupción para cambiar el sentido
}

void loop() {
  while(!movRobot) {
  	delay(50);
  }
  if (calibrateDevice) {
	calibrate();  // Miramos si hace falta calibrar
	currentPosition=Pasos;
	calibrateDevice=false;
  }
  if (movRobot==true){
      if (isOn) {
        Serial.println("Start_ini");
        estado_rbt();       
      }
      else {
        Serial.println("Stop");
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
      axis_home();  // Ponemos el robot en posicion de HOME
      delay(500);
      estado = 1;
      break; 
    case 1: // Caso 1 recoger pieza
    // Posicion de recoleccion alto
      Serial.println("Aproximacion recoleccion");
      inversa(45, 140, 95); // inversa (x,y,z) Los valores se dan en cm y como maximo 2 decimales
      if (movRobot) {
      	delay(1000); // Espera 1 segundo antes de detener el motor
      }

    // Posicion de recoleccion
      Serial.println("Posicion recoleccion");
      inversa(45, 140, 95); // Los valores se dan en cm y como maximo 2 decimales
      if (movRobot) {
      	delay(1000);
      }

    // Cerrar pinza
      Pnz.write(90); // Indicamos el angulo de giro para cerrar la pinza

    // Posicion de recoleccion alto
      inversa(45, 140, 95); // Los valores se dan en cm y como maximo 2 decimales
      delay(1000);

      estado = 2;
      break;
    case 2: // Caso 2 soltando el objeto
    // Posicion de soltado alta
      Serial.println("Aproximacion soltado");
      inversa(Soltado_X, Soltado_Y, 130);// Los valores se dan en cm y como maximo 2 decimales
      delay(1000);
      
    // Posicion de soltado baja
      Serial.println("Posicion soltado");
      inversa(Soltado_X, Soltado_Y, 100);// Los valores se dan en cm y como maximo 2 decimales
      delay(500);
      
    // Abriendo pinza
      Pnz.write(90); // Indicamos el angulo de giro para abrir la pinza
    
    // Posicion de soltado alta
      inversa(Soltado_X, Soltado_Y, 130);// Los valores se dan en cm y como maximo 2 decimales
      delay(1000);

    // Volver a posicion de Home y reinicio del loop 
      Serial.println("Posicion inicial");
      axis_home();
      delay(1000);

      estado = 0;
      break;
  }
}

//------------------------------------------------------------------------------------//
//-------------------------  Posicion inicial ----------------------------------------//
//------------------------------------------------------------------------------------//

void axis_home()
{
  // Posicion Home 
    inversa(18.0 , 0, 24.8); // (x,y,z) en cm
  // Abriendo pinza
    Pnz.write(90); // Indicamos el angulo de giro para abrir la pinza
    delay(100);
}

//------------------------------------------------------------------------------------//
//-------------------------  Control Servos ------------------------------------------//
//------------------------------------------------------------------------------------//

void move_axis(int servoId, float position)
{
  if (servoId == 1) // Servo Base
  { base.write(position);
    delay(100);
  }
  //--------------------------------------------------------
  if (servoId == 2) // Servo Hombro
  { L1.write(position);
    delay(100);
  }
  //--------------------------------------------------------
  if (servoId == 3) // Servo Codo
  { L2.write(position);
    delay(100);
  }
}

//------------------------------------------------------------------------------------//
//-------------------------  Control Motor CNC ---------------------------------------//
//------------------------------------------------------------------------------------//

void moveMotor(float distanceCM) {  // moveMotor(5); // Ejemplo: Mover 5 cm
  targetPosition = distanceCM / screwDistancePerRev * Pasos; // Transformacion de cm a pasos
  // Mostramos la distancia propuesta en pulsos
  Serial.println("Distancia propuesta: "+String(targetPosition));
  // Mostramos la distancia actual en pulsos
  Serial.println("Distancia Actual: "+String(currentPosition));
  Pulsos = targetPosition-currentPosition; // Calculamos cuantos pulsos hemos de avanzar o retroceder
  
  if (Pulsos > 0) {
    digitalWrite(DIR, LOW); // Avanza
    Serial.println("Distancia Actual: "+String(currentPosition));
    for (int i = 0; i < Pulsos; i++) {
      if (movRobot==true){ // Si se pulsa el boton de paro, el motor no se mueve
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
    Serial.println("Distancia Actual: "+String(currentPosition));
    for (int i = 0; i < Pulsos; i++) {
      if (movRobot==true){// Si se pulsa el boton de paro, el motor no se mueve
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

float x_4, y_4, z_4, h, q1, q2, q3, q4, q6, q7;

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

  //Movemos los ejes
  moveMotor(L1);        // Patin
  move_axis( 1, q1 );   // Movimiento de la base 
  move_axis( 2, -q4+90);// Hombro
  move_axis( 3, -q7-90);// Codo
}

//------------------------------------------------------------------------------------//
//-----------------------  Funcion calibracion motor CNC -----------------------------//
//------------------------------------------------------------------------------------//

void calibrate()
{
  if(Pasos == 0||REPEAT_CAL==true){ //Si no tenemos valor de pasos o tenemos el repetir calculo entonces ->
    // Primero, mover el motor en un sentido hasta que se active el pin 3
    digitalWrite(DIR, HIGH);    // giro en un sentido
    while (digitalRead(PIN_3) == LOW) {
      digitalWrite(STEP, HIGH);
      delay(1);
      digitalWrite(STEP, LOW);
      delay(1);
    }
    // Luego, invertir el sentido y contar los pasos hasta que se active el final de carrera 2
    Pasos = 0; // Reiniciar el contador de pasos
    detener = false; // Reiniciar la bandera de detener
    digitalWrite(DIR, LOW);  // giro en sentido opuesto
    while (digitalRead(PIN_2) == LOW && !detener) {
      digitalWrite(STEP, HIGH);
      delay(1);
      digitalWrite(STEP, LOW);
      delay(1);
      Pasos++; // Incrementar el contador de pasos
    }
    // Mostrar el número de pasos por pantalla
    Serial.println("Pasos: "+String(Pasos));
    estado = 0;
    return;
  }
  else {
    Serial.println("Pasos: "+String(Pasos));
    estado = 0;
    return;
  }
}

//------------------------------------------------------------------------------------//
//-------------------------  Final carrera detener conteo ----------------------------//
//------------------------------------------------------------------------------------//

void detenerConteo() {
  detener = true; // Establecer la bandera de detener el conteo de pulsos
}

void cambiarSentido() {
  detachInterrupt(digitalPinToInterrupt(PIN_3)); // Deshabilitar la interrupción para cambiar el sentido
}

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
	calibrateDevice=true;
      }
    }
      delay(50); // Pequeño retardo para evitar rebotes
  }  
  // Actualiza el estado anterior del botón
  lastButtonState = buttonState;
}