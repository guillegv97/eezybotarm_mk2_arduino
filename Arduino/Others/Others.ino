#include <Servo.h> // Incluimos la librería para los servomotores

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
int PINSERVO = 11;		// pin 2 conectado a señal del servo
int PULSOMIN = 450;		// pulso minimo en microsegundos
int PULSOMAX = 2550;		// pulso maximo en microsegundos


//********** Definicion variables de los servos ***********

const int buttonPin = 18;   // Pin del botón Start/Stop (Se utiliza el pin 18 pq el arduino mega tiene asignado ese boton como interrupcion)
int buttonState = HIGH;     // Estado inicial del botón (no pulsado)
int lastButtonState = HIGH; // Estado anterior del botón
bool isOn = false;          // Estado de encendido/apagado
volatile bool detener = false;  // Bandera para detener el motor
volatile int Pasos = 12182;     // Variable para almacenar el número de pasos

int estado = 0;         // Inicial: 0; Recogiendo = 1; Soltando = 2

//*********** Variables para el conteo del motor CNC ********

volatile int Pulsos = 0;  // Variable para almacenar el número de pulsos
int currentPosition = 0;  // Variable para almacenar la posición actual del motor
int targetPosition = 0;   // Variable para almacenar la posición objetivo
const float screwDistancePerRev = 15.0; // Distancia en cm que recorre el husillo
bool movRobot= false;     // Estado start/stop del robot

int pos = 0;


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
  base.attach(pin_base, PULSOMIN, PULSOMAX); // Asociamos la variable base al pin base
  L1.attach(pin_L1);     // Asociamos la variable base al pin base
  L2.attach(pin_L2);     // Asociamos la variable base al pin base
  Pnz.attach(pin_Pnz);   // Asociamos la variable base al pin base
  base.write(0);
  Pnz.write(0);
  // Variables motor CNC
  pinMode(STEP, OUTPUT);  // Paso del motor pin 4 como salida
  pinMode(DIR, OUTPUT);   // Direccion del motor pin 5 como salida
}

void loop() {
  move_axis(0,200,90,200,90,200);// base, brazo, antebrazo
  if (movRobot==true){ //Miramos si se ha pulsado el boton Start
      if (isOn) {
        Serial.println("Start loop");
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
      //Recoleccion
        move_axis(180,400,90,400,90,400);// base, brazo, antebrazo
        delay(500);
        move_axis(180,100,90,200,30,200);// base, brazo, antebrazo
        delay(500);
        move_axis(180,100,165,200,5,200);// base, brazo, antebrazo
        delay(500);
        move_axis(180,100,165,500,35,500);// base, brazo, antebrazo
        delay(500);
        Pnz.write(50);
        delay(1000);
        move_axis(180,100,110,500,60,500);// base, brazo, antebrazo
        delay(500);
        move_axis(0,200,90,200,90,200);// base, brazo, antebrazo
        delay(500);
      if (pos<=14){
        moveMotor(pos); 
        Serial.println(pos);
        pos=pos+7;
      }
      else{
        pos=0;
      }
      // Dejado
        move_axis(0,200,90,200,30,200);// base, brazo, antebrazo
        delay(500);
        move_axis(0,200,160,500,20,500);// base, brazo, antebrazo
        delay(500);
        Pnz.write(0);
        move_axis(0,200,160,500,5,500);// base, brazo, antebrazo
        delay(500);
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
        delayMicroseconds(700);
        digitalWrite(STEP, LOW);
        delayMicroseconds(700);
    }
    currentPosition = 0; // Posicion actual sera 0
    moveMotor(3);
  // Posicion Home 
    move_axis(0,200,90,200,90,200);// base, brazo, antebrazo
  // Abriendo pinza
    Pnz.write(0); // Indicamos el angulo de giro para abrir la pinza
    delay(1000);
}

//------------------------------------------------------------------------------------//
//-------------------------  Control velocidad Servos --------------------------------//
//------------------------------------------------------------------------------------//

void moveServo(Servo *servo, int endAngle, int time = 100){ //Funcion para mover los servos con una cierta velocidad determinada

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

void move_axis(int angle1, int velocidad1, int angle2, int velocidad2, int angle3, int velocidad3) {// base, brazo, antebrazo

  moveServo(&base, angle1, velocidad1); //Definimos el servomotor a mover y el angulo 
  moveServo(&L1, angle2, velocidad2); //Podemos modificar la velocidad a la que tenemos por defecto
  moveServo(&L2, angle3, velocidad3);
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
      }
    }
      delay(50); // Pequeño retardo para evitar rebotes
  }  
  // Actualiza el estado anterior del botón
  lastButtonState = buttonState;
}
//------------------------------------------------------------------------------------//
//-------------------------  Control Motor CNC ---------------------------------------//
//------------------------------------------------------------------------------------//

void moveMotor(float distanceCM) {  // moveMotor(5); // Ejemplo: Mover 5 cm
  targetPosition = (distanceCM) / screwDistancePerRev * Pasos; // Transformacion de cm a pasos
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
