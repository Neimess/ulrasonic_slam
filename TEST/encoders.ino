 /// Variables //////////////////////////////////////////////////////////////////////////////////////////////////////////////
 // Motor A connections
// Motor A connections
int enA = 6;
int in1 = 7;
int in2 = 12;
// Motor B connections
int enB = 5;
int in3 = 4;
int in4 = 8;
 /// Variables //////////////////////////////////////////////////////////////////////////////////////////////////////////////
int encoder_pin_1 = 2;             //Pin 2, donde se conecta el encoder
int encoder_pin_2 = 3;      
unsigned int rpm_1 = 0;           // Revoluciones por minuto calculadas.
float velocity_1 = 0;                  //Velocidad en [Km/h]
volatile byte pulses_1 = 0;       // Número de pulsos leidos por el Arduino en un segundo
unsigned long timeold = 0;  // Tiempo 
unsigned int rpm_2 = 0;           // Revoluciones por minuto calculadas.
float velocity_2 = 0;                  //Velocidad en [Km/h]
volatile byte pulses_2 = 0;       // Número de pulsos leidos por el Arduino en un segundo
static volatile unsigned long debounce_1 = 0; // Tiempo del rebote.
unsigned int pulsesperturn = 30; // Número de muescas que tiene el disco del encoder.
const int wheel_diameter = 64;   // Diámetro de la rueda pequeña[mm]
static volatile unsigned long debounce_2 = 0; // Tiempo del rebote.

void counter_1();
void counter_2();
////  Configuración del Arduino /////////////////////////////////////////////////////////
void setup(){
   Serial.begin(9600); // Configuración del puerto serie  
   pinMode(encoder_pin_1, INPUT);
   pinMode(encoder_pin_2, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoder_pin_1), counter_1, CHANGE); // Привязываем функцию обработки прерывания к пину для первого энкодера
    attachInterrupt(digitalPinToInterrupt(encoder_pin_2), counter_2, CHANGE);
   pulses_1 = 0;
   rpm_1 = 0;
   pulses_2 = 0;
   rpm_2 =0;
  Serial.print("Seconds ");
  Serial.print("RPM ");
  Serial.print("Pulses ");
  Serial.println("Velocity[Km/h]");
  // Set all the motor control pins to outputs
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    
    // Turn off motors - Initial state
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

void move(int speed, float time, char* direction){
  time = time * 1000;
  int speed_left = speed;
  int speed_right = speed;
  analogWrite(enA, speed_right);
  analogWrite(enB, speed_left-10);
    switch (*direction) {
        case 'F':
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          break;
        case 'B':
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          break;
    } 
  delay(time);
}


////  Configuración del Arduino /////////////////////////////////////////////////////////

////  Programa principal ///////////////////////////////////////////////////////////////////////
 void loop(){
   if (millis() - timeold >= 1000){  // Se actualiza cada segundo
      move(120, 1, "F");
      noInterrupts(); //Don't process interrupts during calculations // Desconectamos la interrupción para que no actué en esta parte del programa.
      rpm_1 = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses_1; // Calculamos las revoluciones por minuto
      rpm_2  = (60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses_2;
      velocity_1 = rpm_1 * 3.1416 * wheel_diameter / 1000 / 60; // Cálculo de la velocidad en [m/s] 
      velocity_2 = rpm_2 * 3.1416 * wheel_diameter / 1000 / 60;
      timeold = millis(); // Almacenamos el tiempo actual.
      Serial.print("Time: "); Serial.print(millis()/1000); Serial.print(" s\t");
    Serial.print("Encoder 1 - RPM: "); Serial.print(rpm_1, DEC); Serial.print("\tPulses: "); Serial.print(pulses_1, DEC); Serial.print("\tVelocity: "); Serial.print(velocity_1, 2); Serial.println(" m/s");
    Serial.print("Encoder 2 - RPM: "); Serial.print(rpm_2, DEC); Serial.print("\tPulses: "); Serial.print(pulses_2, DEC); Serial.print("\tVelocity: "); Serial.print(velocity_2, 2); Serial.println(" m/s");

      pulses_1 = 0;  // Inicializamos los pulsos.
      pulses_2 = 0;
      interrupts(); // Restart the interrupt processing // Reiniciamos la interrupción
   }
  }
////Fin de programa principal //////////////////////////////////////////////////////////////////////////////////
///////////////////////////Función que cuenta los pulsos buenos ///////////////////////////////////////////
 void counter_1(){
  if(  digitalRead (encoder_pin_1) && (micros()-debounce_1 > 500)) { 
// Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
        debounce_1 = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
        pulses_1++;}  // Suma el pulso bueno que entra.
        else ; } 




 void counter_2(){
  if(  digitalRead (encoder_pin_2) && (micros()-debounce_2 > 500)) { 
// Vuelve a comprobar que el encoder envia una señal buena y luego comprueba que el tiempo es superior a 1000 microsegundos y vuelve a comprobar que la señal es correcta.
        debounce_2 = micros(); // Almacena el tiempo para comprobar que no contamos el rebote que hay en la señal.
        pulses_2++;}  // Suma el pulso bueno que entra.
        else ; } 






// // encoder pin
// int encoderPin=3;
// // motor pins
// int ENA = 5;
// int IN1 = 4;
// int IN2 = 8;
 
// // we have 20 holes, and consequently, angle/pulse=360/20= 18 degrees per pulse
// volatile int anglePerPulse=15;
// // total pulses from the start of the Arduino program
// volatile unsigned long totalPulses = 0;  
// // total angle calculated from totalPulses
// volatile long totalAngle=0;
// // variables for measuring time
// volatile unsigned long lastTime=0;
// volatile unsigned long currentTime=0;
// volatile double deltaT=0; // deltaT=currentTime-lastTime
 
 
// // this is the average angular velocity that we compute on
// // the basis of the variables shown below
// volatile double angularVelocity=0;
// // total time until sumPulses reaches averageSample
// volatile unsigned long timeAverageAngularVelocity=0;
// // sumPulses goes from 0 to averageSample, and then it is set back to zero
// volatile int sumPulses=0;
// // how many pulses we will take to calculate the average angular velocity
// volatile int averageSample=50;
 
// void setup() {
//   // set the motor pins
//   pinMode(ENA, OUTPUT);
//   pinMode(IN1, OUTPUT);
//   pinMode(IN2, OUTPUT);
 
//   // set the encoder pin
//   pinMode(encoderPin, INPUT);
//   // attach the interrupt for tracking the pulses
//   // one the pulse is detected, interruptFunction() is called
//   attachInterrupt(digitalPinToInterrupt(encoderPin), interruptFunction, RISING);
 
//   Serial.begin(9600);
//   // obtain the current time
//   lastTime=millis();
// }
 
// void loop() {
//   // set the motor speed
//   analogWrite(ENA, 255);
//   // direction
//   digitalWrite(IN1,HIGH);
//   digitalWrite(IN2, LOW);
// }
 
// void interruptFunction(){
//         // get the current time
//         currentTime=millis();
//         // deltaT is approximately equal to the time between the pulses
//         // there is a delay...
//         deltaT=currentTime-lastTime;
//         lastTime=currentTime;
 
//         // increment the total number of pulses
//         totalPulses = totalPulses + 1;
//         // calculate the total angle
//         totalAngle= totalPulses*anglePerPulse;
 
//         // this is another sum of pulses that is used to calculate
//         // the average angular velocity
//         // this sum is set to zero after averageSample
//         sumPulses=sumPulses+1;
//         // track the total time between computing the averages
//         timeAverageAngularVelocity=timeAverageAngularVelocity+deltaT;
 
//        // here we calculate the average angular velocity
//        // after averageSample pulses
//        if (sumPulses>=averageSample)
//        {
//           angularVelocity=1000*((double)(sumPulses * anglePerPulse) )/((double) timeAverageAngularVelocity );
//           Serial.println((String)"Total angle:"+totalAngle+" Angular velocity:"+angularVelocity);
//           sumPulses=0;
//           timeAverageAngularVelocity=0;
//        }
      
        
// }