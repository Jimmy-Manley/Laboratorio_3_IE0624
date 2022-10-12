#include <SPI.h>
#include <Adafruit_PCD8544.h>
//sensor temperatura y humedad
#include "DHT.h" //It is necessary for us to include this library when we use DHT11/DHT22 sensor
#define DHTPIN 11 // Defining the data output pin to Arduino
#define DHTTYPE DHT11 // Specify the sensor type(DHT11 or DHT22)
#include <math.h>
// Software SPI (slower updates, more flexible pin options):
// pin 7 - Serial clock out (SCLK)
// pin 6 - Serial data out (DIN)
// pin 5 - Data/Command select (D/C)
// pin 4 - LCD chip select (CS)
// pin 3 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(7, 6, 5, 4, 3);

//sensor pin
int relayPin = A5; // señal del sensor
int pinControlador = 10;      // la señal de control u se envia por el pin 3 que es PWM


//inicialización pto de operación
float referenciaPin = A2;
float value;      //variable que almacena la lectura analógica raw
//int position;   //posicion del potenciometro en tanto por ciento
int offset = 30;

float h;
float r; //valor deseado o set point o referencia
float y; //señal realimentada
int yi;
float yk_1;//valor pasado de la señal de realimentada
float uk; //valor presente de la señal de control
float uk_1=0.0;//valor pasado u(k-1) de la señal de control
float uk_lim;// valor señal de control limitada
float ek;//valor presente de la señal de error
float ek_1=0.0;//valor pasado de la señal de error
float ek_2;//valor pasado e(k-2)de la señal de control

const int LEDL=13;
const int LEDH=12;
float deltat = 0.1; // 10 ms, deltat se define en segundos.
const double Kp = 2;
const double Ti = 0.1;
const double Td = 0.5;
const double Ts = 100; // tiempo de muestreo de 10 ms

//thermistor
const int Rc = 10000; //valor de la resistencia
const int Vcc = 2.5;
const float sensorTPin = A0;

float A = 1.11492089e-3;
float B = 2.372075385e-4;
float C = 6.954079529e-8;

float K = 2.5; //factor de disipacion en mW/C

//init sensor
DHT dht(DHTPIN, DHTTYPE); //Declaration for varibles below...


void setup()   {
  Serial.begin(9600);
  Serial.println("PCD");
  dht.begin(); //To initialise DHT sensor
  
  display.begin();
  pinMode(LEDL,OUTPUT);
  pinMode(LEDH,OUTPUT);
  pinMode(sensorTPin, INPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(referenciaPin, INPUT);
  pinMode(pinControlador, OUTPUT);
  //digitalWrite(pinControlador,100);
  analogWrite(pinControlador,100);
}

void loop() {
  
  if(ek<=0){

    uk_1 = 0.0;
  }
  //sensado de la temperatura a partir del termistor
  float raw = analogRead(sensorTPin);
  float V =  raw / 1024 * Vcc;

  float R = (Rc * V ) / (Vcc - V);
  
  float logR  = log(R);
  float R_th = 1.0 / (A + B * logR + C * logR * logR * logR );

  float kelvin = R_th - V*V/(K * R)*1000;
  y = kelvin - 273.15;
  yi = int(y);

  if(yi<30){
    digitalWrite(LEDL,HIGH);
  }

    if(yi>30){
    digitalWrite(LEDL,LOW);
  }

    if(yi>42){
    digitalWrite(LEDH,HIGH);
  }
   if(yi<42){
    digitalWrite(LEDH,LOW);
  }
  
  
  // text display tests
  display.clearDisplay();
  //display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0,0);

  value = analogRead(referenciaPin);          // realizar la lectura analógica raw
  r = map(value, 0, 1023, 0, 13);  // convertir a porcentaje
  r += offset;
  r = int(r);
  ek =  r - yi; //se calcula el error
 Serial.print("ek: "); 
  Serial.println(ek);   
  h = dht.readHumidity();

   //y = dht.readTemperature(); //analogRead(sensorPin); //lee la salida del sistema
   //y = map(y, 0, 1023, 0, 100); //se escala la señal, 0 es a 0 como 1023 es a 100, arduino tiene convertidor A/D de 10 bits (pasa 0 a 5V a 0 a 1023)
    
  //Ecuación discreta del controlador
  uk = uk_1 + Kp*((1+deltat/Ti+Td/deltat)*ek-(1+2*Td/deltat)*ek_1+(Td/deltat)*ek_2);

  if (uk > 100) { //Bloque de saturacion de la salida del controlador
  uk_lim= 100;
  }
  else if (uk < 0) {
  uk_lim = 0;
  }
  else{
     uk_lim = uk;
  } 

  uk_lim = uk_lim*255/100; // Se escala de 0 a 255 por ser PWM con 8 bits

  
  
  uk_1 = uk;//declaracion del estado anterior como el estado presente para el siguiente iteración

    if(ek<=0){

    uk_1 = 0.0;
  }
  
  ek_1 = ek;
  ek_2 = ek_1;
  Serial.print("uk1: "); 
  Serial.println(uk_1);
  Serial.print("uk: "); 
  Serial.println(uk); 
  Serial.print("uk_lim: "); 
  Serial.println(uk_lim); 

  if(ek>0){
  analogWrite(pinControlador,uk_lim);  // salida PWM de la señal de control

  analogWrite(relayPin,uk_lim); //vamo a ver
  }
  else{
  analogWrite(pinControlador,0);  // salida PWM de la señal de control

  analogWrite(relayPin,0); //vamo a ver 
  }
  delay(50);
  delay(Ts);
    
  display.print("T Op.: ");
  display.print(r);
  display.println(" C");

  display.print("SIG. crtl: ");
  display.println(uk_lim);
  
  display.print("T Sens: ");
  display.println(yi);

  display.print("H Sens: ");
  display.println(h);
  
  display.display();
  //delay(500);
}
