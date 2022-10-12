#include "DHT.h" //It is necessary for us to include this library when we use DHT11/DHT22 sensor
#define DHTPIN 11 // Defining the data output pin to Arduino
#define DHTTYPE DHT11 // Specify the sensor type(DHT11 or DHT22)

// Controlador P, PI y PID, Sistemas de Control, EIE, UCR

//Declaracion de variables y pines
int sensorPin = A5; // señal del sensor, variable realimentada se lee en el pin análogico 5 del arduino
int pinControlador = 10;      // la señal de control u se envia por el pin 3 que es PWM
const int referenciaPin=A2;  // variable para almacenar el valor de la referencia del sistema
int value;      //variable que almacena la lectura analógica raw
int position;   //posicion del potenciometro en tanto por ciento
int offset = 30;//offset para settear la referencia

float r; //valor deseado o set point o referencia
float y; //señal realimentada
float yk_1;//valor pasado de la señal de realimentada
float uk; //valor presente de la señal de control
float uk_1=0.0;//valor pasado u(k-1) de la señal de control
float uk_lim;// valor señal de control limitada
float ek;//valor presente de la señal de error
float ek_1=0.0;//valor pasado de la señal de error
float ek_2;//valor pasado e(k-2)de la señal de control

float deltat = 10; // 10 s, deltat se define en segundos.
const double Kp = 2;
const double Ti = 0.1;
const double Td = 0.5;
const double Ts = 10000; // tiempo de muestreo de 10 s

// LCD display

#include <LiquidCrystal.h>
LiquidCrystal lcd(7,6, 5, 4, 3, 2);

DHT dht(DHTPIN, DHTTYPE); //Declaration for varibles below...

void setup() {
  Serial.begin(9600); //To engage serial monitor
  dht.begin(); //To initialise DHT sensor

  lcd.begin(16,2);

  pinMode(sensorPin, INPUT);
  pinMode(referenciaPin, INPUT);
  pinMode(pinControlador, OUTPUT);
}

void loop() {
   //settea referencia mediante potenciómetro
   value = analogRead(referenciaPin);          // realizar la lectura analógica raw
   position = map(value, 0, 1023, 0, 12);  // convertir a porcentaje
   position += offset;
   //...hacer lo que se quiera, con el valor de posición medido
   Serial.print("potenciometro: ");
   Serial.print(position);
   Serial.print("\n");
   delay(500);

    //lee temperatura actual desde el sensor y calcula el error
    y = dht.readTemperature(); //analogRead(sensorPin); //lee la salida del sistema
    y = map(y, 0, 1023, 0, 100); //se escala la señal, 0 es a 0 como 1023 es a 100, arduino tiene convertidor A/D de 10 bits (pasa 0 a 5V a 0 a 1023)
    r = analogRead(referenciaPin); //lee la salida del sistema
    r = map(r, 0, 1023, 0, 100); //se escala la señal, 0 es a 0 como 1023 es a 100, arduino tiene convertidor A/D de 10 bits (pasa 0 a 5V a 0 a 1023)
    ek =  r - y; //se calcula el error

    //se actualiza la señal de control con base en el error
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
    ek_1 = ek;
    ek_2 = ek_1;
    
    Serial.println("señal de control: ");
    Serial.println(uk_lim); 
    //analogWrite(pinControlador,uk_lim);  // salida PWM de la señal de control
    delay(Ts);

    //lcd.setCursor(0,0);
    //lcd.setCursor(0,1);

///
  
//  float tC= dht.readTemperature();
//  lcd.setCursor(0,0);
//  lcd.print("Temp: ");
//  lcd.print(dht.readTemperature());
//  lcd.print(" C ");
//  lcd.setCursor(0,0);
//  lcd.setCursor(0,1);
//  lcd.print(millis()/1000);
//  lcd.print(" seg.       ");
//  delay(1000);
//  lcd.print("Jimmy Manley P. ");
//  lcd.setCursor(0,1);
//  lcd.print("A73725 UCR Intel");
//  delay(2000);
  //Here h= Humidity; tC= Temperature in degree C; tF= Temperature in degree F
  //float h= dht.readHumidity();
  //float tC= dht.readTemperature();
  //float tF= dht.readTemperature(true);

//  if (isnan(h) || isnan(tC) ){
//    Serial.println("Failed to read the DHT sensor. Check connections");
//    }
//  else{
//    Serial.print("Humidity: ");
//    Serial.print(h);
//    Serial.print("%");
//    Serial.print("  ||  ");
//    Serial.print("Temperature: ");
//    Serial.print(tC);
//    Serial.print("C");
//    Serial.print("\n");
//    }
}    
