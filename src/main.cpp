///////////////// Libs /////////////////

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h> //bt
#include <Ultrasonic.h>
#include <math.h>

///////////////// Define Pins /////////////////

// Config. MPU6050
//Endereco I2C do MPU6050
const int MPU = 0x68;
//variáveis para acelerometro e giroscópio.
int eixoX, eixoY, eixoZ, Tmp, GiX, GiY, GiZ;
float eixoXA, eixoYA, eixoZA;
float acX, acXA, acY, acYA, acZ, acZA;
float velX, velY, velZ, x, y, z;
int ctt;
unsigned long millis_I;
unsigned long millis_F;
unsigned long millis_R;


// Bluetooth
#define TX A0
#define RX A1

SoftwareSerial bluetooth(TX, RX); //TX, RX (Bluetooth)
char buf;
String command = "";
Ultrasonic HCSR04(10,11);
int Dist = 0;
///////////////// Define ajusts /////////////////

int speed = 180;  // Defines the base speed of the Rover
float P = 0.25;   // Proportion of rotation

////////////////  Functions Declare //////////////////
void Contagem_Desc();
void Solic_MPU6050();
void Seriall_PrintS();
////////////////  SETUP  //////////////////

void setup() {
  // bluetooth module
  Serial.begin(9600);   // Default speed in HC-06 modules
  // Serial.println(F("Type the AT commands:"));
  bluetooth.begin(9600);
  //configura MPU6050
  // configura a escala de variação do giroscópio em graus/segundos
  Wire.begin();
  //Inicializa o MPU-6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);
  Wire.endTransmission(true);
  delay(5); 
}


////////////////  LOOP  //////////////////
void loop() {
  Contagem_Desc();
  bluetooth.print(";");bluetooth.print(x);
  bluetooth.print(";");bluetooth.println(y);
  delay(10);
}

void Solic_MPU6050(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU,14,true);
  //Armazena o valor dos sensores nas variaveis correspondentes
  eixoX=Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  eixoY=Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  eixoZ=Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GiX=Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GiY=Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GiZ=Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  // Converte valores acelerometro
  if(eixoX < 0) eixoX = eixoX * -1;
  if(eixoY < 0) eixoY = eixoY * -1;
  eixoXA = eixoX/1638.4;
  eixoYA = eixoY/1638.4;
  eixoZA = eixoZ/1638.4;

  velX = eixoXA * 0.98;
  velY = eixoYA * 0.98;
}

void Contagem_Desc(){
  millis_I = millis();
  while(millis_R <= 50){
    ctt++;
    Solic_MPU6050();
    x = x + velX;
    y = y + velY;

    millis_F = millis();
    millis_R = millis_F - millis_I;
  }
  millis_R = 0;
  x = x/ctt;
  y = y/ctt;
  ctt = 0;
  acX = x * 0.05;
  acY = y * 0.05;
  if(acX > 0.05) acXA = acXA + acX;
  if(acY > 0.05) acYA = acYA + acY;
}

