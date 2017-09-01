#include <LiquidCrystal.h>
#include <DS18B20.h>
#include <OneWire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

const byte DS18B20_PIN = 12;
const byte DS18B20ADDRESS[8] = {0x28, 0x7C, 0x1, 0x50, 0x5, 0x0, 0x0, 0xDB};
MPU6050 mpu;
bool blinkState = false;
unsigned long timerMpu=0;
unsigned long timerLed=0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprMax[3];        // [yaw, pitch, roll]   yaw/pitch/roll in degrees

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
volatile bool guziorPressed = false;
void dmpDataReady() 
{
    mpuInterrupt = true;
}
void guziorInterrupt()
{
    guziorPressed = true;
}

LiquidCrystal lcd(11, 6, 7, 8, 9, 10); //Informacja o podłączeniu nowego wyświetlacza
OneWire onewire(DS18B20_PIN);
DS18B20 ds18b20(&onewire);



// lcd.clear()  //czyści zawartość wyświetlacza
void setup() {
  Serial.begin(115200);
  //MPU6050  
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz) 
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) 
  {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
  }    

  //DS18B20
  ds18b20.begin();
  ds18b20.request(DS18B20ADDRESS);

  //LCD
  lcd.begin(16, 2); //Deklaracja typu
  lcd.setCursor(0, 0); //Ustawienie kursora
  lcd.print("Temp: "); //Wyświetlenie tekstu
  lcd.setCursor(6, 0); //Ustawienie kursora
  lcd.print("25.6");

  //LED
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);  

  digitalWrite(A1, INPUT_PULLUP);
  digitalWrite(A2, INPUT_PULLUP);
  digitalWrite(A3, INPUT_PULLUP);
  //GUZIOR
  digitalWrite(3, HIGH);
  attachInterrupt(1, guziorInterrupt, FALLING);
}
 
void loop() 
{
  if (ds18b20.available())
  {
    //Serial.println("wypisuje temperature");
    float temp = ds18b20.readTemperature(DS18B20ADDRESS);
    lcd.setCursor(6, 0); //Ustawienie kursora
    lcd.print(temp);
    //Serial.println(temp);
    ds18b20.request(DS18B20ADDRESS);
  }
  else if(millis()-timerMpu > 200)
  {
    timerMpu = millis();
    if (dmpReady && !mpuInterrupt && (fifoCount < packetSize)) 
    {
    }
    else
    {
      //Serial.println("wchodzi w mpu6050");
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
  
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
  
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
      {
          // reset so we can continue cleanly
          Serial.println("reset fifo!!!!!!!!!!!!!!!!!!!!!!!!!");
          mpu.resetFIFO();      
      }
      // otherwise, check for DMP data ready interrupt (this should happen frequently) 
      else if (mpuIntStatus & 0x02) 
      {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;  
       
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //lcd.clear();
        if(ypr[1] > 0)
        {
          lcd.setCursor(0, 1);
          lcd.print(ypr[1]*180/M_PI);
          lcd.setCursor(11, 1);
          lcd.print("0.000");
        }
        else
        {
          lcd.setCursor(11, 1);
          lcd.print(ypr[1]*-180/M_PI);
          lcd.setCursor(0, 1);
          lcd.print("0.000");
        }
      }
    }
  }
  else if(guziorPressed == true)
  {
//    digitalWrite(A1, HIGH);
//    digitalWrite(A2, HIGH);
//    digitalWrite(A3, HIGH);
//    delay(100);
//    digitalWrite(A1, LOW);
//    digitalWrite(A2, LOW);
//    digitalWrite(A3, LOW);
//    delay(50);
//    guziorPressed = false;
    if(millis() - timerLed <200)
    {
      digitalWrite(A1, LOW);
      digitalWrite(A2, HIGH);
      digitalWrite(A3, HIGH);
      Serial.println("myslałem, że: 2");
    }
    else if(millis() - timerLed <400)
    {
      digitalWrite(A1, HIGH);
      digitalWrite(A2, LOW);
      digitalWrite(A3, HIGH);
      Serial.println("myslałem, że: 3");
    }
    else if(millis() - timerLed <600)
    {
      digitalWrite(A1, HIGH);
      digitalWrite(A2, HIGH);
      digitalWrite(A3, LOW);
      Serial.println("myslałem, że: 4");
    }
    else if(millis()-timerLed <800)
    { 
      guziorPressed = false;
      digitalWrite(A1, HIGH);
      digitalWrite(A2, HIGH);
      digitalWrite(A3, HIGH);
      Serial.println("myslałem, że: 5");
    }
    else if(millis() - timerLed >= 800)
    {
      Serial.println("myslałem, że: 1");
      timerLed = millis();
    }
  }
}











  

