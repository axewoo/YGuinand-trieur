#include <Arduino.h>
#include "rgb_lcd.h"
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

ESP32Encoder encoder;
Servo myservo;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
rgb_lcd lcd;

const int PWM_CHANNEL = 0;     
const int PWM_FREQ = 25000;      // 25 kHz frequency
const int PWM_RESOLUTION = 11; // 11 bits of resolution: 0-2047
const int servoPin = 13; // Pin connected to the servo signal wire


unsigned long lastMovementTime = 0;
const unsigned long RETURN_DELAY = 3000; 

void posinit(void);
void posinitreverse(void);

void setup() {

  // Initialise la liaison avec le terminal
  Serial.begin(115200);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  Wire.begin(21,22); //SDA, SCL
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);
  lcd.setRGB(0, 0, 0);

  // Initialise les entrées/sorties (enable internal pull-ups)
  pinMode(0, INPUT_PULLUP); // Bouton0 (active low)
  pinMode(2, INPUT_PULLUP); // Bouton1 (active low)
  pinMode(12, INPUT_PULLUP); // Bouton2 (active low)
  pinMode(33, INPUT); //Potentiomètre

  pinMode(27, OUTPUT); //PWM 
  pinMode(26, OUTPUT); //Signal Sens
  pinMode(25, OUTPUT); //Motor Disable
  pinMode(13, OUTPUT); //Servo Motor

  pinMode(36, INPUT); //Capteur CNY70

  encoder.attachFullQuad(23, 19);
  encoder.setCount(0); // Initialize encoder count to 0

  // Configure PWM on pin 27: 25 kHz, 8-bit resolution (0-255)
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(27, PWM_CHANNEL);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin);
 posinit();
}

void loop() {
  uint16_t r, g, b, c, colorTemp, lux;

  // Read raw pins
  int raw0 = digitalRead(0);
  int raw1 = digitalRead(2);
  int raw2 = digitalRead(12);
  int potValue = (analogRead(33)); // Map potentiometer value to 0-180 degrees for servo control
  int cnyValue = analogRead(36);

  bool etatBouton0 = !raw0;
  bool etatBouton1 = !raw1;
  bool etatBouton2 = !raw2;

  // Read encoder value
  int32_t encoderValue = encoder.getCount();
  lcd.setRGB(0, 0, 0);
  lcd.clear();
  myservo.write(88);

if (etatBouton1 == 1) //Mesure de couleur
{

    while (encoderValue > -150)
    {
      digitalWrite(26, LOW); // Set direction
      int distance = encoderValue - -150;
      int pwmSpeed = (distance > 30) ? 700 : 250; // Fast until 30 units away, then slow
      ledcWrite(PWM_CHANNEL, pwmSpeed);
      encoderValue = encoder.getCount();
    }
    ledcWrite(PWM_CHANNEL, 0); // Stop motor
    encoder.setCount(0);
    delay(500); // Short delay between movements

    while (encoderValue < 670)
    {
      digitalWrite(26, HIGH); // Set direction
      int distance = 700 -encoderValue;
      int pwmSpeed = (distance > 50) ? 650 : 300; // Fast until 30 units away, then slow
      ledcWrite(PWM_CHANNEL, pwmSpeed);
      encoderValue = encoder.getCount();
    }
    ledcWrite(PWM_CHANNEL, 0); // Stop motor
    delay(500); // Short delay between movements  
  posinitreverse();
    while (encoderValue > -150)
    {
      digitalWrite(26, LOW); // Set direction
      int distance = encoderValue - -150;
      int pwmSpeed = (distance > 30) ? 700 : 250; // Fast until 30 units away, then slow
      ledcWrite(PWM_CHANNEL, pwmSpeed);
      encoderValue = encoder.getCount();
    }
    ledcWrite(PWM_CHANNEL, 0); // Stop motor
    encoder.setCount(0);
    delay(500); // Short delay between movements
  posinit();
    int i = 0;
    while(i!=5)
    {
      tcs.getRawData(&r, &g, &b, &c);
      colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
        Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
        Serial.println(" ");
        lcd.setRGB(0, 20, 100);
        lcd.setCursor(0, 0);
        lcd.print("Color Temp:");
        lcd.print(colorTemp);

      i++;
      delay(1000);
    }
    if (colorTemp<=6000)
    {
      lcd.setRGB(255, 0, 0); // Red for low color temperature
      lcd.setCursor(0, 1);
      lcd.print("Autre Balle");
          while (encoderValue < 650)
    {
      digitalWrite(26, HIGH); // Set direction
      int distance = 650 -encoderValue;
      int pwmSpeed = (distance > 40) ? 650 : 300; // Fast until 30 units away, then slow
      ledcWrite(PWM_CHANNEL, pwmSpeed);
      encoderValue = encoder.getCount();
    }
    ledcWrite(PWM_CHANNEL, 0); // Stop motor
    delay(500); // Short delay between movements  
    posinitreverse();
        while (encoderValue > -50)
    {
      digitalWrite(26, LOW); // Set direction
      int distance = encoderValue - -50;
      int pwmSpeed = (distance > 30) ? 700 : 250; // Fast until 30 units away, then slow
      ledcWrite(PWM_CHANNEL, pwmSpeed);
      encoderValue = encoder.getCount();
    }
    ledcWrite(PWM_CHANNEL, 0); // Stop motor
    encoder.setCount(0);
      myservo.write(50);
      delay(1000);
      myservo.write(88);

    delay(500); // Short delay between movements
    posinit();
    }
    else if (colorTemp>6000)
    {
      lcd.setRGB(0, 150, 20); // Green for white ball
      lcd.setCursor(0, 1);
      lcd.print("Balle Blanche !");
          while (encoderValue < 750)
    {
      digitalWrite(26, HIGH); // Set direction
      int distance = 750 -encoderValue;
      int pwmSpeed = (distance > 40) ? 650 : 300; // Fast until 30 units away, then slow
      ledcWrite(PWM_CHANNEL, pwmSpeed);
      encoderValue = encoder.getCount();
    }
    ledcWrite(PWM_CHANNEL, 0); // Stop motor
    delay(500); // Short delay between movements
    encoder.setCount(0);
          while (encoderValue > -650)
    {
      digitalWrite(26, LOW); // Set direction
      int distance = encoderValue- -650;
      int pwmSpeed = (distance > 40) ? 650 : 300; // Fast until 30 units away, then slow
      ledcWrite(PWM_CHANNEL, pwmSpeed);
      encoderValue = encoder.getCount();
    }
    while (analogRead(36)>2000)
    {}
    posinit();
    posinitreverse();
    }
}

posinit();
}

void posinit(void){
  digitalWrite(25, HIGH); // Enable motor
  digitalWrite(26, HIGH); // Set direction
  
  while (analogRead(36) < 2000){
    ledcWrite(PWM_CHANNEL, 650); // Higher speed

    delay(1); // Small delay to allow sensor reading
  }
  ledcWrite(PWM_CHANNEL, 0); // Stop motor
  encoder.setCount(0); // Reset encoder count
  return;
}

void posinitreverse(void){
  digitalWrite(25, HIGH); // Enable motor
  digitalWrite(26, LOW); // Set direction
  
  while (analogRead(36) < 2000){
    ledcWrite(PWM_CHANNEL, 650); // Higher speed

    delay(1); // Small delay to allow sensor reading
  }
  ledcWrite(PWM_CHANNEL, 0); // Stop motor
  encoder.setCount(0); // Reset encoder count
  return;
}

