#include <Arduino.h>
#include "rgb_lcd.h"
#include <ESP32Encoder.h>

ESP32Encoder encoder;

rgb_lcd lcd;
const int PWM_CHANNEL = 0;     
const int PWM_FREQ = 25000;      // 25 kHz frequency
const int PWM_RESOLUTION = 11; // 11 bits of resolution: 0-2047

const int PositionInitiale = 0;
const int Position1 = 100;
const int Position2 = 200;
const int Position3 = 300;
const int Position4 = 400;
const int Position5 = 500;
const int Position6 = 600;
const int Position7 = 700;


unsigned long lastMovementTime = 0;
const unsigned long RETURN_DELAY = 3000; 

void posinit(void);

void setup() {
  // Initialise la liaison avec le terminal
  Serial.begin(115200);

  // Initialise l'écran LCD
  Wire1.setPins(15, 5);
  lcd.begin(16, 2, LCD_5x8DOTS, Wire1);
  lcd.setRGB(0, 20, 100);

  // Initialise les entrées/sorties (enable internal pull-ups)
  pinMode(0, INPUT_PULLUP); // Bouton0 (active low)
  pinMode(2, INPUT_PULLUP); // Bouton1 (active low)
  pinMode(12, INPUT_PULLUP); // Bouton2 (active low)
  pinMode(33, INPUT); //Potentiomètre

  pinMode(27, OUTPUT); //PWM 
  pinMode(26, OUTPUT); //Signal Sens
  pinMode(25, OUTPUT); //Motor Disable
  
  pinMode(36, INPUT); //Capteur CNY70

  encoder.attachFullQuad(23, 19);
  encoder.setCount(0); // Initialize encoder count to 0

  // Configure PWM on pin 27: 25 kHz, 8-bit resolution (0-255)
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(27, PWM_CHANNEL);

 posinit();
}

void loop() {
  // Read raw pins
  int raw0 = digitalRead(0);
  int raw1 = digitalRead(2);
  int raw2 = digitalRead(12);
  int potValue = (analogRead(33)/2);
  int cnyValue = analogRead(36);

  bool etatBouton0 = !raw0;
  bool etatBouton1 = !raw1;
  bool etatBouton2 = !raw2;

  // Read encoder value
  int32_t encoderValue = encoder.getCount();

  // Serial output for encoder
  Serial.printf("Encoder: %d\n", encoderValue);
  Serial.printf("CNY70: %d\n", cnyValue);

  digitalWrite(25, etatBouton2 ? LOW : HIGH); 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Po:"); lcd.print(potValue);
  lcd.print(" Enc:"); lcd.print(encoderValue);
  lcd.setCursor(0, 1);
  lcd.print("CNY70:"); lcd.print(cnyValue);


 
if (etatBouton0 == 1) //Sens Horaire
{
  for (int target = -100; target >= -700; target -= 100)
  {
    while (encoderValue > target)
    {
      digitalWrite(26, LOW); // Set direction
      ledcWrite(PWM_CHANNEL, 600); // Higher speed
      encoderValue = encoder.getCount();
    }
    ledcWrite(PWM_CHANNEL, 0); // Stop motor
    delay(500); // Short delay between movements
  }
  
  // Move down until analog sensor triggers
  while (analogRead(36) < 2000)
  {
    digitalWrite(26, LOW); // Set direction
    ledcWrite(PWM_CHANNEL, 600); // Higher speed
    encoderValue = encoder.getCount();
  }
  ledcWrite(PWM_CHANNEL, 0); // Stop motor
  encoder.setCount(0); // Reset encoder count
  
}

if (etatBouton1 == 1) //Sens AntiHoraire
{
  for (int target = 100; target <= 700; target += 100)
  {
    while (encoderValue < target)
    {
      digitalWrite(26, HIGH); // Set direction
      ledcWrite(PWM_CHANNEL, 620); // Higher speed
      encoderValue = encoder.getCount();
    }
    ledcWrite(PWM_CHANNEL, 0); // Stop motor
    delay(500); // Short delay between movements
  }
  
  // Move until CNY70 triggers
  while (analogRead(36) < 2000)
  {
    digitalWrite(26, HIGH); // Set direction
    ledcWrite(PWM_CHANNEL, 620); // Higher speed
    encoderValue = encoder.getCount();
  }
  ledcWrite(PWM_CHANNEL, 0); // Stop motor
  encoder.setCount(0); // Reset encoder count
}


  if (cnyValue > 2000){
    encoder.setCount(0); // Reset encoder count
  }
  delay(100);
}


void posinit(void){
  digitalWrite(25, HIGH); // Enable motor
  digitalWrite(26, HIGH); // Set direction
  
  while (analogRead(36) < 2000){
    ledcWrite(PWM_CHANNEL, 620); // Higher speed

    delay(10); // Small delay to allow sensor reading
  }
  ledcWrite(PWM_CHANNEL, 0); // Stop motor
  encoder.setCount(0); // Reset encoder count
  return;
}