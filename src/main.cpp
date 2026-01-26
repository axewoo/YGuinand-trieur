#include <Arduino.h>
#include "rgb_lcd.h"
#include <ESP32Encoder.h>

ESP32Encoder encoder;

rgb_lcd lcd;
const int PWM_CHANNEL = 0;     // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_FREQ = 25000;      // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 11; // 11 bits of resolution: 0-2047

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
  
  encoder.attachHalfQuad(23, 19);
  encoder.setCount(0); // Initialize encoder count to 0

  // Configure PWM on pin 27: 25 kHz, 8-bit resolution (0-255)
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(27, PWM_CHANNEL);
}

void loop() {
  // Read raw pins
  int raw0 = digitalRead(0);
  int raw1 = digitalRead(2);
  int raw2 = digitalRead(12);
  int potValue = (analogRead(33)/2);
  bool etatBouton0 = !raw0;
  bool etatBouton1 = !raw1;
  bool etatBouton2 = !raw2;

  // Read encoder value
  int32_t encoderValue = encoder.getCount();

  // Serial output for encoder
  Serial.printf("Encoder: %d\n", encoderValue);

  // Control motor based on potentiometer value
    ledcWrite(PWM_CHANNEL, potValue); //0 -> 2047
    digitalWrite(26, etatBouton0 ? LOW : HIGH); // Set motor direction
    digitalWrite(25, etatBouton1 ? LOW : HIGH); // Enable/Disable motor (active low)

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("B0:"); lcd.print(potValue);
  lcd.print(" Enc:"); lcd.print(encoderValue);

  delay(200);
}
