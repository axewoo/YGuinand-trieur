#include <Arduino.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

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
}

void loop() {
  // Read raw pins
  int raw0 = digitalRead(0);
  int raw1 = digitalRead(2);
  int raw2 = digitalRead(12);
  int potValue = analogRead(33);
  bool etatBouton0 = !raw0;
  bool etatBouton1 = !raw1;
  bool etatBouton2 = !raw2;

  //Serial.printf("Bouton0: %d, Bouton1: %d, Bouton2: %d\n", etatBouton0, etatBouton1, etatBouton2);
  Serial.printf("Potentiometre: %d\n", potValue);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("B0:"); lcd.print(etatBouton0 ? "ON " : "OFF");
  lcd.print(" B1:"); lcd.print(etatBouton1 ? "ON " : "OFF");

  lcd.setCursor(0, 1);
  lcd.print("B2:"); lcd.print(etatBouton2 ? "ON " : "OFF");

  delay(200);
}
