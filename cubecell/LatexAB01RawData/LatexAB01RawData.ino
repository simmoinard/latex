#include "Arduino.h"
#define TRANSISTOR_PIN GPIO4     // Alimentation du capteur (commande transistor)
#define LIDAR_PIN      GPIO0     // Sortie PWM du Pololu 4071

#define LORAWAN_REGION        REGION_EU868   // ex: REGION_EU868, REGION_US915…
#define LORAWAN_CLASS         CLASS_A        // ou CLASS_C
#define LORAWAN_DEVEUI        CUSTOM         // tu gères devEui dans ton code
#define LORAWAN_NETMODE       OTAA           // ou ABP
#define LORAWAN_ADR_ON        true           // équivaut à ADR:ON
#define LORAWAN_UPLINKMODE    CONFIRMED      // ou UNCONFIRMED
#define LORAWAN_AT_SUPPORT    0              // 0 = OFF, 1 = ON
#define LORAWAN_RGB           ACTIVE         // ou OFF


void setup() {
  Serial.begin(115200);

  // Broche qui alimente le capteur
  pinMode(TRANSISTOR_PIN, OUTPUT);
  digitalWrite(TRANSISTOR_PIN, LOW); // capteur éteint au démarrage

  // Entrée pour le signal PWM
  pinMode(LIDAR_PIN, INPUT);
}

void loop() {
  // Allume le capteur via le transistor
  digitalWrite(TRANSISTOR_PIN, HIGH);
  delay(1500);  // temps de stabilisation (~1,5 s)

  // Mesure la durée du niveau haut (pulse HIGH) en microsecondes
  unsigned long pulse_us = pulseIn(LIDAR_PIN, HIGH, 20000UL); // timeout 20 ms

  // Convertit en distance (mm) selon la datasheet
  int16_t distance_mm;
  if (pulse_us == 0) {
    distance_mm = -2;            // timeout → pas de pulse
  } else if (pulse_us > 1850) {
    distance_mm = -1;            // hors portée
  } else {
    long d = (long)(pulse_us - 1000) * 2;
    if (d < 0) d = 0;
    if (d > 1300) d = 1300;
    distance_mm = (int16_t)d;
  }

  // Affiche les mesures brutes et converties
  Serial.printf("Pulse: %lu us  ->  Distance: %d mm\n", pulse_us, distance_mm);

  // Éteint le capteur entre les mesures pour économiser la batterie
  digitalWrite(TRANSISTOR_PIN, LOW);

  delay(1000); // intervalle entre deux mesures
}
