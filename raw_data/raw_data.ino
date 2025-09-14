// Test Pololu #4071 sur Arduino Nano
// Affiche la distance estimée en millimètres

const uint8_t PIN_S1 = 2;          // sortie OUT du capteur
const unsigned long PULSE_TIMEOUT_US = 20000; // attend jusqu'à 50 ms

// Conversion durée HIGH (µs) -> distance (mm)
int16_t pulseToMm(unsigned long t_us) {
  if (t_us == 0)   return -2;    // timeout : aucun pulse reçu
  if (t_us > 1850) return -1;    // >1850 µs ≈ pas d’objet détecté
  long d = (long)(t_us - 1000) * 2;  // formule Pololu/Sharp : 2 × (µs - 1000)
  if (d < 0) d = 0;
  if (d > 1300) d = 1300;        // limite supérieure
  return (int16_t)d;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_S1, INPUT);
  Serial.println("Lecture capteur Pololu #4071 en mm...");
}

void loop() {
  unsigned long pulse_us = pulseIn(PIN_S1, HIGH, PULSE_TIMEOUT_US);
  int16_t distance_mm   = pulseToMm(pulse_us);

  Serial.print("Pulse: ");
  Serial.print(pulse_us);
  Serial.print(" us  ->  Distance: ");
  Serial.print(distance_mm);
  Serial.println(" mm");

  delay(200); // 5 mesures par seconde
}
