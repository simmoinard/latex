#include <SPI.h>
#include <SD.h>
#include <math.h>

// ==================== Paramètres utilisateur ====================
const uint8_t PIN_LIDAR      = 4;      // Capteur Pololu 4071
const uint8_t SD_CS          = 10;     // Chip Select carte SD

const unsigned SAMPLES_PER_SALVE = 10;      // X : nombre d'échantillons par salve. Moins de 100
const unsigned ACQ_FREQ_HZ       = 50;      // Y : fréquence d'acquisition (mesures/seconde). Ne pas dépasser 100Hz
const unsigned long PULSE_TIMEOUT_US = 20000; // Z : timeout pulseIn (µs).

const unsigned long DELAY_1ST_MINUTE   = 1000UL;    // période entre salves 0-1 min  (1 s)
const unsigned long DELAY_2   = 5000UL;    // 1-10 min (5 s)
const unsigned long DELAY_3   = 30000UL;   // 10-60 min (30 s)
const unsigned long DELAY_4   = 60000UL;   // >60 min (1 min)

// =================================================================

// Compteurs
unsigned long salveID = 0;     // ID de salve
unsigned long startMillis;     // temps au lancement

// Fichier SD
File dataFile;
char filename[20];

// Conversion PWM → distance (mm)
int16_t pulseToMm(unsigned long t_us) {
  if (t_us == 0)   return -2;    // pas de pulse
  if (t_us > 1850) return -1;    // hors portée
  long d = (long)(t_us - 1000) * 2;
  if (d < 0) d = 0;
  if (d > 1300) d = 1300;
  return (int16_t)d;
}

// Calcul statistiques simples
void computeStats(int16_t *a, size_t n,
                  int16_t &min, int16_t &max,
                  float &mean, float &sd, int16_t &median)
{
  if (n == 0) { min = max = median = -1; mean = sd = NAN; return; }

  long sum = 0;
  min = INT16_MAX;
  max = INT16_MIN;

  for (size_t i = 0; i < n; i++) {
    if (a[i] < min) min = a[i];
    if (a[i] > max) max = a[i];
    sum += a[i];
  }
  mean = (float)sum / n;

  double acc = 0;
  for (size_t i = 0; i < n; i++) {
    double d = a[i] - mean;
    acc += d * d;
  }
  sd = sqrt(acc / n);

  // médiane (tri insertion)
  for (size_t i = 1; i < n; i++) {
    int16_t key = a[i];
    int j = i;
    while (j > 0 && a[j-1] > key) {
      a[j] = a[j-1];
      j--;
    }
    a[j] = key;
  }
  median = (n % 2) ? a[n/2] : (a[n/2 - 1] + a[n/2]) / 2;
}

// Crée un nom unique data_XX.csv
void makeUniqueName(char *outName) {
  for (uint8_t i = 1; i < 100; i++) {
    sprintf(outName, "data_%02d.csv", i);
    if (!SD.exists(outName)) return;
  }
  sprintf(outName, "data_99.csv");
}

// Calcule la période entre deux salves selon l’âge du programme
unsigned long computeDelay() {
  unsigned long elapsed = millis() - startMillis;
  if (elapsed < 60000UL)            return DELAY_1;
  else if (elapsed < 600000UL)      return DELAY_2;
  else if (elapsed < 3600000UL)     return DELAY_3;
  else                              return DELAY_4;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LIDAR, INPUT);
  startMillis = millis();

  Serial.println(F("=== Initialisation ==="));

  // Vérification capteur (10 à 50 cm au démarrage)
  Serial.print(F("Test capteur... "));
  unsigned long t = pulseIn(PIN_LIDAR, HIGH, PULSE_TIMEOUT_US);
  int16_t d = pulseToMm(t);
  if (d < 100 || d > 500) {
    Serial.println(F("ECHEC : distance hors 10-50 cm"));
  } else {
    Serial.print(F("OK : ")); Serial.print(d); Serial.println(F(" mm"));
  }

  // Initialisation SD
  Serial.print(F("Initialisation SD... "));
  while (!SD.begin(SD_CS)) {
    Serial.println(F("echec"));
    delay(1000);
  }
  Serial.println(F("OK"));

  // Création du fichier
  makeUniqueName(filename);
  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.println(F("Impossible de créer le fichier"));
    while (1);
  }

  // Entête CSV
  dataFile.print(F("salve_id,start_ms,end_ms,min,max,mean,sd,median"));
  for (unsigned i = 1; i <= SAMPLES_PER_SALVE; i++) {
    dataFile.print(F(",raw_"));
    if (i < 10) dataFile.print('0');
    dataFile.print(i);
  }
  dataFile.println();
  dataFile.close();

  Serial.print(F("Fichier créé : "));
  Serial.println(filename);
  Serial.println(F("======================="));
}

void loop() {
  // Début de salve
  unsigned long salveStart = millis();
  int16_t buf[SAMPLES_PER_SALVE];

  // Acquisition des X mesures
  for (unsigned i = 0; i < SAMPLES_PER_SALVE; i++) {
    unsigned long p = pulseIn(PIN_LIDAR, HIGH, PULSE_TIMEOUT_US);
    buf[i] = pulseToMm(p);
    delay(1000 / ACQ_FREQ_HZ);
  }
  unsigned long salveEnd = millis();

  // Calcul statistiques
  int16_t min, max, median;
  float mean, sd;
  computeStats(buf, SAMPLES_PER_SALVE, min, max, mean, sd, median);

  // Écriture dans le fichier
  dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    dataFile.print(salveID); dataFile.print(',');
    dataFile.print(salveStart); dataFile.print(',');
    dataFile.print(salveEnd); dataFile.print(',');
    dataFile.print(min); dataFile.print(',');
    dataFile.print(max); dataFile.print(',');
    dataFile.print(mean, 2); dataFile.print(',');
    dataFile.print(sd, 2); dataFile.print(',');
    dataFile.print(median);
    for (unsigned i = 0; i < SAMPLES_PER_SALVE; i++) {
      dataFile.print(',');
      dataFile.print(buf[i]);
    }
    dataFile.println();
    dataFile.close();
  }

  // Affichage console
  Serial.print(F("Salve ")); Serial.print(salveID);
  Serial.print(F(" : min=")); Serial.print(min);
  Serial.print(F(" mm, max=")); Serial.print(max);
  Serial.print(F(" mm, mean=")); Serial.print(mean,1);
  Serial.print(F(" mm, sd=")); Serial.print(sd,1);
  Serial.print(F(" mm, median=")); Serial.println(median);

  salveID++;

  // Attente avant la prochaine salve
  unsigned long waitTime = computeDelay();
  delay(waitTime);
}
