#include "LoRaWanMinimal_APP.h"
#include "Arduino.h"

#define TRANSISTOR_PIN GPIO4      // Alimentation capteur
#define LIDAR_PIN      GPIO0      // Sortie PWM Pololu 4071

// Clés TTN (à modifier)
const char* APP_EUI = "0000000000000000";
const char* DEV_EUI = "70B3D57ED0072D6C";
const char* APP_Key = "1763B6C5B89584F4E5F9BC524179B7E3";

// Fréquence d'envoi (secondes)
int temps = 300;

// Paramètres d'acquisition
const uint16_t SAMPLES_PER_SALVE = 20;    // nombre de mesures par salve
const unsigned ACQ_FREQ_HZ       = 50;    // Hz (max 100Hz)
const unsigned long PULSE_TIMEOUT_US = 20000; // timeout pour pulseIn

// LoRaWAN variables
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
static uint8_t counter=0;

// Clefs converties
const int AppEUI_len = strlen(APP_EUI);
const int DevEUI_len = strlen(DEV_EUI);
const int AppKey_len = strlen(APP_Key);
byte AppEUI_clefConvertie[8];
byte DevEUI_clefConvertie[8];
byte AppKey_clefConvertie[16];
uint8_t appEui[8], devEui[8], appKey[16];

// Basse conso
TimerEvent_t sleepTimer;
bool sleepTimerExpired;
static void wakeUp() { sleepTimerExpired = true; }
static void lowPowerSleep(uint32_t ms) {
  sleepTimerExpired = false;
  TimerInit(&sleepTimer, &wakeUp);
  TimerSetValue(&sleepTimer, ms);
  TimerStart(&sleepTimer);
  while (!sleepTimerExpired) lowPowerHandler();
  TimerStop(&sleepTimer);
}

// Utilitaires clefs
void convertirClef(const char* clef, byte* clefConvertie, int longueur) {
  for (int i = 0; i < longueur; i += 2) {
    char byteStr[3] = {clef[i], clef[i + 1], '\0'};
    clefConvertie[i / 2] = strtol(byteStr, NULL, 16);
  }
}
void remplirTableau(uint8_t* tableau, byte* clefConvertie, int longueur) {
  for (int i = 0; i < longueur / 2; i++) tableau[i] = clefConvertie[i];
}

// Conversion PWM → distance en mm
int16_t pulseToMm(unsigned long t_us) {
  if (t_us == 0)   return -2;    // timeout
  if (t_us > 1850) return -1;    // hors portée
  long d = (long)(t_us - 1000) * 2;
  if (d < 0) d = 0;
  if (d > 1300) d = 1300;
  return (int16_t)d;
}

// Calcule min / max / mean*10 / sd*10 / median sur les valeurs filtrées
// validCount = nombre de valeurs gardées
void computeStats(int16_t *a, size_t n,
                  int16_t &minv, int16_t &maxv,
                  int16_t &mean10, int16_t &sd10,
                  int16_t &median,
                  uint8_t &validCount)
{
  // 1) Collecter uniquement les valeurs brutes valides (>= 0)
  static int16_t vals[256];
  size_t m = 0;
  for (size_t i = 0; i < n; i++) {
    int16_t v = a[i];
    if (v >= 0) vals[m++] = v;
  }

  if (m == 0) {
    minv = maxv = mean10 = sd10 = median = -1;
    validCount = 0;
    return;
  }

  // 2) Trier pour trouver la médiane initiale
  for (size_t i = 1; i < m; i++) {
    int16_t key = vals[i];
    int j = i - 1;
    while (j >= 0 && vals[j] > key) { vals[j + 1] = vals[j]; j--; }
    vals[j + 1] = key;
  }
  int16_t median0 = (m & 1) ? vals[m / 2]
                            : (int16_t)((vals[m / 2 - 1] + vals[m / 2]) / 2);

  // 3) Ne garder que les valeurs dans ±10 mm de la médiane initiale
  static int16_t filt[256];
  size_t k = 0;
  for (size_t i = 0; i < m; i++) {
    int16_t v = vals[i];
    if (abs(v - median0) <= 10) filt[k++] = v;
  }

  validCount = k;
  if (k == 0) {
    minv = maxv = mean10 = sd10 = median = -1;
    return;
  }

  // 4) Trier les valeurs filtrées pour min, max et médiane finale
  for (size_t i = 1; i < k; i++) {
    int16_t key = filt[i];
    int j = i - 1;
    while (j >= 0 && filt[j] > key) { filt[j + 1] = filt[j]; j--; }
    filt[j + 1] = key;
  }
  int16_t medianF = (k & 1) ? filt[k / 2]
                            : (int16_t)((filt[k / 2 - 1] + filt[k / 2]) / 2);

  // 5) Calcul min / max / mean / sd sur les valeurs filtrées
  int16_t minT = filt[0];
  int16_t maxT = filt[k - 1];
  long sum = 0;
  for (size_t i = 0; i < k; i++) sum += filt[i];
  float mean = (float)sum / (float)k;

  double acc = 0;
  for (size_t i = 0; i < k; i++) {
    double d = (double)filt[i] - mean;
    acc += d * d;
  }
  float sd = sqrt(acc / (double)k);

  // 6) Sorties
  minv   = minT;
  maxv   = maxT;
  mean10 = (int16_t)lroundf(mean * 10.0f);
  sd10   = (int16_t)lroundf(sd   * 10.0f);
  median = medianF;
}



///////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);

  convertirClef(APP_EUI, AppEUI_clefConvertie, AppEUI_len);
  convertirClef(DEV_EUI, DevEUI_clefConvertie, DevEUI_len);
  convertirClef(APP_Key, AppKey_clefConvertie, AppKey_len);
  remplirTableau(appEui, AppEUI_clefConvertie, AppEUI_len);
  remplirTableau(devEui, DevEUI_clefConvertie, DevEUI_len);
  remplirTableau(appKey, AppKey_clefConvertie, AppKey_len);

  pinMode(TRANSISTOR_PIN,OUTPUT);
  digitalWrite(TRANSISTOR_PIN,LOW);
  pinMode(LIDAR_PIN, INPUT);

  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);
  LoRaWAN.setAdaptiveDR(true);

  while (!LoRaWAN.isJoined()) {
    Serial.println("Joining...");
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
    if (!LoRaWAN.isJoined()) {
      Serial.println("JOIN FAILED, sleep 30 s");
      lowPowerSleep(30000);
    }
  }
  Serial.println("JOINED");
}

///////////////////////////////////////////////////
void loop() {
  counter++;
  delay(10);
  uint8_t voltage = getBatteryVoltage()/50; //Voltage in %

  // Allume capteur et attend la stabilisation
  digitalWrite(TRANSISTOR_PIN, HIGH);
  delay(1000);

  // Salve de mesures
  int16_t buf[SAMPLES_PER_SALVE];
  for (uint16_t i = 0; i < SAMPLES_PER_SALVE; i++) {
    unsigned long p = pulseIn(LIDAR_PIN, HIGH, PULSE_TIMEOUT_US);
    buf[i] = pulseToMm(p);
    delay(1000 / ACQ_FREQ_HZ);
  }

  digitalWrite(TRANSISTOR_PIN, LOW); // Éteint le capteur

  Serial.print("Valeurs brutes : ");
  for (uint16_t i = 0; i < SAMPLES_PER_SALVE; i++) {
    Serial.print(buf[i]);
    if (i < SAMPLES_PER_SALVE - 1) Serial.print(", ");
  }
  Serial.println();

  //Calcul statistique
  int16_t minv, maxv, mean10, sd10, median;
  uint8_t validCount;
 
  computeStats(buf, SAMPLES_PER_SALVE, minv, maxv, mean10, sd10, median, validCount);
  Serial.printf("min=%d mm  max=%d mm  mean=%.1f mm  sd=%.1f mm  median=%d mm  valid=%u\n",minv, maxv, mean10 / 10.0f, sd10 / 10.0f, median, validCount);

  // Prépare le payload
  uint8_t lora_data[12];
  lora_data[0] = highByte(minv);   lora_data[1] = lowByte(minv);
  lora_data[2] = highByte(maxv);   lora_data[3] = lowByte(maxv);
  lora_data[4] = highByte(mean10); lora_data[5] = lowByte(mean10);
  lora_data[6] = highByte(sd10);   lora_data[7] = lowByte(sd10);
  lora_data[8] = highByte(median); lora_data[9] = lowByte(median);
  lora_data[10] = validCount;       lora_data[11] = voltage;
  // Envoi LoRa
  bool requestack = counter < 2; // Ack demandé pour les deux premiers envois
  if (LoRaWAN.send(sizeof(lora_data), lora_data, 1, requestack)) {
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
  }

  // Sommeil avant la prochaine salve
  lowPowerSleep(temps * 1000UL);
}
