#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SPI.h>
#include <arduinoFFT.h>

// Comandes de l'ADS1292R
#define CMD_READ_REG 0x20
#define CMD_STOP 0x0A
#define CMD_SDATAC 0x11 // Sortir lectura continua
#define CMD_RESET 0x06
#define CMD_RDATA 0x12 // llegir dades

// Pins SPI
#define ADS1292_SCLK 18
#define ADS1292_MISO 19
#define ADS1292_MOSI 23
#define ADS1292_DRDY_PIN 16
#define ADS1292_CS_PIN 5
#define ADS1292_START_PIN 4
#define ADS1292_PWDN_PIN 22

// Característiques BLE
typedef std::string String;
#define SERVICE_UUID        "0000180D-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Tamany buffers
define BUFFER_SIZE 50
#define ECG_BUF_SZ BUFFER_SIZE

// FFT i interpolació
#define N 256  // punts per interpolar i FFT
arduinoFFT<double> FFT;
double vReal[N];
double vImag[N];
float potencies[N/2];  // Potència espectral
double f[N/2];       // Freqüències corresponents

// Captura dades
float ecg_buffer[ECG_BUF_SZ];
float resp_buffer[ECG_BUF_SZ];
volatile bool novesDadesECG = false;
volatile bool bufferPle = false;

// HRV i càlculs
float tempsPics[BUFFER_SIZE];
float rrIntervals[BUFFER_SIZE];
int countPics = 0;
int rrCount = 0;

float potenciaLF = 0;
float potenciaHF = 0;
float stress_val = 0;

// Temporitzador
hw_timer_t * timerCalc = NULL;
volatile bool ferCalculs = false;

// Funcions utilitàries
void IRAM_ATTR onTime() { ferCalculs = true; }

float convertir_mv(uint32_t rawData) {
  const float vRef = 2.42;
  const int gain = 6;
  const float fullScale = 8388607.0;
  return (rawData / fullScale) * (vRef / gain) * 1000.0;
}

// SPI amb ADS1292R
void writeCommand(uint8_t cmd) {
  digitalWrite(ADS1292_CS_PIN, LOW);
  delayMicroseconds(2);
  SPI.transfer(cmd);
  delayMicroseconds(2);
  digitalWrite(ADS1292_CS_PIN, HIGH);
  delay(1);
}

uint8_t readRegister(uint8_t reg) {
  digitalWrite(ADS1292_CS_PIN, LOW);
  delayMicroseconds(2);
  SPI.transfer(CMD_READ_REG | reg);
  SPI.transfer(0x00);
  uint8_t value = SPI.transfer(0x00);
  delayMicroseconds(2);
  digitalWrite(ADS1292_CS_PIN, HIGH);
  return value;
}

void reinicialitza_ads1292r() {
  digitalWrite(ADS1292_PWDN_PIN, LOW);
  delay(2);
  digitalWrite(ADS1292_PWDN_PIN, HIGH);
  delay(1000);
  writeCommand(CMD_RESET);
  delay(10);
  writeCommand(CMD_SDATAC);
  delay(10);
  digitalWrite(ADS1292_START_PIN, HIGH);
}

void writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(ADS1292_CS_PIN, LOW);
  delayMicroseconds(2);
  SPI.transfer(0x40 | reg);
  SPI.transfer(0x00);
  SPI.transfer(value);
  delayMicroseconds(2);
  digitalWrite(ADS1292_CS_PIN, HIGH);
  delay(1);
  uint8_t back = readRegister(reg);
  if (back != value) Serial.printf("Error registre 0x%02X: escrit 0x%02X, llegit 0x%02X\n", reg, value, back);
}

// BLE
enum { ECG_CHAR_SZ = BUFFER_SIZE };
BLEServer* pServer;
BLECharacteristic* pCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* s) override { deviceConnected = true; Serial.println("Connectat"); }
  void onDisconnect(BLEServer* s) override { deviceConnected = false; Serial.println("Desconnectat"); }
};

// Tasques
TaskHandle_t TaskECG, TaskBLE, TaskCalc;

// ISR SPI data ready
void IRAM_ATTR DRDYinterrupt() { novesDadesECG = true; }

// Task ECG: llegeix per SPI
void TaskECGcode(void* pv) {
  int idx = 0;
  while (true) {
    if (novesDadesECG) {
      novesDadesECG = false;
      digitalWrite(ADS1292_CS_PIN, LOW);
      SPI.transfer(CMD_RDATA);
      uint32_t ecg = SPI.transfer(0x00) << 16;
      ecg |= SPI.transfer(0x00) << 8;
      ecg |= SPI.transfer(0x00);
      uint32_t rsp = SPI.transfer(0x00) << 16;
      rsp |= SPI.transfer(0x00) << 8;
      rsp |= SPI.transfer(0x00);
      digitalWrite(ADS1292_CS_PIN, HIGH);
      ecg_buffer[idx] = convertir_mv(ecg);
      resp_buffer[idx] = convertir_mv(rsp);
      idx = (idx + 1) % ECG_BUF_SZ;
      if (idx == 0) bufferPle = true;
    }
  }
}

// Task BLE: envia valors i estrès
void TaskBLEcode(void* pv) {
  while (true) {
    if (bufferPle && deviceConnected) {
      bufferPle = false;
      std::string s;
      for (int i=0;i<ECG_BUF_SZ;i++) s += String(ecg_buffer[i],2) + ",";
      for (int i=0;i<ECG_BUF_SZ;i++) s += String(resp_buffer[i],2) + ",";
      s += String(potenciaLF,2) + "," + String(potenciaHF,2) + "," + String(stress_val,2);
      pCharacteristic->setValue((uint8_t*)s.c_str(), s.length());
      pCharacteristic->notify();
      delay(200);
    }
  }
}

// Detecta pics R
void detectarPics() {
  for (int i=1;i<ECG_BUF_SZ-1;i++) {
    if (ecg_buffer[i]>0.5 && ecg_buffer[i]>ecg_buffer[i-1] && ecg_buffer[i]>ecg_buffer[i+1]) {
      if (countPics < BUFFER_SIZE) tempsPics[countPics++] = i * (1.0/Fs);
      if (countPics>=2 && rrCount<BUFFER_SIZE) rrIntervals[rrCount++] = tempsPics[countPics-1] - tempsPics[countPics-2];
    }
  }
}

// Interpolació a 4Hz
void interpolarRR() {
  if (rrCount<2) return;
  float dt = 1.0/4.0;
  int j=0;
  for (int i=0;i<N/2;i++) {
    f[i] = i * dt;
    float t = i * dt;
    while (j<rrCount-1 && tempsPics[j+1]<t) j++;
    float x0=tempsPics[j], x1=tempsPics[j+1];
    float y0=rrIntervals[j], y1=rrIntervals[j+1];
    vReal[i] = y0 + (y1-y0)*(t-x0)/(x1-x0);
    vImag[i] = 0.0;
  }
}

// Càlcul FFT i magnituds
void calcularFFT() {
  FFT.Windowing(vReal, N/2, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, N/2, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, N/2);
  for (int i=0;i<N/2;i++) potencies[i] = vReal[i];
}

// Suma potències LF/HF
void calcularPotencies() {
  potenciaLF=0; potenciaHF=0;
  for (int i=0;i<N/2;i++) {
    double freq = f[i];
    if (freq>=0.04 && freq<=0.15) potenciaLF += potencies[i];
    else if (freq>0.15 && freq<=0.4) potenciaHF += potencies[i];
  }
  stress_val = (potenciaHF>0)? potenciaLF/potenciaHF : 0;
}

// Task càlculs HRV
void TaskCalcCode(void* pv) {
  while (true) {
    if (ferCalculs) {
      ferCalculs = false;
      detectarPics();
      interpolarRR();
      calcularFFT();
      calcularPotencies();
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Inici BLE
  BLEDevice::init("ESP32-ECG");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService* svc = pServer->createService(SERVICE_UUID);
  pCharacteristic = svc->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  svc->start();
  BLEDevice::startAdvertising();

  // SPI i ADS1292R
  SPI.begin(ADS1292_SCLK, ADS1292_MISO, ADS1292_MOSI, ADS1292_CS_PIN);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  pinMode(ADS1292_CS_PIN, OUTPUT); digitalWrite(ADS1292_CS_PIN, HIGH);
  pinMode(ADS1292_DRDY_PIN, INPUT_PULLDOWN);
  pinMode(ADS1292_START_PIN, OUTPUT);
  pinMode(ADS1292_PWDN_PIN, OUTPUT);
  reinicialitza_ads1292r();
  writeRegister(0x01, 0x01); writeRegister(0x02, 0xE0);
  writeRegister(0x03, 0x19); writeRegister(0x04, 0x50);
  writeRegister(0x05, 0x00); writeRegister(0x06, 0x3F);
  writeRegister(0x07, 0x2C); writeRegister(0x09, 0xC2);
  writeRegister(0x0A, 0x05); writeRegister(0x0B, 0x00);

  attachInterrupt(digitalPinToInterrupt(ADS1292_DRDY_PIN), DRDYinterrupt, FALLING);
  timerCalc = timerBegin(0, 80, true);
  timerAttachInterrupt(timerCalc, &onTime, true);
  timerAlarmWrite(timerCalc, 900000000, true);
  timerAlarmEnable(timerCalc);

  // Crear tasques
  xTaskCreatePinnedToCore(TaskECGcode, "ECG", 4096, NULL, 1, &TaskECG, 0);
  xTaskCreatePinnedToCore(TaskBLEcode, "BLE", 4096, NULL, 1, &TaskBLE, 1);
  xTaskCreatePinnedToCore(TaskCalcCode, "Calc", 4096, NULL, 1, &TaskCalc, 1);
}

void loop() {
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) oldDeviceConnected = deviceConnected;

  // Comprovació ID ADS1292R
  uint8_t id; int tries=0;
  do {
    id = readRegister(0x00);
    if (id == 0x73) break;
    tries++; reinicialitza_ads1292r(); delay(100);
  } while (tries < 3);
  Serial.println(id==0x73?"ID OK":"ID ERROR");
  delay(2000);
}
