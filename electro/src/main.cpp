#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SPI.h>

// Comandes de l'ADS1292R 
#define CMD_READ_REG 0x20
#define CMD_STOP 0x0A
#define CMD_SDATAC 0x11 //Sortir de lectura continua
#define CMD_RESET 0x06
#define CMD_RDATA 0x12 //llegir dades
//Pins SPI
#define ADS1292_SCLK 18 
#define ADS1292_MISO 19 
#define ADS1292_MOSI 23 
#define ADS1292_DRDY_PIN 16 
#define ADS1292_CS_PIN 5
#define ADS1292_START_PIN 4 
#define ADS1292_PWDN_PIN 22

//Característiques BLE
using namespace std;
#define SERVICE_UUID        "0000180D-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BUFFER_SIZE 50
//capturar dades
float ecg_buffer[BUFFER_SIZE];
float resp_buffer[BUFFER_SIZE];
volatile bool novesDadesECG= false;
volatile bool bufferPle=false;
volatile bool ferCalculs = false;

//Càlculs
unsigned long last_calc_time= 0;
hw_timer_t * timerCalc = NULL;

float tempsPics[BUFFER_SIZE];
float rrIntervals[BUFFER_SIZE];
int countPics = 0;
int rrCount = 0;
#define Fs 250 //frequència de mostreig
#define Ts (1.0/Fs)
float threshold = 0.5; //llindar per detectar pics

//Funcions

//Comunicació amb SPI
void writeCommand(uint8_t cmd) { 
  digitalWrite(ADS1292_CS_PIN, LOW); //Selecciona xip
  delayMicroseconds(2); 
  SPI.transfer(); 
  delayMicroseconds(2); 
  digitalWrite(ADS1292_CS_PIN, HIGH); //Deselecciona
  delay(1);
}

uint8_t readRegister(uint8_t reg) { 
  digitalWrite(ADS1292_CS_PIN, LOW); 
  delayMicroseconds(2); 
  SPI.transfer(CMD_READ_REG | reg); // 0x20→OPCODE per llegir 
  SPI.transfer(0x00); 
  uint8_t value = SPI.transfer(0x00); 
  //enviem una dada qualsevol per poder rebre el valor a la vegada 
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
  SPI.transfer(0x40 | reg); // 0x40→OPCODE per escriure 
  SPI.transfer(0x00); 
  SPI.transfer(value); 
  delayMicroseconds(2); 
  digitalWrite(ADS1292_CS_PIN, HIGH); 
  delay(1); 
  // Verificació: llegir el registre després d'escriure 
  uint8_t readBack = readRegister(reg); 
  Serial.print("Registre 0x"); 
  Serial.print(reg, HEX); 
  Serial.print(" escrit amb 0x"); 
  Serial.print(value, HEX); 
  Serial.print(" | Llegit: 0x"); 
  Serial.println(readBack, HEX); 
  if (readBack != value) { 
     Serial.println(" ERROR: El valor escrit no coincideix amb el llegit!"); 
  }
}


//Crear el servidor BLE i les característiques
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Dispositiu connectat");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Dispositiu desconnectat");
    }
};

//Separem per cores
TaskHandle_t TaskECG;
TaskHandle_t TaskBLE;

//interrupcions
void IRAM_ATTR DRDYinterrupt();
void IRAM_ATTR onTime(); 
//Quan han passat 2 minuts fem els càlculs
void IRAM_ATTR onTime() {
  ferCalculs = true;
}
//capturar dades ECG, avisa quan hi ha noves dades 
void IRAM_ATTR DRDYinterrupt(){
  novesDadesECG= true;
}
//convertir a mv
float convertir_mv(long rawData){
  const float vRef= 2.42;
  int gain=6;
  float fullScale = 8388607.0; 
  float voltage = (rawData / fullScale) * (vRef / gain); // en Volts 
  return voltage * 1000.0; // en mV 
}


void TaskECGcode(void *pvParameters){
  static int index = 0; 
  while(true){
    if(novesDadesECG){
      novesDadesECG=false;
      digitalWrite(ADS1292_CS_PIN, LOW);
      SPI.transfer(CMD_RDATA);
      uint32_t ecgData=0;
      ecgData = SPI.transfer(0x00)<<16; // Llegir primer byte
      ecgData |= (uint32_t)SPI.transfer(0x00)<<8; // Llegir segon byte 
      ecgData |= (uint32_t)SPI.transfer(0x00); // Llegir tercer byte

      uint32_t respData=0;
      respData = SPI.transfer(0x00)<<16; // Llegir primer byte
      respData |= (uint32_t)SPI.transfer(0x00)<<8; // Llegir segon byte 
      respData |= (uint32_t)SPI.transfer(0x00); // Llegir tercer byte

      digitalWrite(ADS1292_CS_PIN, HIGH);
      
      ecg_buffer[index] = convertir_mv(ecgData);
      resp_buffer[index]= convertir_mv(respData);
      index++;
    
      if(index >= BUFFER_SIZE){
        index = 0;
        bufferPle = true;
      }
    }
  }
}

//Enviament de dades per BLE
void TaskBLEcode(void *pvParameters){
  for(;;){
    if(bufferPle && deviceConnected){
      bufferPle = false;
      String data="";
      for(int i=0; i<BUFFER_SIZE; i++){
        data +=String(ecg_buffer[i], 2) + ",";
      }
      for(int i=0; i<BUFFER_SIZE; i++){
        data +=String(resp_buffer[i], 2) + ",";
      } 

      size_t len = data.length();
      std::vector<uint8_t> buf(len);
      memcpy(buf.data(), data.c_str(), len);

      pCharacteristic->setValue(buf.data(), buf.size());
      pCharacteristic->notify();
        
      delay(200); 
    }
    if (ferCalculs && deviceConnected) {
      ferCalculs = false;
      const char* tag = "CALCULA";
      pCharacteristic->setValue((uint8_t*)msg, strlen(msg));
      pCharacteristic->notify();
      //Enviem per bluetooth que ja podem fer els càlculs(cada 2 min)
    }
  }
}


void setup(){
  Serial.begin(115200);
  Serial.println("Iniciant BLE...");
  BLEDevice::init("Marc_Lola");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);; 
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE iniciat i en publicació");
  
  // Inicialitzar ADS1292R i SPI 
  delay(2000);
  SPI.begin(ADS1292_SCLK, ADS1292_MISO, ADS1292_MOSI, ADS1292_CS_PIN);
  SPI.setBitOrder(MSBFIRST);
  //CPOL = 0, CPHA = 1
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV16); //1MHz
  pinMode(ADS1292_DRDY_PIN, INPUT);
  pinMode(ADS1292_CS_PIN, OUTPUT);
  pinMode(ADS1292_START_PIN, OUTPUT);
  pinMode(ADS1292_PWDN_PIN, OUTPUT);
  digitalWrite(ADS1292_CS_PIN, HIGH); //Estableix el CS a HIGH per garantir que el xip no estigui seleccionat
  reinicialitza_ads1292r();
  //Configurem els registres de l'ADS1292R
  digitalWrite(ADS1292_CS_PIN, LOW); 
  delayMicroseconds(2);  //Seleccionem el xip 𝐶𝑆=0 
  writeRegister(0x01, 0x01); // 00000001 Config1: 250 SPS 
  writeRegister(0x02, 0xE0); // 11100000 Config2: Enable RLD, test signal off 
  writeRegister(0x03, 0x19); // 00011000 Lead-Off control 
  writeRegister(0x04, 0x50); // 01010000 Channel 1: Enabled, GAIN = 8, Normal Electrode
  writeRegister(0x05, 0x00); // 00000000 Channel 2: Enabled, Gain=6, Normal Electrode 
  writeRegister(0x06, 0x3F); // 0011111 RLD_SENS escollim els dos canals 
  writeRegister(0x07, 0x2C); // 01001100 
  //writeRegister 0x08
  writeRegister(0x09, 0xC2); // 11000010 RESP1 Resp. control register 1 
  writeRegister(0x0A, 0x05); // 00000101 RESP2 Resp. control register 1, offset off 
  writeRegister(0x0B, 0x00); // 00000000 GPIOs 
  delayMicroseconds(2); 
  digitalWrite(ADS1292_CS_PIN, HIGH);
  Serial.println("ADS1292R Iniciat");

  //interrupció ECG
  attachInterrupt(digitalPinToInterrupt(ADS1292_DRDY_PIN), DRDYinterrupt, FALLING);
  //Interrupció RR
  timerCalc = timerBegin(0, 80, true);
  timerAttachInterrupt(timerCalc, &onTime, true);
  timerAlarmWrite(timerCalc, 120000000, true); //2 minuts en microssegons
  timerAlarmEnable(timerCalc);
  //Core 0
  xTaskCreatePinnedToCore(
    TaskECGcode, //Nom de la funció que implementa la tasca
    "TaskECG", //Nom de la tasca
    4096,
    nullptr,
    1, //Prioritat
    nullptr,//nullptrTasca associada.
    0 //Core 0
  );

  //Core 1
  xTaskCreatePinnedToCore(
    TaskBLEcode,
    "TaskBLE",
    4096,
    nullptr,
    1, //Prioritat
    nullptr,
    1 //Core 1
  );
  
}

void loop(){
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Buscant dispositiu...");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) 
     oldDeviceConnected = deviceConnected;
  //LECTURA ID
  uint8_t id;
  int intents = 0;
  do {
    id = readRegister(0x00); 
    Serial.print("Intent "); 
    Serial.print(intents + 1); 
    Serial.print(": Registre ID (0x00)= 0x"); 
    Serial.println(id, HEX);
    if (id == 0x73) { 
      break;
    } 
    intents++;
    reinicialitza_ads1292r();
    delay(100);
  } while (intents < 3);
  if (id != 0x73) { 
    Serial.println(" Error: No s'ha detectat ID vàlid després de 3 intents.");
  } else {
    Serial.println("ID vàlid detectat");
  }
  delay(2000);
}



