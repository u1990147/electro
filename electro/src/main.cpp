#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SPI.h>



// Comandes de l'ADS1292R 
#define CMD_READ_REG 0x20
#define CMD_STOP 0x0A
#define CMD_SDATAC 0x11
#define CMD_RESET 0x06

//Característiques
using namespace std;
#define HR_SERVICE_UUID        "00000180D-0000-1000-8000-00805F9B34FB"
#define HRcp_CHARACTERISTIC_UUID "000002A39-0000-1000-8000-00805F9B34FB"
#define HRmax_CHARACTERISTIC_UUID "000002A37-0000-1000-8000-00805F9B34FB"
#define HRmesura_CHARACTERISTIC_UUID "000002A8D-0000-1000-8000-00805F9B34FB"

#define RESP_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define RESP_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


//Pins SPI
#define ADS1292_SCLK 18 
#define ADS1292_MISO 19 
#define ADS1292_MOSI 23 
#define ADS1292_DRDY_PIN 16 
#define ADS1292_CS_PIN 5
#define ADS1292_START_PIN 4 
#define ADS1292_PWDN_PIN 22

ads1292r ADS1292R;

//Crear el servidor BLE i les característiques
BLEServer* pServer = NULL;
BLECharacteristic* pHRCharacteristic = NULL;
BLECharacteristic* pRESPCharacteristic = NULL;
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

void setup(){
  Serial.begin(115200);
  Serial.println("Iniciant BLE...");
  BLEDevice::init("Marc_Lola");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pHRService = pServer->createService(HR_SERVICE_UUID);
  BLEService *pRESPService = pServer->createService(RESP_SERVICE_UUID);
  pHRcpCharacteristic = pHRService->createCharacteristic( //HR control point demana que reinci els RR-intervals acumulats
                      HRcp_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pHRcpCharacteristic->addDescriptor(new BLE2902());

  pHRmaxCharacteristic = pHRService->createCharacteristic(
                      HRmax_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pHRmaxCharacteristic->addDescriptor(new BLE2902());

  pHRmesuraCharacteristic = pHRService->createCharacteristic(
                      HRmesura_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pHRmesuraCharacteristic->addDescriptor(new BLE2902());
  pHRService->start();
  
  pRESPCharacteristic = pRESPService->createCharacteristic(
                      RESP_CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
  );
  RESPCharacteristic->addDescriptor(new BLE2902());
  pRESPService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(HR_SERVICE_UUID);
  pAdvertising->addServiceUUID(RESP_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE iniciat i en publicació");



  // Inicialitzar ADS1292R i SPI 
  delay(2000)
  SPI.begin(ADS1292_SCLK, ADS1292_MISO, ADS1292_MOSI, ADS1292_CS_PIN);
  SPI.setBitOrder(MSBFIRST);
  //CPOL = 0, CPHA = 1
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV16); // 1MHz
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
  writeRegister(0x07, 0x00); // LOFF_SENS FALTA
  //writeRegister 0x08
  writeRegister(0x09, 0xC2); // 11000010 RESP1 Resp. control register 1 
  writeRegister(0x0A, 0x05); // 00000101 RESP2 Resp. control register 1, offset off 
  //writeRegister(0x0B, 0x00); // 00000000 GPIOs 1 
  delayMicroseconds(2); 
  digitalWrite(ADS1292_CS_PIN, HIGH);
  serial.println("ADS1292R Iniciat");


}

void loop(){
  delay(500);
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



//Funcions

//Comunicació amb SPI
void writeCommand(uint8_t cmd) { 
  digitalWrite(ADS1292_CS_PIN, LOW); //Selecciona xip
  delayMicroseconds(2); 
  SPI.transfer(cmd); 
  delayMicroseconds(2); 
  digitalWrite(ADS1292_CS_PIN, HIGH); //Deselecciona
  delay(1);
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
   // 0x40→OPCODE per escriure 
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
    Desenvolup. Projectes electrònica32 
  }
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
