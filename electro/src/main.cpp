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


using namespace std;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

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
  pAdvertising->addServiceUUID(SERVICE_UUID);
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
  ADS1292R.ads1292Reset(ADS1292_CS_PIN, ADS1292_START_PIN, ADS1292_PWDN_PIN); // Reset de l'ADS1292R
  //Configurem els registres de l'ADS1292R
  //FALTA ID
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

  if (deviceConnected && !oldDeviceConnected) oldDeviceConnected = deviceConnected;
}



//Funcions

void ads1292r::ads1292Reset(const int chipSelect,const int resetPin,const int startPin) 
{
  ads1922Reset(resetpin);
  delay(100);
  ads1292DisableStart(startPin); 
  ads1292EnableStart(startPin); 
  ads1292HardStop(startPin); 
  ads1292StartDataConvCommand(chipSelect); 
  ads1292SoftStop(chipSelect); 
  delay(50); 
  ads1292StopReadDataContinuous(chipSelect); //Atura el mode de lectura continua (SDATAC)
  delay(300);
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
   if (readBack != value) { Serial.println(" ERROR: El valor escrit no coincideix amb el llegit!"); Desenvolup. Projectes electrònica32 }
}

uint8_t readRegister(uint8_t reg) { 
  digitalWrite(ADS1292_CS_PIN, LOW); 
  delayMicroseconds(2); 
  SPI.transfer(0x20 | reg); // 0x20→OPCODE per llegir 
  SPI.transfer(0x00); 
  uint8_t value = SPI.transfer(0x00); 
  //enviem una dada qualsevol per poder rebre el 
  //valor a la vegada 
  delayMicroseconds(2); 
  digitalWrite(ADS1292_CS_PIN, HIGH); 
  return value; 
}
