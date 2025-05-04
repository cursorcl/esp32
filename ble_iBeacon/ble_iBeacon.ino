#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEDevice.h>
#include <BLEAdvertisedDevice.h>

// Función auxiliar para convertir un String de Arduino a formato hexadecimal
String toHexString(const String &value) {
  String res;
  // Recorremos cada carácter del String
  for (int i = 0; i < value.length(); i++) {
    // Obtenemos el byte en la posición i
    byte b = (byte)value.charAt(i);

    // Si es menor que 0x10, añadimos un '0' para alinear a 2 dígitos
    if (b < 0x10) {
      res += "0";
    }

    // Convertimos el byte a hex y lo concatenamos
    res += String(b, HEX);

    // Entre cada byte (excepto el último) añadimos un espacio
    if (i < value.length() - 1) {
      res += " ";
    }
  }

  return res;
}
float calculateDistance(int txPower, int rssi) {
  if (rssi == 0) return -1.0;
  float ratio = rssi * 1.0 / txPower;
  if (ratio < 1.0) return pow(ratio, 10);
  else return (0.89976) * pow(ratio, 7.7095) + 0.111;
}
// Función para intentar parsear datos de iBeacon
void parseIBeaconData(BLEAdvertisedDevice advertisedDevice) {
    String mData = advertisedDevice.getManufacturerData();
    Serial.print("Manufacturer Data (Hex): ");
    Serial.println(toHexString(mData));
    int rssi = advertisedDevice.getRSSI();
  // Formato iBeacon (total 25 bytes):
  // [0-1]: 0x4C 0x00 (Apple Company ID)
  // [2-3]: 0x02 0x15 (iBeacon indicator)
  // [4-19]: 16-byte UUID
  // [20-21]: Major
  // [22-23]: Minor
  // [24]: Tx Power (1 byte)
  if (
    // mData.length() == 25 &&
      (uint8_t)mData.charAt(0) == 0x4C && (uint8_t)mData.charAt(1) == 0x00 && 
      (uint8_t)mData.charAt(2) == 0x02 && (uint8_t)mData.charAt(3) == 0x15) 
  {
    // Extraer el UUID (16 bytes, posiciones [4..19])
    char uuid[37]; // 36 chars + null terminator
    sprintf(uuid,
            "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
            (uint8_t)mData.charAt(4),  (uint8_t)mData.charAt(5),  (uint8_t)mData.charAt(6),  (uint8_t)mData.charAt(7),
            (uint8_t)mData.charAt(8),  (uint8_t)mData.charAt(9),  (uint8_t)mData.charAt(10), (uint8_t)mData.charAt(11),
            (uint8_t)mData.charAt(12), (uint8_t)mData.charAt(13), (uint8_t)mData.charAt(14), (uint8_t)mData.charAt(15),
            (uint8_t)mData.charAt(16), (uint8_t)mData.charAt(17), (uint8_t)mData.charAt(18), (uint8_t)mData.charAt(19));

    // Extraer el Major (2 bytes, posiciones [20..21])
    uint16_t major = ((uint8_t)mData.charAt(20) << 8) | (uint8_t)mData.charAt(21);
    // Extraer el Minor (2 bytes, posiciones [22..23])
    uint16_t minor = ((uint8_t)mData.charAt(22) << 8) | (uint8_t)mData.charAt(23);

    // Extraer Tx Power (byte [24], firmado en iBeacon)
    int8_t txPower = (int8_t)mData.charAt(24);
    int distance = calculateDistance(txPower, rssi);
    // Imprimimos resultados
    Serial.println("----- iBeacon detectado -----");
    Serial.print("UUID:   "); Serial.println(uuid);
    Serial.print("Major:  "); Serial.println(major);
    Serial.print("Minor:  "); Serial.println(minor);
    Serial.print("TxPower:"); Serial.println(txPower);
    Serial.print("RSSI:"); Serial.println(rssi);
    Serial.print("Distance:"); Serial.println(rssi);
    Serial.println("-----------------------------");
  }
}

// Callback para cada dispositivo BLE encontrado
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    Serial.println("============================================");
    Serial.print("Direccion BLE: ");
    Serial.println(advertisedDevice.getAddress().toString().c_str());
    String nombre = advertisedDevice.getName().c_str();
    if (nombre.length() > 0) {
      Serial.print("Nombre: ");
      Serial.println(nombre);
    }

    // Si es un iBeacon, estará en Manufacturer Data
    if (advertisedDevice.haveManufacturerData()) {
        parseIBeaconData(advertisedDevice);
      }

    // Para Eddystone-UID/URL/AltBeacon, puedes revisar getServiceData() o getServiceUUID().
    // Eddystone usa 'Service UUID: 0xFEAA' + serviceData. 
  }
};

BLEScan* pBLEScan;
bool isScanning = false;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("Iniciando BLE Scanner para iBeacon...");

  BLEDevice::init("WT32-ETH01_BLE");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);

  // Iniciar primer escaneo de 5 seg (no bloqueante)
  pBLEScan->start(5, false);
  isScanning = true;
  Serial.println("Primer escaneo iniciado...");
}

void loop() {
    Serial.println("Escaneo finalizado. Reiniciando...");

    // Limpiar resultados (opcional)
    pBLEScan->clearResults();

    // Relanzar escaneo
    pBLEScan->start(5, false);
}
