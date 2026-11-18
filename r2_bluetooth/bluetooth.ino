#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "robot_types.h"

// R1_new と同じService/Characteristic UUID。R1用のコントローラーアプリをそのまま使える。
constexpr char BLE_DEVICE_NAME[] = "R2";
constexpr char BLE_SERVICE_UUID[] = "12345678-1234-1234-1234-123456789abc";
constexpr char BLE_CHARACTERISTIC_UUID[] = "abcdefab-1234-1234-1234-abcdefabcdef";

// BLEのコールバックは別タスクで動くため、loop()とはvolatile変数で受け渡す。
volatile char receivedCommand = 0;
volatile bool disconnected = false;

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String rx = characteristic->getValue();
    if (rx.length() == 0) return;
    receivedCommand = rx[0];
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onDisconnect(BLEServer *server) override {
    disconnected = true;
    // 切断後も再接続できるよう、アドバタイズを再開する。
    BLEDevice::startAdvertising();
  }
};

void bluetooth_Init() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *service = server->createService(BLE_SERVICE_UUID);

  BLECharacteristic *characteristic = service->createCharacteristic(
    BLE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  characteristic->setCallbacks(new CommandCallbacks());
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->start();
}

char bluetooth_TakeCommand() {
  const char command = receivedCommand;
  receivedCommand = 0;
  return command;
}

bool bluetooth_TakeDisconnected() {
  const bool value = disconnected;
  disconnected = false;
  return value;
}
