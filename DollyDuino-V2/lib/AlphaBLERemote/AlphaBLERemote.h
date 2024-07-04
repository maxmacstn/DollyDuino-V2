#ifndef ALPHA_BLE_REMOTE_H_
#define ALPHA_BLE_REMOTE_H_

#include <BLEDevice.h>
#include <Arduino.h>
#include <ArduinoNvs.h>
#include <AdvertisedDeviceCallback.h>

#define SHUTTER_CONTROL_SERVICE_UUID        "8000ff00-ff00-ffff-ffff-ffffffffffff"
#define SHUTTER_CONTROL_WRITE_SERVICE_UUID  "0000ff01-0000-1000-8000-00805f9b34fb"
#define SHUTTER_CONTROL_NOTIFY_SERVICE_UUID "0000ff02-0000-1000-8000-00805f9b34fb"

//Shooting states of the camera
#define RECORD_STARTED  0x02d520
#define RECORD_STOPPED  0x02d500
#define SHUTTER_READY   0x02A000
#define SHUTTER_ACTIVE  0x02A020
#define FOCUS_LOST      0x023F00
#define FOCUS_ACQUIRED  0x023F20
#define FOCUS_FAILED    0x023F40

class ConnectivityState : public BLEClientCallbacks
{
private:
    bool connected = false;

public:
    void onConnect(BLEClient *pclient) override;
    void onDisconnect(BLEClient *pclient) override;
    bool isConnected();

};

class AlphaBLERemote
{
private:
    const BLEUUID shutter_control_service;
    const BLEUUID shutter_control_write_service;
    const BLEUUID shutter_control_notify_service;

    BLEClient *pclient = BLEDevice::createClient();
    ConnectivityState *pconnection_state = new ConnectivityState();
    BLEAddress camera_address = BLEAddress("");
    BLERemoteService *pRemoteService;
    BLERemoteCharacteristic *pRemoteCharacteristic_Pairing;
    BLERemoteCharacteristic *pRemoteCharacteristic_shutter_control;
    BLERemoteCharacteristic *pRemoteCharacteristic_shutter_state;
    ArduinoNvs nvs;

    bool ready_to_connect = false;
    String remote_name = "";
    String camera_name = "";

    uint8_t FOCUS_PRESSED[2] = {0x01, 0x07};
    uint8_t FOCUS_RELEASED[2] = {0x01, 0x06};
    uint8_t SHUTTER_PRESSED[2] = {0x01, 0x09};
    uint8_t SHUTTER_RELEASED[2] = {0x01, 0x08};
    uint8_t RECORD_PRESSED[2] = {0x01, 0x0e};
    uint8_t RECORD_RELEASED[2] = {0x01, 0x0f};

    void (*shutterStateCBPTR)(uint32_t) = nullptr;

    void scan(unsigned int scan_duration);
    bool checkConnectionReady();
    void sendCommand(uint8_t *command, size_t commandLength, unsigned int waitMS = 50);
    void shutterStateNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);

public:
    bool pair_result = false;

    AlphaBLERemote(String name);
    void init();
    bool pair(unsigned int scan_duration);
    bool isConnected();

    bool connect();
    void disconnect();

    bool trigger(unsigned int waitFocusMS = 500);
    bool video();

    void setCameraStateChangedCallback(void (*cbFunctionPTR)(uint32_t));

    BLEAddress getPairedAddress();
    String getPairedAddressString();
};

#endif