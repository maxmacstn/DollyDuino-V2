#ifndef ALPHA_ADVD_BLE_CALLBACK_H_
#define ALPHA_ADVD_BLE_CALLBACK_H_

#include <BLEDevice.h>
#include <Arduino.h>


class advdCallback : public BLEAdvertisedDeviceCallbacks
{
private:
    BLEAddress *paddress_to_connect;
    bool *pready_to_connect;
    String *pcamera_name;

public:
    advdCallback(bool *ready_to_connect, BLEAddress *address_to_connect, String *camera_name);
    void onResult(BLEAdvertisedDevice advertisedDevice);
};

#endif