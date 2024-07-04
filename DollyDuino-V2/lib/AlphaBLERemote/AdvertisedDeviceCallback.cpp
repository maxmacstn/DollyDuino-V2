#include <AdvertisedDeviceCallback.h>

advdCallback::advdCallback(bool *ready_to_connect, BLEAddress *address_to_connect, String *camera_name)
{
    pready_to_connect = ready_to_connect;
    paddress_to_connect = address_to_connect;
    pcamera_name = camera_name;
}

void advdCallback::onResult(BLEAdvertisedDevice advertisedDevice)
{
    // Serial.print("BLE Advertised Device found: ");
    // Serial.println(advertisedDevice.toString().c_str());

    constexpr std::array<uint8_t, 4> CAMERA_MANUFACTURER_LOOKUP = {0x2D, 0x01, 0x03, 0x00};
    auto mfd = advertisedDevice.getManufacturerData();
    bool isSonyCamera =
        std::equal(CAMERA_MANUFACTURER_LOOKUP.begin(),
                    CAMERA_MANUFACTURER_LOOKUP.end(), mfd.begin());

    // can also add check for it is ready for pairing and display on screen
    if (isSonyCamera) {
        String logMsg = "Found Sony Camera: "; 
        logMsg += String(advertisedDevice.getName().c_str());
        log_i("Found Sony Camera: %s", advertisedDevice.getName().c_str());
        *paddress_to_connect = advertisedDevice.getAddress();
        *pready_to_connect = true;
        *pcamera_name = String(advertisedDevice.getName().c_str());
        advertisedDevice.getScan()->stop();
    }
}