#include "AlphaBLERemote.h"
#include <Arduino.h>
#include <BLEDevice.h>

void ConnectivityState::onConnect(BLEClient *pclient)
{
    // Serial.println("Device is connected");
    connected = true;
}

void ConnectivityState::onDisconnect(BLEClient *pclient)
{
    // Serial.println("Device disconnnect");
    connected = false;
}

bool ConnectivityState::isConnected()
{
    return connected;
}

static const char *LOG_TAG = "MY_SECURITY";

class MySecurity : public BLESecurityCallbacks

{
public:
    MySecurity(AlphaBLERemote *alphaBLERemote)
    {
        m_alphaBLERemote = alphaBLERemote;
    }

private:
    AlphaBLERemote *m_alphaBLERemote;

    uint32_t onPassKeyRequest() { return 123456; }
    void onPassKeyNotify(uint32_t pass_key)
    {
        ESP_LOGE(LOG_TAG, "The passkey Notify number:%d", pass_key);
    }
    bool onConfirmPIN(uint32_t pass_key)
    {
        ESP_LOGI(LOG_TAG, "The passkey YES/NO number:%d", pass_key);
        vTaskDelay(5000);
        return true;
    }
    bool onSecurityRequest()
    {
        ESP_LOGI(LOG_TAG, "Security Request");
        return true;
    }
    void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl)
    {
        if (auth_cmpl.success)
        {
            ESP_LOGI(LOG_TAG, "remote BD_ADDR:");
            esp_log_buffer_hex(LOG_TAG, auth_cmpl.bd_addr,
                               sizeof(auth_cmpl.bd_addr));
            ESP_LOGI(LOG_TAG, "address type = %d", auth_cmpl.addr_type);
        }
        ESP_LOGI(LOG_TAG, "pair status = %s",auth_cmpl.success ? "success" : "fail");
        m_alphaBLERemote->pair_result = auth_cmpl.success;
    }
};

AlphaBLERemote::AlphaBLERemote(String name) : shutter_control_service(SHUTTER_CONTROL_SERVICE_UUID),
                                              shutter_control_write_service(SHUTTER_CONTROL_WRITE_SERVICE_UUID),
                                              shutter_control_notify_service(SHUTTER_CONTROL_NOTIFY_SERVICE_UUID)
{
    remote_name = name;
    pclient->setClientCallbacks(pconnection_state);
}

void AlphaBLERemote::init()
{
    BLEDevice::init(remote_name.c_str());
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

    BLEDevice::setSecurityCallbacks(new MySecurity(this));

    if (nvs.begin())
    {
        log_e("Initialize NVS Success");
        String address = nvs.getString("cameraaddr");

        if (address.length() == 17)
        {
            Serial.printf("Using paired camera address: %s\n", address.c_str());
            camera_address = BLEAddress(address.c_str());
        }
        else
        {
            Serial.println("No camera has been paired yet.");
        }
    }
    else
    {
        log_e("Initialize NVS Failed");
    }
}

// Purpose : Scanning for new BLE devices around.
//           When found -> advdCallback::OnResult
void AlphaBLERemote::scan(unsigned int scan_duration)
{

    log_i("Start BLE scan");
    BLEScan *pBLEScan = BLEDevice::getScan();
    advdCallback *advert_dev_callback = new advdCallback(&ready_to_connect, &camera_address, &camera_name);

    pBLEScan->setAdvertisedDeviceCallbacks(advert_dev_callback); // Retrieve a Scanner and set the callback we want to use to be informed when we have detected a new device.
    pBLEScan->setActiveScan(true);
    pBLEScan->start(scan_duration); // Specify that we want active scanning and start the scan to run for 30 seconds.
}

BLEAddress AlphaBLERemote::getPairedAddress()
{
    return camera_address;
}

String AlphaBLERemote::getPairedAddressString()
{
    return String(camera_address.toString().c_str());
}

/**
 *  Scan and pair camera
 *
 *  This function should be called only when you want to pair with the new camera, and the camera is in remote paring screen.
 *  After paired, camera mac address will be stored in ESP32 NVS for later connection use.
 *
 *  @param  scan_duration : Scan duration in seconds
 */

bool AlphaBLERemote::pair(unsigned int scan_duration)
{
    log_i("Scanning for camera...");
    scan(scan_duration);
    unsigned long start_ms = millis();
    while (!ready_to_connect && millis() - start_ms < scan_duration * 1000)
    {
    }

    if (ready_to_connect)
    {
        log_i("Alpha device found");
        Serial.println(camera_address.toString().c_str());
    }
    else
    {
        log_i("Camera not found");
        return false;
    }

    // Pair camera
    log_i("Pairing..");
    if (pclient->connect(camera_address))
    {
        // Connection success, now check if the device has a shutter control service?
        pRemoteService = pclient->getService(shutter_control_service);
        if (pRemoteService != nullptr)
        {
            bool pairSuccess = false;
             //Wait 30 sec for user to press pair button on camera.
            for (int i = 0; i < 30; i++){
                if (pair_result){
                    pairSuccess = true;
                    break;
                }

                Serial.printf(". ");
                delay(1000);
            }

            if (pairSuccess){
                // Connected and has a shutter control service. Pairing sucess.
                log_i("Camera paring success");
                nvs.setString("cameraaddr", String(camera_address.toString().c_str()));
                if (nvs.commit())
                {
                    log_i("Saving camera's address to NVS success");

                    disconnect();   //Somehow it requires disconnect after initial pairing, otherwise controls does not work.
                    delay(500);

                    //Re-init the BLE
                    // BLEDevice::init(remote_name.c_str());
                    // BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
                    // pclient = BLEDevice::createClient();
                    return true;
                }
                else
                {
                    log_e("Storing camera's address to NVS failed");
                    return false;
                }
            }else{
                log_i("Camera paring failed. (timeout)");
            }

        }
        else
        {
            log_e("Get shutter control service failed");
        }
    }
    else
    {
        log_e("Couldn't connect the BLEClient to the device");
    }
    return false;
}

bool AlphaBLERemote::connect()
{
    if (pclient->connect(camera_address))
    {
        pRemoteService = pclient->getService(shutter_control_service);
        if (pRemoteService != nullptr)
        {
            pRemoteCharacteristic_shutter_control = pRemoteService->getCharacteristic(shutter_control_write_service);
            if (pRemoteCharacteristic_shutter_control != nullptr)
            {
                log_i("Camera connection Success");

                pRemoteCharacteristic_shutter_state = pRemoteService->getCharacteristic(shutter_control_notify_service);
                // pRemoteCharacteristic_shutter_state->registerForNotify(shutterStateNotifyCallback);
                pRemoteCharacteristic_shutter_state->registerForNotify(std::bind(&AlphaBLERemote::shutterStateNotifyCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
                log_i("Registered notify callback");

                return true;
            }
            else
            {
                log_e("Get shutter control service failed");
            }
        }
        else
        {
            log_e("Couldn't acquire the remote main service");
        }
        disconnect();
    }
    return false;
}

void AlphaBLERemote::disconnect()
{
    pclient->disconnect();
}

bool AlphaBLERemote::isConnected()
{
    return pconnection_state->isConnected();
}

bool AlphaBLERemote::checkConnectionReady()
{
    if (!isConnected())
    {
        if (!connect())
        {
            return false;
        }
    }
    return true;
}

void AlphaBLERemote::sendCommand(uint8_t *command, size_t commandLength, unsigned int waitMS)
{
    if (pRemoteCharacteristic_shutter_control != nullptr)
    {
        pRemoteCharacteristic_shutter_control->writeValue(command, commandLength, true);
    }
    delay(waitMS);
}

/** Trigger Camera
 *  If the camera is in photo mode, it will take a single picture.
 *  If the camera is in movie mode, it will start/stop movie recording.
 */

bool AlphaBLERemote::trigger(unsigned int waitFocusMS)
{

    if (!checkConnectionReady())
        return false;

    sendCommand(FOCUS_PRESSED, sizeof(FOCUS_PRESSED), waitFocusMS);
    sendCommand(SHUTTER_PRESSED, sizeof(SHUTTER_PRESSED));
    sendCommand(SHUTTER_RELEASED, sizeof(SHUTTER_RELEASED));
    sendCommand(FOCUS_RELEASED, sizeof(FOCUS_RELEASED));
    return true;
}

bool AlphaBLERemote::video()
{
    if (!checkConnectionReady())
        return false;

    sendCommand(RECORD_PRESSED, sizeof(RECORD_PRESSED));
    sendCommand(RECORD_RELEASED, sizeof(RECORD_RELEASED));

    return true;
}

void AlphaBLERemote::setCameraStateChangedCallback(void (*cbFunction)(uint32_t))
{
    shutterStateCBPTR = cbFunction;
}

void AlphaBLERemote::shutterStateNotifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                                                uint8_t *pData, size_t length, bool isNotify)
{
    // Serial.println("State changed");
    uint32_t state_int = 0;
    state_int = pData[0];
    state_int = state_int << 8 | pData[1];
    state_int = state_int << 8 | pData[2];
    // Serial.println(state_int, HEX);
    // Serial.println(RECORD_STARTED, HEX);

    // switch (state_int){
    //     case RECORD_STARTED:
    //         Serial.println("Record started"); break;
    //     case RECORD_STOPPED:
    //         Serial.println("Record stopped"); break;
    //     case SHUTTER_ACTIVE:
    //         Serial.println("Shutter active"); break;
    //     case SHUTTER_READY:
    //         Serial.println("Shutter ready"); break;
    //     case FOCUS_ACQUIRED:
    //         Serial.println("Focus acquired"); break;
    //     case FOCUS_LOST:
    //         Serial.println("Focus lost"); break;
    //     case FOCUS_FAILED:
    //         Serial.println("Focus failed"); break;
    //     default:{
    //         Serial.print("Unknown shutter state: ");
    //         Serial.println(state_int, HEX);
    //     }

    // }

    if (shutterStateCBPTR != nullptr)
    {
        shutterStateCBPTR(state_int);
    }
}
