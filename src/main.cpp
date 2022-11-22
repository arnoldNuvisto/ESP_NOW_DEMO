/*
 * FUTURE:
 * > implement device reset: note this involves a dialogue between HUB & NODE to complete the reset system-wide
 * >> requires step-wise reversal of the pairing process
 * > implement selective system-wide forgetting of a device
 *
 * NOTE:
 * Generate random 16 hex byte keys: https://www.random.org/bytes/
 * Taskscheduler details: https://github.com/arkhipenko/TaskScheduler
 *
 */
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include <WiFi.h>

/// @brief Serial printing macros
#define SerialD Serial
#define _PM(a)             \
  SerialD.print(millis()); \
  SerialD.print(": ");     \
  SerialD.println(a)
#define _PP(a) SerialD.print(a)
#define _PL(a) SerialD.println(a)
#define _PX(a) SerialD.println(a, HEX)

/// @brief Build parameters
#define _HUB_
// #define _SECURE_
// #define _RESET_NVM_
// #define _RETAIN_MSG_

/// @brief Build configuration
enum deviceType
{
  HUB, // 0
  NODE // 1
};

enum deviceRole
{
  SENSOR,     // 0
  ACTUATOR,   // 1
  CONTROLLER, // 2
  GATEWAY     // 3
};

enum deviceMode
{
  NOT_PAIRED, // 0
  PAIRING,    // 1
  PAIRED      // 2
};

#ifdef _HUB_
#ifdef _SECURE_
#define MAX_PEERS 6
#else
#define MAX_PEERS 20
#endif
#else
#define MAX_PEERS 1
#endif

#ifdef _HUB_
uint16_t device_ID = 1111;
uint8_t device_TYPE = HUB;
uint8_t device_ROLE = CONTROLLER;
uint8_t device_MODE = NOT_PAIRED;
uint8_t device_PEERS = 0;
#else
uint16_t device_ID = 5555;
uint8_t device_TYPE = NODE;
uint8_t device_ROLE = SENSOR;
uint8_t device_MODE = NOT_PAIRED;
uint8_t device_PEERS = 0;
#endif

#ifdef _SECURE_
bool KEY_AUTH = true;
#else
bool KEY_AUTH = false;
#endif

/// @brief Device configuration
struct struct_device_t
{
  uint16_t device_ID;
  uint8_t device_TYPE;
  uint8_t device_ROLE;
  uint8_t device_MODE;
  uint8_t device_PEERS;
  uint8_t device_MAC[6];
};

struct_device_t thisDevice;
struct_device_t pairedDevice[MAX_PEERS];

/// @brief Message configuration
struct struct_payload_t
{
  int count;
  int x;
  int y;
};

enum messageTopic
{
  pairingRequest,       // 0
  pairingValidation,    // 1
  pairingConfirmation,  // 2
  pairingWelcome,       // 3
  sensorData,           // 4
  batteryStatusRequest, // 5
  batteryStatusReport   // 6
};

#ifdef _RETAIN_MSG_
bool retain = true;
#else
bool retain = false;
#endif

struct __attribute__((packed)) struct_message_t
{
  messageTopic topic;
  struct_device_t device;
  struct_payload_t payLoad;
  bool retain;
};

uint8_t counter = 0;

/// @brief ESP-NOW configuration
#define ESPNOW_CHANNEL 1

uint8_t PMK_KEY_STR[ESP_NOW_KEY_LEN] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
uint8_t LMK_KEY_STR[ESP_NOW_KEY_LEN] = {0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44, 0x33, 0x44};

esp_now_peer_num_t numPeers;

uint8_t pairingRequestCount = 0;
uint8_t maxPairingRequests = 5;

/// @brief Function prototypes: MUST be specified BEFORE including TaskScheduler.h
void initESPNOW();
void initWiFi();
void initDevice();
void initTaskScheduler();
void managePairing();
void requestPairing();
void confirmPairing(uint8_t *senderMac, struct_message_t message);
void validatePairing(uint8_t *senderMac, struct_message_t message);
void welcomePeer(uint8_t *senderMac, struct_message_t message);
void finalizePairing(uint8_t *senderMac, struct_message_t message);
void updateTaskRunner();

void OnDataRecv_CB(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void OnDataSent_CB(const uint8_t *mac_addr, esp_now_send_status_t status);

void routeIncomingMessage(uint8_t *senderMac, struct_message_t message);

bool rememberPeer(uint8_t *, boolean);
bool forgetPeer(uint8_t *);
bool sendMessage(uint8_t *, messageTopic, struct_payload_t);
bool updatePeerEncryption(uint8_t *, boolean);

void checkSensors();
void processSensorData(uint8_t *senderMac, struct_message_t message);

void requestBatteryStatus();
void reportBatteryStatus(uint8_t *senderMac, struct_message_t message);
void processBatteryStatus(uint8_t *senderMac, struct_message_t message);

void handleErrorESPNOW(esp_err_t err);
void printMAC(const uint8_t *);

/// @brief TaskScheduler: Uncomment desired TaskScheduler compile options BEFORE including the header file
#define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
#define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
#define _TASK_TIMEOUT           // Support for overall task timeout
// #define _TASK_TIMECRITICAL       // Enable monitoring scheduling overruns
// #define _TASK_WDT_IDS            // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER        // Compile with support for local task storage pointer
// #define _TASK_PRIORITY           // Support for layered scheduling priority
// #define _TASK_MICRO_RES          // Support for microsecond resolution
// #define _TASK_STD_FUNCTION       // Support for std::function (ESP8266 and ESP32 ONLY)
// #define _TASK_DEBUG              // Make all methods and variables public for debug purposes
// #define _TASK_INLINE             // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_OO_CALLBACKS       // Support for dynamic callback method binding
// #define _TASK_DEFINE_MILLIS      // Force forward declaration of millis() and micros() "C" style
// #define _TASK_EXPOSE_CHAIN       // Methods to access tasks in the task chain
// #define _TASK_SCHEDULING_OPTIONS // Support for multiple scheduling options

#include <TaskScheduler.h>

Scheduler runner;

/*
  Default scheduling options cheat sheet...

  TASK_MILLISECOND
  TASK_SECOND
  TASK_MINUTE
  TASK_HOUR
  TASK_IMMEDIATE
  TASK_FOREVER
  TASK_ONCE
  TASK_NOTIMEOUT
  TASK_SCHEDULE     - schedule is a priority, with "catch up" (default)
  TASK_SCHEDULE_NC  - schedule is a priority, without "catch up"
  TASK_INTERVAL     - interval is a priority, without "catch up"
*/
Task requestPairing_TASK(TASK_SECOND * 5, TASK_FOREVER, &managePairing);
Task checkSensors_TASK(TASK_MINUTE, TASK_FOREVER, &checkSensors);
Task requestBatteryStatus_TASK(TASK_SECOND * 33, TASK_FOREVER, &requestBatteryStatus);
// Task onStartMcuVoltage_TASK(TASK_IMMEDIATE, TASK_ONCE, &onStartMcuVoltage_CALLBACK);
// Task onStartDeviceStatus_TASK(TASK_IMMEDIATE, TASK_ONCE, &onStartDeviceStatus_CALLBACK);

/// @brief NVM backup
Preferences prefs;

/// @brief SETUP
void setup()
{
  Serial.begin(115200);
  while (!Serial) // Wait for serial to intialize
  {
  }
  _PP("Starting the ");
  switch (device_TYPE)
  {
  case HUB:
    _PL("HUB");
    break;

  case NODE:
    _PL("NODE");
    break;

  default:
    return;
  }

  initWiFi();

  initESPNOW();

  initDevice();

  initTaskScheduler();
}

/// @brief LOOP
void loop()
{
  runner.execute();
}

/// @brief Start up WiFi
/// @param none
/// @return void
void initWiFi()
{
  WiFi.mode(WIFI_STA);

  WiFi.disconnect();
}

/// @brief Start up ESP-NOW, register callbacks, set PMK key
/// @param : none
/// @return : void
void initESPNOW()
{
  esp_err_t status = esp_now_init();

  if (ESP_OK != status)
  {
    _PL("ERROR: ESPNow init FAILED");

    delay(100);

    ESP.restart(); // software restart
  }

  status = esp_now_register_recv_cb(OnDataRecv_CB);

  if (ESP_OK != status)
  {
    _PL("ERROR: Could not register OnDataRecv_CB callback");

    handleErrorESPNOW(status);
  }

  status = esp_now_register_send_cb(OnDataSent_CB);

  if (ESP_OK != status)
  {
    _PL("ERROR: Could not register OnDataSent_CB callback");

    handleErrorESPNOW(status);
  }

  if (KEY_AUTH)
  {
    status = esp_now_set_pmk(PMK_KEY_STR);

    if (ESP_OK != status)
    {
      _PL("ERROR: Could not set PMK key");

      handleErrorESPNOW(status);
    }
  }
}

/// @brief Initialize the device with data stored in NVM
/// @param : none
/// @return : void
void initDevice()
{
  prefs.begin("setUp", false);

#ifdef _RESET_NVM_
  prefs.clear(); // To remove all preferences under the opened namespace
  //   prefs.remove("keyName"); // To remove one item only
#endif

  byte mac[6];
  WiFi.macAddress(mac);

  // this returns an error if deviceMac is not found in prefs... is OK, but need to capture and fail elegantly
  prefs.getBytes("deviceMac", thisDevice.device_MAC, sizeof(thisDevice.device_MAC));

  int n = memcmp(thisDevice.device_MAC, mac, sizeof(thisDevice.device_MAC)); // returns 0 when matched

  if (n != 0) // if MAC address is NOT already stored in working memory
  {
    prefs.putBytes("deviceMac", mac, sizeof(mac));

    for (int i = 0; i < 6; i++)
    {
      thisDevice.device_MAC[i] = mac[i];
    }
  }

  thisDevice.device_ID = prefs.getUInt("deviceId", device_ID);
  thisDevice.device_TYPE = prefs.getUInt("deviceType", device_TYPE);
  thisDevice.device_ROLE = prefs.getUInt("deviceRole", device_ROLE);
  thisDevice.device_MODE = prefs.getUInt("deviceMode", device_MODE);
  thisDevice.device_PEERS = prefs.getUInt("peersNum", device_PEERS);

  if (thisDevice.device_PEERS > 0)
  {
    size_t listLen = prefs.getBytesLength("peerList");

    char buffer[listLen];

    prefs.getBytes("peerList", buffer, listLen);

    struct_device_t *peerList = (struct_device_t *)buffer;

    for (int i = 0; i < thisDevice.device_PEERS; i++) // loop thru the peerList[] array
    {
      byte pairedMac[6];

      for (int j = 0; j < 6; j++) // Loop thru the device_MAC array for the given pairedDevice[i]
      {
        pairedDevice[i].device_MAC[j] = peerList[i].device_MAC[j];

        pairedMac[j] = pairedDevice[i].device_MAC[j];
      }

      if (!rememberPeer(pairedMac, KEY_AUTH))
      {
        _PL("ERROR: initDevice() -> rememberPeer() failed");
      }
    }
  }
  
  prefs.end();
}

/// @brief Initialize task scheduling
/// @param none
/// @return void
void initTaskScheduler()
{
  runner.init();

  if (thisDevice.device_TYPE == NODE && thisDevice.device_MODE == NOT_PAIRED)
  {
    runner.addTask(requestPairing_TASK);

    requestPairing_TASK.enable();
  }

  if (thisDevice.device_MODE == PAIRED)
  {
    updateTaskRunner();
  }
}

/// @brief Control the pairing process
/// @param none
/// @return void
void managePairing()
{
  ++pairingRequestCount;

  if (pairingRequestCount > maxPairingRequests)
  {
    _PL("ALERT: pairingRequestCount has exceeded maxPairingRequests");

    requestPairing_TASK.disable();

    pairingRequestCount = 0;

    thisDevice.device_MODE = NOT_PAIRED;

    _PL(">>> Cycle device power to restart pairing");
  }
  else
  {
    requestPairing();
  }
}

/// @brief Send a pairing request
/// @param none
/// @return void
void requestPairing()
{
  uint8_t broadcastAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

  if (!rememberPeer(broadcastAddress, false))
  {
    _PL("ERROR: requestPairing() -> rememberPeer() failed");
  }

  struct_payload_t payLoad = {0, 0, 0};

  if (!sendMessage(broadcastAddress, pairingRequest, payLoad))
  {
    _PL("ERROR: requestPairing() -> sendMessage() failed");
  }
  else
  {
    if (pairingRequestCount == 1) // only needed on the first iteration of the request cycle
    {
      thisDevice.device_MODE = PAIRING;
    }
  }

  if (!forgetPeer(broadcastAddress))
  {
    _PL("ERROR: requestPairing() -> forgetPeer() failed");
  }
}

/// @brief Validate a pairing request from a peer
/// @param senderMac
/// @param message
/// @return void
void validatePairing(uint8_t *senderMac, struct_message_t message)
{
  if (!rememberPeer(senderMac, false))
  {
    _PL("ERROR: validatePairing() -> rememberPeer() failed");
  }

  struct_payload_t payLoad = {0, 0, 0};

  if (!sendMessage(senderMac, pairingValidation, payLoad))
  {
    _PL("ERROR: validatePairing() -> sendMessage() failed");
  }
  else
  {
    if (KEY_AUTH)
    {
      if (!updatePeerEncryption(senderMac, KEY_AUTH))
      {
        _PL("ERROR: validatePairing() -> updatePeerEncryption() failed");
      }
    }
  }
}

/// @brief Confirm pairing with a peer
/// @param senderMac
/// @param message
/// @return void
void confirmPairing(uint8_t *senderMac, struct_message_t message)
{
  if (!rememberPeer(senderMac, KEY_AUTH))
  {
    _PL("ERROR: confirmPairing() -> rememberPeer() failed");
  }

  struct_payload_t payLoad = {0, 0, 0};

  if (!sendMessage(senderMac, pairingConfirmation, payLoad))
  {
    _PL("ERROR: confirmPairing() -> sendMessage() failed");

    thisDevice.device_MODE = NOT_PAIRED; // if NODE

    if (!forgetPeer(senderMac))
    {
      _PL("ERROR: confirmPairing() -> !sendMessage() -> forgetPeer() failed");
    }
  }
}

/// @brief Initialize a newly-confirmed peer
/// @param senderMac
/// @param message
/// @return void
void welcomePeer(uint8_t *senderMac, struct_message_t message)
{
  struct_payload_t payLoad = {0, 0, 0};

  if (!sendMessage(senderMac, pairingWelcome, payLoad))
  {
    _PL("ERROR: welcomePeer() failed");

    if (!forgetPeer(senderMac))
    {
      _PL("ERROR: welcomePeer() -> !sendMessage() -> forgetPeer() failed");
    }
  }
  else
  {
    finalizePairing(senderMac, message);
  }
}

/// @brief Finalize the pairing process
/// @param senderMac
/// @param message
/// @return void
void finalizePairing(uint8_t *senderMac, struct_message_t message)
{
  uint8_t idx = thisDevice.device_PEERS;

  bool isMatch;

  if (idx > 0)
  {
    for (int i = 0; i < idx; i++) // loop thru the pairedDevice[] array
    {
      isMatch = false;

      int matchCount = 0;

      for (int j = 0; j < 6; j++) // Loop thru the device_MAC array for the given pairedDevice[i]
      {
        if (pairedDevice[i].device_MAC[j] == senderMac[j])
        {
          matchCount++;
        }

        if (matchCount == 6)
        {
          isMatch = true;
          idx = i; // use this index to UPDATE the paired device array

          break;
        }
      }
    }
  }

  pairedDevice[idx].device_ID = message.device.device_ID;
  pairedDevice[idx].device_TYPE = message.device.device_TYPE;
  pairedDevice[idx].device_ROLE = message.device.device_ROLE;
  pairedDevice[idx].device_MODE = message.device.device_MODE;
  pairedDevice[idx].device_PEERS = message.device.device_PEERS;
  for (int i = 0; i < 6; i++)
  {
    pairedDevice[idx].device_MAC[i] = senderMac[i];
  }

  prefs.begin("setUp", false);

  prefs.putBytes("peerList", pairedDevice, MAX_PEERS * sizeof(struct_device_t));

  if (!isMatch)
  {
    idx++; // bump up peersNum to reflect the addition of a new pairedDevice

    thisDevice.device_PEERS = idx;

    prefs.putUInt("peersNum", idx);
  }

  if (thisDevice.device_MODE != PAIRED)
  {
    thisDevice.device_MODE = PAIRED;
  }

  prefs.putUInt("deviceMode", thisDevice.device_MODE);

  prefs.end();

  updateTaskRunner();
}

/// @brief Add & remove tasks from the task schedule
void updateTaskRunner()
{
  _PL("START: updateTaskRunner()");

  if (thisDevice.device_TYPE == NODE)
  {
    requestPairing_TASK.disable();
  }

  if (thisDevice.device_ROLE == CONTROLLER)
  {
    runner.addTask(requestBatteryStatus_TASK);

    requestBatteryStatus_TASK.enable();
  }

  if (thisDevice.device_ROLE == SENSOR)
  {
    runner.addTask(checkSensors_TASK);

    checkSensors_TASK.enable();
  }
  // include additional tasks here
}

/// @brief Callback function that will be executed when data is received
/// @param mac_addr
/// @param incomingData
/// @param len
/// @return void
void OnDataRecv_CB(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  _PP("INFO: Packet received from: ");
  printMAC(mac_addr);

  uint8_t senderMacAddr[6];

  memcpy(&senderMacAddr, mac_addr, 6);

  struct_message_t messageIn;

  memcpy(&messageIn, incomingData, sizeof(messageIn));

  routeIncomingMessage(senderMacAddr, messageIn);
}

/// @brief Callback function that will be executed when data is sent
/// @param mac_addr
/// @param status
/// @return void
void OnDataSent_CB(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  _PP("INFO: Packet sent to: ");
  printMAC(mac_addr);

  _PP("INFO: Send status: ");
  _PL(status != ESP_NOW_SEND_SUCCESS ? "Delivery Fail" : "Delivery Success");
}

/// @brief Directs incoming message traffic according to message topic
/// @param senderMac
/// @param message
/// @return void
void routeIncomingMessage(uint8_t *senderMac, struct_message_t message)
{
  switch (message.topic)
  {
  case pairingRequest:

    if (thisDevice.device_TYPE == HUB && thisDevice.device_ROLE == CONTROLLER)
    {
      if (thisDevice.device_PEERS < MAX_PEERS)
      {
        validatePairing(senderMac, message);
      }
      else
      {
        _PL("ALERT: Cannot fulfill pairingRequest - already at MAX_PEERS");

        return;
      }
    }
    break;

  case pairingValidation:

    if (thisDevice.device_TYPE == NODE)
    {
      confirmPairing(senderMac, message);
    }
    break;

  case pairingConfirmation:

    if (thisDevice.device_TYPE == HUB && thisDevice.device_ROLE == CONTROLLER)
    {
      welcomePeer(senderMac, message);
    }
    break;

  case pairingWelcome:

    if (thisDevice.device_TYPE == NODE)
    {
      finalizePairing(senderMac, message);
    }
    break;

  case sensorData:

    if (thisDevice.device_TYPE == HUB && thisDevice.device_ROLE == CONTROLLER)
    {
      processSensorData(senderMac, message);
    }
    break;

  case batteryStatusRequest:

    if (thisDevice.device_TYPE == NODE)
    {
      reportBatteryStatus(senderMac, message);
    }
    break;

  case batteryStatusReport:

    if (thisDevice.device_TYPE == HUB && thisDevice.device_ROLE == CONTROLLER)
    {
      processBatteryStatus(senderMac, message);
    }
    break;

  default:
    _PL("ERROR: incoming message contains an invalid topic");

    return;
  }
}

/// @brief Send a message to a target MAC address
/// @param targetMacAddress
/// @param topic
/// @param payLoad
/// @return bool
bool sendMessage(uint8_t *targetMacAddress, messageTopic topic, struct_payload_t payLoad)
{
  bool messageSent = false;

  struct_message_t messageOut;

  messageOut.topic = topic;
  
  messageOut.retain = retain;

  messageOut.device.device_ID = thisDevice.device_ID;
  messageOut.device.device_MODE = thisDevice.device_MODE;
  messageOut.device.device_PEERS = thisDevice.device_PEERS;
  messageOut.device.device_ROLE = thisDevice.device_ROLE;
  messageOut.device.device_TYPE = thisDevice.device_TYPE;
  memcpy(&messageOut.device.device_MAC, targetMacAddress, 6);

  messageOut.payLoad.count = payLoad.count;
  messageOut.payLoad.x = payLoad.x;
  messageOut.payLoad.y = payLoad.y;

  esp_err_t status = esp_now_send(
      targetMacAddress,
      (uint8_t *)&messageOut,
      sizeof(struct_message_t));

  if (status != ESP_OK)
  {
    _PP("ERROR: Message with topic ");
    _PP(topic);
    _PP(" was NOT sent to MAC:");
    printMAC(targetMacAddress);
    handleErrorESPNOW(status);
  }
  else
  {
    messageSent = true;
  }

  return messageSent;
}

/// @brief Register the target peer with the ESP-NOW paired device list
/// @param targetMacAddress
/// @param encrypt
/// @return bool
bool rememberPeer(uint8_t *targetMacAddress, bool encrypt)
{
  bool peerRegistered = false;

  esp_now_peer_info_t peerInfo = {};

  if (!esp_now_is_peer_exist(targetMacAddress))
  {

    memset(&peerInfo, 0, sizeof(peerInfo));

    memcpy(&peerInfo.peer_addr, targetMacAddress, 6);

    peerInfo.ifidx = WIFI_IF_STA;

    peerInfo.channel = ESPNOW_CHANNEL;

    peerInfo.encrypt = encrypt;

    if (encrypt)
    {
      memcpy(&peerInfo.lmk, LMK_KEY_STR, ESP_NOW_KEY_LEN);
    }

    esp_err_t status = esp_now_add_peer(&peerInfo);

    if (status != ESP_OK)
    {
      _PL("ERROR: Failed to add targetMacAddress to ESP-NOW paired devices list");

      handleErrorESPNOW(status);
    }
    else
    {
      peerRegistered = true;
    }
  }
  else
  {
    _PL("ALERT: targetMacAddress already exists in the ESP-NOW paired devices list");
  }

  return peerRegistered;
}

/// @brief update a target peer's encryption status
/// @param targetMacAddress
/// @param encrypt
/// @return bool
bool updatePeerEncryption(uint8_t *targetMacAddress, bool encrypt)
{
  bool peerEncryptionUpdated = false;

  if (!forgetPeer(targetMacAddress))
  {
    _PL("ERROR: updatePeerEncryption() -> forgetPeer() failed");
  }
  else
  {
    if (!rememberPeer(targetMacAddress, encrypt))
    {
      _PL("ERROR: updatePeerEncryption() -> rememberPeer() failed");
    }
    else
    {
      peerEncryptionUpdated = true;
    }
  }

  return peerEncryptionUpdated;
}

/// @brief Remove a target peer from the ESP-NOW paired device list
/// @param targetMacAddress
/// @return bool
bool forgetPeer(uint8_t *targetMacAddress)
{
  bool peerRemoved = false;

  esp_err_t status = esp_now_del_peer(targetMacAddress);

  if (status != ESP_OK)
  {
    _PL("ERROR: Failed to remove peer from the ESP-NOW paired devices list");

    handleErrorESPNOW(status);
  }
  else
  {
    peerRemoved = true;
  }

  return peerRemoved;
}

/// @brief Handle errors related to ESP-NOW
/// @param err
/// @return void
void handleErrorESPNOW(esp_err_t err)
{
  switch (err)
  {
  case ESP_ERR_ESPNOW_NOT_INIT:
    _PL("ESP-NOW ERROR: Not init");
    break;

  case ESP_ERR_ESPNOW_ARG:
    _PL("ESP-NOW ERROR: Argument invalid");
    break;

  case ESP_ERR_ESPNOW_INTERNAL:
    _PL("ESP-NOW ERROR: Internal error");
    break;

  case ESP_ERR_ESPNOW_NO_MEM:
    _PL("ESP-NOW ERROR: Out of memory");
    break;

  case ESP_ERR_ESPNOW_NOT_FOUND:
    _PL("ESP-NOW ERROR: Peer was not found");
    break;

  case ESP_ERR_ESPNOW_IF:
    _PL("ESP-NOW ERROR: Current WiFi interface doesn't match that of peer");
    break;

  case ESP_ERR_ESPNOW_FULL:
    _PL("ESP-NOW ERROR: peer list is full");
    break;

  case ESP_ERR_ESPNOW_EXIST:
    _PL("ESP-NOW ERROR: peer has existed");
    break;

  default:
    _PL("ESP-NOW ERROR: An unknown ESP-NOW-related error has occurred");
    break;
  }

  return;
}

/// @brief Print a MAC address to the Serial Monitor
/// @param mac_addr
/// @return void
void printMAC(const uint8_t *mac_addr)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  _PL(macStr);
}

/// @brief Request battery status from paired devices
void requestBatteryStatus()
{
  struct_payload_t payLoad = {0, 0, 0};

  if (thisDevice.device_PEERS == 0)
  {
    _PL("ALERT: Unable to send requestBatteryStatus >> thisDevice.device_PEERS == 0");
  }
  else
  {
    for (int i = 0; i < thisDevice.device_PEERS; i++)
    {
      uint8_t targetMac[6];

      memcpy(&targetMac, pairedDevice[i].device_MAC, 6);

      sendMessage(targetMac, batteryStatusRequest, payLoad);
    }
  }
}

/// @brief check & report the device's battery voltage
/// @param senderMac
/// @param message
void reportBatteryStatus(uint8_t *senderMac, struct_message_t message)
{
  //! for now...
  struct_payload_t payLoad = {0, 0, 0};

  sendMessage(senderMac, batteryStatusReport, payLoad);
}

/// @brief Process a battery status report
/// @param senderMac
/// @param message
void processBatteryStatus(uint8_t *senderMac, struct_message_t message)
{
  //! for now:
  processSensorData(senderMac, message);
}

/// @brief Read and report sensor values to the CONTROLLER
void checkSensors()
{
  struct_payload_t payLoad;

  //! for now...
  payLoad.count = ++counter;
  payLoad.x = random(0, 50);
  payLoad.y = random(0, 500);

  if (thisDevice.device_PEERS == 0)
  {
    _PL("ALERT: Unable to send sensorData >> thisDevice.device_PEERS == 0");
  }
  else
  {
    for (int i = 0; i < thisDevice.device_PEERS; i++)
    {
      uint8_t targetMac[6];

      memcpy(&targetMac, pairedDevice[i].device_MAC, 6);

      sendMessage(targetMac, sensorData, payLoad);
    }
  }
}

/// @brief Process reported sensor data
/// @param senderMac
/// @param message
void processSensorData(uint8_t *senderMac, struct_message_t message)
{
  _PP("INFO: Message received from: ");
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           senderMac[0], senderMac[1], senderMac[2], senderMac[3], senderMac[4], senderMac[5]);
  _PL(macStr);
  _PP("> message topic: ");
  _PL(message.topic);
  _PP("> device_ID: ");
  _PL(message.device.device_ID);
  _PP("> device_TYPE: ");
  _PL(message.device.device_TYPE);
  _PP("> device_ROLE: ");
  _PL(message.device.device_ROLE);
  _PP("> device_MODE: ");
  _PL(message.device.device_MODE);
  _PP("> device_PEER: ");
  _PL(message.device.device_PEERS);
  _PP("> count: ");
  _PL(message.payLoad.count);
  _PP("> X: ");
  _PL(message.payLoad.x);
  _PP("> Y: ");
  _PL(message.payLoad.y);
}
