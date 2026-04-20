#if defined(TARGET_MAVLINK_COMPANION)

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "MAVLink.h"
#include "options.h"
#include "logging.h"

/////////// CONSTANTS ///////////

#define ESPNOW_MAX_PAYLOAD 250

/////////// GLOBALS ///////////

uint8_t txBackpackAddress[6];
esp_now_peer_info_t txPeerInfo;

// Uplink: UART -> MAVLink message queue -> ESP-NOW
mavlink_message_t uplinkMsgBuf[MAVLINK_BUF_SIZE];
uint8_t           uplinkMsgCount = 0;
unsigned long     lastUplinkFlush = 0;

// Downlink: ESP-NOW -> ring buffer -> MAVLink frame -> UART
uint8_t downlinkBuf[1024];
volatile uint16_t downlinkHead = 0;
volatile uint16_t downlinkTail = 0;

// Stats (updated from ISR callbacks, read/reset from loop)
volatile int32_t  rssiSum = 0;
volatile uint16_t rssiCount = 0;
volatile uint16_t rxRetryCount = 0;
volatile uint16_t rxFrameCount = 0;
volatile uint32_t sendOkCount = 0;
volatile uint32_t sendFailCount = 0;
volatile uint8_t  lastSigMode = 0;  // 0=11b/g, 1=HT(11n)
volatile uint8_t  lastRate = 0;
uint32_t lastStatsPrint = 0;

#define STATS_UART Serial1
#define STATS_UART_TX_PIN 43
#define STATS_UART_BAUD 115200

/////////// ESP-NOW CALLBACKS ///////////

// Downlink: ESP-NOW -> ring buffer (unsafe to call Serial.write from WiFi task on USB CDC)
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  if (memcmp(mac_addr, txBackpackAddress, 6) != 0)
  {
    return;
  }
  for (int i = 0; i < data_len; i++)
  {
    uint16_t nextHead = (downlinkHead + 1) % sizeof(downlinkBuf);
    if (nextHead == downlinkTail)
    {
      break; // buffer full, drop remaining bytes
    }
    downlinkBuf[downlinkHead] = data[i];
    downlinkHead = nextHead;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)
    sendOkCount++;
  else
    sendFailCount++;
}

// Promiscuous RX: capture RSSI and 802.11 retry bit from TX backpack frames
void promiscuousRxCb(void *buf, wifi_promiscuous_pkt_type_t type)
{
  if (type != WIFI_PKT_MGMT) return;

  wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
  // Address 2 (source/transmitter) is at offset 10 in the 802.11 MAC header
  if (memcmp(pkt->payload + 10, txBackpackAddress, 6) != 0) return;

  rssiSum += pkt->rx_ctrl.rssi;
  rssiCount++;
  rxFrameCount++;
  lastSigMode = pkt->rx_ctrl.sig_mode;
  lastRate = pkt->rx_ctrl.rate;
  // Frame Control byte 1, bit 3 = Retry bit
  if (pkt->payload[1] & 0x08)
    rxRetryCount++;
}

/////////// MAC ADDRESS SETUP ///////////

void SetCompanionMACAddress()
{
  uint8_t mac[6];
  memcpy(mac, firmwareOptions.uid, 6);

  // First byte must be even for unicast
  mac[0] = mac[0] & ~0x01;
  // Differentiate from TX backpack by XOR on last byte
  mac[5] ^= 0x01;

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
  WiFi.begin("network-name", "pass-to-network", 1);
  WiFi.disconnect();

  esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_11M_S);
  esp_wifi_set_mac(WIFI_IF_STA, mac);

  // TX backpack address is the standard UID-derived MAC (first byte even, no XOR)
  memcpy(txBackpackAddress, firmwareOptions.uid, 6);
  txBackpackAddress[0] = txBackpackAddress[0] & ~0x01;
}

/////////// UPLINK FLUSH ///////////

void flushUplinkBuf()
{
  if (uplinkMsgCount == 0)
  {
    return;
  }

  uint8_t buf[ESPNOW_MAX_PAYLOAD];
  uint8_t bufLen = 0;

  uint8_t tmpBuf[MAVLINK_MAX_PACKET_LEN];
  for (uint8_t i = 0; i < uplinkMsgCount; i++)
  {
    uint16_t msgLen = mavlink_msg_to_send_buffer(tmpBuf, &uplinkMsgBuf[i]);

    // If adding this message would overflow, send current buffer first
    if (bufLen + msgLen > sizeof(buf) && bufLen > 0)
    {
      esp_now_send(txBackpackAddress, buf, bufLen);
      bufLen = 0;
    }

    // If a single message exceeds 250 bytes, fragment it
    if (msgLen > sizeof(buf))
    {
      uint16_t offset = 0;
      while (offset < msgLen)
      {
        uint16_t chunkLen = min((uint16_t)(sizeof(buf)), (uint16_t)(msgLen - offset));
        esp_now_send(txBackpackAddress, tmpBuf + offset, chunkLen);
        offset += chunkLen;
      }
    }
    else
    {
      memcpy(buf + bufLen, tmpBuf, msgLen);
      bufLen += msgLen;
    }
  }

  if (bufLen > 0)
  {
    esp_now_send(txBackpackAddress, buf, bufLen);
  }

  uplinkMsgCount = 0;
}

/////////// SETUP ///////////

void setup()
{
  Serial.setRxBufferSize(4096);
  Serial.begin(460800);

  STATS_UART.begin(STATS_UART_BAUD, SERIAL_8N1, -1, STATS_UART_TX_PIN);

  options_init();

  SetCompanionMACAddress();

  if (esp_now_init() != ESP_OK)
  {
    DBGLN("Error initializing ESP-NOW");
    ESP.restart();
  }

  memcpy(txPeerInfo.peer_addr, txBackpackAddress, 6);
  txPeerInfo.channel = 0;
  txPeerInfo.encrypt = false;
  if (esp_now_add_peer(&txPeerInfo) != ESP_OK)
  {
    DBGLN("ESP-NOW failed to add TX backpack peer");
    ESP.restart();
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuousRxCb);

  DBGLN("MAVLink companion started");
  STATS_UART.println("MAVLink companion started");
}

/////////// LOOP ///////////

void loop()
{
  uint32_t now = millis();

  // Drain downlink ring buffer, frame MAVLink, write complete messages to Serial
  {
    mavlink_status_t dlStatus;
    mavlink_message_t dlMsg;
    uint8_t dlBuf[MAVLINK_MAX_PACKET_LEN];
    while (downlinkHead != downlinkTail)
    {
      uint8_t c = downlinkBuf[downlinkTail];
      downlinkTail = (downlinkTail + 1) % sizeof(downlinkBuf);

      if (mavlink_frame_char(MAVLINK_COMM_1, c, &dlMsg, &dlStatus) != MAVLINK_FRAMING_INCOMPLETE)
      {
        uint16_t len = mavlink_msg_to_send_buffer(dlBuf, &dlMsg);
        Serial.write(dlBuf, len);
      }
    }
  }

  // Parse MAVLink from UART, queue complete messages
  {
    mavlink_status_t ulStatus;
    mavlink_message_t ulMsg;
    while (Serial.available())
    {
      uint8_t c = Serial.read();

      if (mavlink_frame_char(MAVLINK_COMM_0, c, &ulMsg, &ulStatus) != MAVLINK_FRAMING_INCOMPLETE)
      {
        if (uplinkMsgCount < MAVLINK_BUF_SIZE)
        {
          uplinkMsgBuf[uplinkMsgCount++] = ulMsg;
        }
      }
    }
  }

  // Flush using same thresholds as Tx_main.cpp
  bool thresholdHit = uplinkMsgCount >= MAVLINK_BUF_THRESHOLD;
  bool timeoutHit   = uplinkMsgCount > 0 && (now - lastUplinkFlush) > MAVLINK_BUF_TIMEOUT;
  if (thresholdHit || timeoutHit)
  {
    flushUplinkBuf();
    lastUplinkFlush = now;
  }

  // Print stats once per second on dedicated UART
  if (now - lastStatsPrint >= 1000)
  {
    int32_t avgRssi = rssiCount > 0 ? rssiSum / (int32_t)rssiCount : 0;
    uint16_t rCnt = rssiCount;
    uint16_t retries = rxRetryCount;
    uint16_t frames = rxFrameCount;
    uint32_t txOk = sendOkCount;
    uint32_t txFail = sendFailCount;

    rssiSum = 0;
    rssiCount = 0;
    rxRetryCount = 0;
    rxFrameCount = 0;
    sendOkCount = 0;
    sendFailCount = 0;

    // sig_mode: 0=11b/g (non-HT), 1=HT(11n); 11b rates: 0=1M,1=2M,2=5.5M,3=11M
    const char *phy = (lastSigMode == 0) ? "11b/g" : "11n";
    STATS_UART.printf("RSSI: %d dBm (%u pkts) | RX retry: %u/%u | TX: %u ok %u fail | PHY: %s rate: %u\r\n",
                      (int)avgRssi, rCnt, retries, frames, txOk, txFail, phy, lastRate);

    lastStatsPrint = now;
  }
}

#endif
