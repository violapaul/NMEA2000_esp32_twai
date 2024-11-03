#ifndef _NMEA2000_ESP32_H_
#define _NMEA2000_ESP32_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "NMEA2000.h"
#include "N2kMsg.h"

#ifndef ESP32_CAN_TX_PIN
#define ESP32_CAN_TX_PIN GPIO_NUM_6
#endif
#ifndef ESP32_CAN_RX_PIN
#define ESP32_CAN_RX_PIN GPIO_NUM_7
#endif
#ifndef ESP32_TWAI_MODE
#define ESP32_TWAI_MODE TWAI_MODE_NORMAL
#endif

typedef enum
{
  CAN_SPEED_100KBPS = 100,  /**< \brief CAN Node runs at 100kBit/s. */
  CAN_SPEED_125KBPS = 125,  /**< \brief CAN Node runs at 125kBit/s. */
  CAN_SPEED_250KBPS = 250,  /**< \brief CAN Node runs at 250kBit/s. */
  CAN_SPEED_500KBPS = 500,  /**< \brief CAN Node runs at 500kBit/s. */
  CAN_SPEED_800KBPS = 800,  /**< \brief CAN Node runs at 800kBit/s. */
  CAN_SPEED_1000KBPS = 1000 /**< \brief CAN Node runs at 1000kBit/s. */
} CAN_speed_t;

class tNMEA2000_esp32 : public tNMEA2000
{
private:
  bool IsOpen;
  static bool CanInUse;

protected:
  CAN_speed_t speed;
  gpio_num_t TxPin;
  gpio_num_t RxPin;
  QueueHandle_t RxQueue;
  QueueHandle_t TxQueue;

public:
  esp_err_t CAN_read_frame(); // Read frame to queue within interrupt

protected:
  void CAN_init();

protected:
  bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent = true);
  bool CANOpen();
  bool CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf);
  virtual void InitCANFrameBuffers();

public:
  tNMEA2000_esp32(gpio_num_t _TxPin = ESP32_CAN_TX_PIN, gpio_num_t _RxPin = ESP32_CAN_RX_PIN);
};

esp_err_t ESP32_CAN_read_frame();

#endif
