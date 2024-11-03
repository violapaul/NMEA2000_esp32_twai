#include "driver/twai.h"
#include "NMEA2000_esp32.h"

bool tNMEA2000_esp32::CanInUse = false;
tNMEA2000_esp32 *pNMEA2000_esp32 = 0;

//*****************************************************************************
tNMEA2000_esp32::tNMEA2000_esp32(gpio_num_t _TxPin, gpio_num_t _RxPin) : tNMEA2000(), IsOpen(false),
                                                                         speed(CAN_SPEED_250KBPS), TxPin(_TxPin), RxPin(_RxPin),
                                                                         RxQueue(NULL), TxQueue(NULL)
{
}

//*****************************************************************************

bool tNMEA2000_esp32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool /*wait_sent*/)
{
  twai_message_t message;

  message.identifier = id;
  message.data_length_code = len > 8 ? 8 : len;
  message.extd = 1;
  message.ss = 0;

  uint8_t frame_buf[8];
  memcpy(frame_buf, buf, len);

  for (size_t i = 0; i < len; i++)
  {
    message.data[i] = frame_buf[i];
  }

  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(1000));
  // Queue message for transmission
  if (result == ESP_OK)
  {
    // printf("Message queued for transmission\n");
    return true;
  }
  else
  {
    Serial.printf("Failed to queue message for transmission - %s \n", esp_err_to_name(result));
    return false;
  }
}

//*****************************************************************************
void tNMEA2000_esp32::InitCANFrameBuffers()
{
  if (MaxCANReceiveFrames < 10)
    MaxCANReceiveFrames = 50; // ESP32 has plenty of RAM
  if (MaxCANSendFrames < 10)
    MaxCANSendFrames = 40;
  uint16_t CANGlobalBufSize = MaxCANSendFrames - 4;
  MaxCANSendFrames = 4; // we do not need much libary internal buffer since driver has them.
  RxQueue = xQueueCreate(MaxCANReceiveFrames, sizeof(twai_message_t));
  TxQueue = xQueueCreate(CANGlobalBufSize, sizeof(twai_message_t));

  tNMEA2000::InitCANFrameBuffers(); // call main initialization
}

//*****************************************************************************
bool tNMEA2000_esp32::CANOpen()
{
  if (IsOpen)
    return true;

  if (CanInUse)
    return false; // currently prevent accidental second instance. Maybe possible in future.

  pNMEA2000_esp32 = this;
  IsOpen = true;
  CAN_init();

  CanInUse = IsOpen;

  return IsOpen;
}

//*****************************************************************************
bool tNMEA2000_esp32::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf)
{
  bool HasFrame = false;
  twai_message_t message;

  // receive next CAN frame from queue
  if (xQueueReceive(RxQueue, &message, 0) == pdTRUE)
  {
    HasFrame = true;
    id = message.identifier;
    len = message.data_length_code;
    memcpy(buf, message.data, message.data_length_code);
  }

  return HasFrame;
}

esp_err_t tNMEA2000_esp32::CAN_read_frame()
{
  // Wait for message to be received
  twai_message_t message;
  esp_err_t result = twai_receive(&message, portMAX_DELAY);
  if (result == ESP_OK)
  {
    // send frame to input queue
    xQueueSendToBackFromISR(RxQueue, &message, 0);
  }
  else
  {
    Serial.printf("Failed to receive message - %s \n", esp_err_to_name(result));
  }
  return result;
}

//*****************************************************************************
void tNMEA2000_esp32::CAN_init()
{
  // Fixed a bug which used the preprocessor defined pins instead of the class instance variables.
  // The problem is that this was determined at compile time, which was before the user code was
  // built.  So there was not way to configure the pins to anything other than the preprocessor defaults.
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(this->TxPin, this->RxPin, TWAI_MODE_NO_ACK);
  g_config.tx_queue_len = 20;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));

  // Start TWAI driver
  ESP_ERROR_CHECK(twai_start());

  Serial.println("TWAI Driver started");

  // TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF
  ESP_ERROR_CHECK(twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL));
}


esp_err_t ESP32_CAN_read_frame()
{
  return pNMEA2000_esp32->CAN_read_frame();
}
