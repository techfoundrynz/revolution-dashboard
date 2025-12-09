// Compatible with VESC FW3.49 //also tested with 6.02

#include "VescComms.h"
#include <HardwareSerial.h>

VescComms::VescComms(void) {
  nunchuck.valueX = 127;
  nunchuck.valueY = 127;
  nunchuck.lowerButton = false;
  nunchuck.upperButton = false;
}

void VescComms::setSerialPort(HardwareSerial *port) {
  serialPort = port;
}

void VescComms::setDebugPort(Stream *port) {
  debugPort = port;
}

// Variables for CAN reassembly
static uint8_t canRxBuffer[256];
static uint8_t canRxIndex = 0;
static bool canRxComplete = false;
static unsigned long canRxTimeout = 0;

void VescComms::beginCAN(int txPin, int rxPin, uint8_t controllerId, uint8_t ownId) {
  _useCAN = true;
  _canId = controllerId;
  _ownId = ownId;

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)txPin, (gpio_num_t)rxPin, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config;
  #if CAN_BAUD_RATE == 250000
    t_config = TWAI_TIMING_CONFIG_250KBITS();
  #elif CAN_BAUD_RATE == 500000
    t_config = TWAI_TIMING_CONFIG_500KBITS();
  #elif CAN_BAUD_RATE == 1000000
    t_config = TWAI_TIMING_CONFIG_1MBITS();
  #else
    t_config = TWAI_TIMING_CONFIG_250KBITS(); // Default
  #endif
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
  }
  else {
    if (debugPort)
      debugPort->println("CAN Init Failed");
  }
}

void VescComms::comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
  twai_message_t message;
  message.identifier = id;
  message.extd = 1;
  message.data_length_code = len;
  memcpy(message.data, data, len);
  twai_transmit(&message, pdMS_TO_TICKS(10));
}

int VescComms::sendCanPayload(uint8_t *payload, int len) {
  if (len <= 6) {
    // Send Short Buffer
    uint32_t id = (CAN_PACKET_PROCESS_SHORT_BUFFER << 8) | _canId;
    uint8_t data[8];
    data[0] = _ownId;
    data[1] = 0; // Reserved
    memcpy(&data[2], payload, len);
    comm_can_transmit_eid(id, data, len + 2);
  }
  else {
    // Send Long Buffer via Fill/Process RX Buffer
    // 1. Fill Buffer
    uint8_t end_a = 0;
    for (int i = 0; i < len; i += 4) { // Send 4 bytes at a time (index + 4 bytes = 5, or more efficient?)
      // VESC implementation often uses index in data[0], and up to 7 bytes?
      // "data[0] = message_index"
      uint8_t chunkLen = len - i;
      if (chunkLen > 7)
        chunkLen = 7;

      uint32_t id = (CAN_PACKET_FILL_RX_BUFFER << 8) | _canId;
      uint8_t data[8];
      data[0] =
          i; // Index (Byte offset? No, "block index" or byte index? VESC commands.c: buffer[data[0]] = data[1]... wait.
             // VESC implementation: commands_process_packet:
             // comm_can_fill_rx_buffer(uint8_t controller_id, uint8_t *data, uint8_t len)
             // buffer_offset = data[0];
             // memcpy(rx_buffer + buffer_offset, data + 1, len - 1);

      data[0] = i;
      memcpy(&data[1], &payload[i], chunkLen);
      comm_can_transmit_eid(id, data, chunkLen + 1);
      delay(1); // Small delay to prevent flooding if needed
    }

    // 2. Process Buffer
    // Payload: [SenderID, Command(0=Process), Len>>8, Len&0xFF, CRC>>8, CRC&0xFF]
    uint16_t crcVal = crc16(payload, len);
    uint8_t data[6];
    data[0] = _ownId;
    data[1] = 0; // 0 = Process
    data[2] = len >> 8;
    data[3] = len & 0xFF;
    data[4] = crcVal >> 8;
    data[5] = crcVal & 0xFF;

    uint32_t id = (CAN_PACKET_PROCESS_RX_BUFFER << 8) | _canId;
    comm_can_transmit_eid(id, data, 6);
  }
  return len;
}

int VescComms::receiveCanMessage(uint8_t *payloadReceived) {
  twai_message_t message;
  while (twai_receive(&message, 0) == ESP_OK) { // Non-blocking check
    if (!message.extd)
      continue;

    uint8_t id = message.identifier & 0xFF;
    CAN_PACKET_ID cmd = (CAN_PACKET_ID)(message.identifier >> 8);

    if (id != _ownId)
      continue; // Not for us

    if (cmd == CAN_PACKET_PROCESS_SHORT_BUFFER) {
      // data[0] = sender, data[1] = 0, rest is payload
      if (message.data_length_code > 2) {
        int len = message.data_length_code - 2;
        memcpy(payloadReceived, &message.data[2], len);
        return len;
      }
    }
    else if (cmd == CAN_PACKET_FILL_RX_BUFFER) {
      int offset = message.data[0];
      int len = message.data_length_code - 1;
      if (offset + len <= 256) {
        memcpy(&canRxBuffer[offset], &message.data[1], len);
        canRxTimeout = millis() + 500;
      }
    }
    else if (cmd == CAN_PACKET_PROCESS_RX_BUFFER) {
      // data[0] = sender, data[1] = command, data[2/3] = len, data[4/5] = crc
      if (message.data_length_code >= 6) {
        int len = (message.data[2] << 8) | message.data[3];
        uint16_t crcRx = (message.data[4] << 8) | message.data[5];
        if (len <= 256) {
          uint16_t crcCalc = crc16(canRxBuffer, len);
          if (crcCalc == crcRx) {
            memcpy(payloadReceived, canRxBuffer, len);
            return len;
          }
        }
      }
    }
  }
  return 0;
}

int VescComms::receiveUartMessage(uint8_t *payloadReceived) {
  if (_useCAN) {
    return receiveCanMessage(payloadReceived);
  }

  // Messages <= 255 starts with "2", 2nd byte is length
  // Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

  uint16_t counter = 0;
  uint16_t endMessage = 256;
  bool messageRead = false;
  uint8_t messageReceived[256];
  uint16_t lenPayload = 0;

  uint32_t timeout = millis() + 100; // Defining the timestamp for timeout (100ms before timeout)

  while (millis() < timeout && messageRead == false) {

    while (serialPort->available()) {

      messageReceived[counter++] = serialPort->read();

      if (counter == 2) {

        switch (messageReceived[0]) {
        case 2:
          endMessage = messageReceived[1] + 5; // Payload size + 2 for sice + 3 for SRC and End.
          lenPayload = messageReceived[1];
          break;

        case 3:
          // ToDo: Add Message Handling > 255 (starting with 3)
          if (debugPort != NULL) {
            debugPort->println("Message is larger than 256 bytes - not supported");
          }
          break;

        default:
          if (debugPort != NULL) {
            debugPort->println("Unvalid start bit");
          }
          break;
        }
      }

      if (counter >= sizeof(messageReceived)) {
        break;
      }

      if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
        messageReceived[endMessage] = 0;
        if (debugPort != NULL) {
          debugPort->println("End of message reached!");
        }
        messageRead = true;
        break; // Exit if end of message is reached, even if there is still more data in the buffer.
      }
    }
  }
  if (messageRead == false && debugPort != NULL) {
    debugPort->println("Timeout");
  }

  bool unpacked = false;

  if (messageRead) {
    unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
  }

  if (unpacked) {
    // Message was read
    return lenPayload;
  }
  else {
    // No Message Read
    return 0;
  }
}

bool VescComms::unpackPayload(uint8_t *message, int lenMes, uint8_t *payload) {

  uint16_t crcMessage = 0;
  uint16_t crcPayload = 0;

  // Rebuild crc:
  crcMessage = message[lenMes - 3] << 8;
  crcMessage &= 0xFF00;
  crcMessage += message[lenMes - 2];

  if (debugPort != NULL) {
    debugPort->print("SRC received: ");
    debugPort->println(crcMessage);
  }

  // Extract payload:
  memcpy(payload, &message[2], message[1]);

  crcPayload = crc16(payload, message[1]);

  if (debugPort != NULL) {
    debugPort->print("SRC calc: ");
    debugPort->println(crcPayload);
  }

  if (crcPayload == crcMessage) {
    if (debugPort != NULL) {
      debugPort->print("Received: ");
      serialPrint(message, lenMes);
      debugPort->println();

      debugPort->print("Payload :      ");
      serialPrint(payload, message[1] - 1);
      debugPort->println();
    }

    return true;
  }
  else {
    return false;
  }
}

int VescComms::packSendPayload(uint8_t *payload, int lenPay) {
  if (_useCAN) {
    return sendCanPayload(payload, lenPay);
  }

  uint16_t crcPayload = crc16(payload, lenPay);
  int count = 0;
  uint8_t messageSend[256];

  if (lenPay <= 256) {
    messageSend[count++] = 2;
    messageSend[count++] = lenPay;
  }
  else {
    messageSend[count++] = 3;
    messageSend[count++] = (uint8_t)(lenPay >> 8);
    messageSend[count++] = (uint8_t)(lenPay & 0xFF);
  }

  memcpy(&messageSend[count], payload, lenPay);

  count += lenPay;
  messageSend[count++] = (uint8_t)(crcPayload >> 8);
  messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
  messageSend[count++] = 3;
  messageSend[count] = '\0';

  if (debugPort != NULL) {
    debugPort->print("UART package send: ");
    serialPrint(messageSend, count);
  }

  // Sending package
  serialPort->write(messageSend, count);

  // Returns number of send bytes
  return count;
}

bool VescComms::processReadPacket(bool deviceType, uint8_t *message) {

  COMM_PACKET_ID packetId;
  COMM_PACKET_ID_DIEBIEMS packetIdDieBieMS;

  int32_t ind = 0;

  if (!deviceType) { // device if VESC type
    packetId = (COMM_PACKET_ID)message[0];
    message++; // Removes the packetId from the actual message (payload)

    switch (packetId) {
    case COMM_FW_VERSION: // Structure defined here:
                          // https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

      fw_version.major = message[ind++];
      fw_version.minor = message[ind++];
      return true;

    case COMM_GET_VALUES:
    case COMM_GET_VALUES_SELECTIVE: { // Structure defined here:
                                      // https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164
      uint32_t mask = 0xFFFFFFFF;

      if (packetId == COMM_GET_VALUES_SELECTIVE) {
        mask = buffer_get_uint32(message, &ind);
      }

      if (mask & ((uint32_t)1 << 0)) {
        data.tempFET = buffer_get_float16(message, 10.0, &ind);
      }
      if (mask & ((uint32_t)1 << 1)) {
        data.tempMotor = buffer_get_float16(message, 10.0, &ind);
      }
      if (mask & ((uint32_t)1 << 2)) {
        data.avgMotorCurrent = buffer_get_float32(message, 100.0, &ind);
      }
      if (mask & ((uint32_t)1 << 3)) {
        data.avgInputCurrent = buffer_get_float32(message, 100.0, &ind);
      }
      if (mask & ((uint32_t)1 << 4)) {
        data.avgIdCurent = buffer_get_float32(message, 100.0, &ind);
      }
      if (mask & ((uint32_t)1 << 5)) {
        data.avgIqCurent = buffer_get_float32(message, 100.0, &ind);
      }
      if (mask & ((uint32_t)1 << 6)) {
        data.dutyCycleNow = buffer_get_float16(message, 1000.0, &ind);
      }
      if (mask & ((uint32_t)1 << 7)) {
        data.rpm = buffer_get_int32(message, &ind);
      }
      if (mask & ((uint32_t)1 << 8)) {
        data.inpVoltage = buffer_get_float16(message, 10.0, &ind);
      }
      if (mask & ((uint32_t)1 << 9)) {
        data.ampHours = buffer_get_float32(message, 10000.0, &ind);
      }
      if (mask & ((uint32_t)1 << 10)) {
        data.ampHoursCharged = buffer_get_float32(message, 10000.0, &ind);
      }
      if (mask & ((uint32_t)1 << 11)) {
        data.watt_hours = buffer_get_float32(message, 10000.0, &ind);
      }
      if (mask & ((uint32_t)1 << 12)) {
        data.watt_hours_charged = buffer_get_float32(message, 10000.0, &ind);
      }
      if (mask & ((uint32_t)1 << 13)) {
        data.tachometer = buffer_get_int32(message, &ind);
      }
      if (mask & ((uint32_t)1 << 14)) {
        data.tachometerAbs = buffer_get_int32(message, &ind);
      }
      if (mask & ((uint32_t)1 << 15)) {
        data.fault = message[ind];
      }
      // Others values are ignored. You can add them here accordingly to commands.c in VESC Firmware. Please add those
      // variables in "struct dataPackage" in VescUart.h file.

      return true;
    }

    case COMM_GET_VALUES_SETUP_SELECTIVE: { // Structure defined here:
                                            // https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164
      uint32_t mask = 0;
      mask += ind++ << 24;
      mask += ind++ << 16;
      mask += ind++ << 8;
      mask += ind++;

      if (mask & ((uint32_t)1 << 0)) {
        data.tempFET = buffer_get_float16(message, 10.0, &ind);
      }
      if (mask & ((uint32_t)1 << 1)) {
        data.tempMotor = buffer_get_float16(message, 10.0, &ind);
      }
      if (mask & ((uint32_t)1 << 2)) {
        data.avgMotorCurrent = buffer_get_float32(message, 100.0, &ind);
      }
      if (mask & ((uint32_t)1 << 3)) {
        data.avgInputCurrent = buffer_get_float32(message, 100.0, &ind);
      }
      if (mask & ((uint32_t)1 << 4)) {
        data.dutyCycleNow = buffer_get_float16(message, 1000.0, &ind);
      }
      if (mask & ((uint32_t)1 << 5)) {
        data.rpm = buffer_get_int32(message, &ind);
      }
      if (mask & ((uint32_t)1 << 6)) { /* speed */
      };
      if (mask & ((uint32_t)1 << 7)) {
        data.inpVoltage = buffer_get_float16(message, 10.0, &ind);
      }
      if (mask & ((uint32_t)1 << 8)) { /* batt level */
      }
      if (mask & ((uint32_t)1 << 9)) {
        data.ampHours = buffer_get_float32(message, 10000.0, &ind);
      }
      if (mask & ((uint32_t)1 << 10)) {
        data.ampHoursCharged = buffer_get_float32(message, 10000.0, &ind);
      }
      if (mask & ((uint32_t)1 << 11)) {
        data.watt_hours = buffer_get_float32(message, 10000.0, &ind);
      }
      if (mask & ((uint32_t)1 << 12)) {
        data.watt_hours_charged = buffer_get_float32(message, 10000.0, &ind);
      }
      if (mask & ((uint32_t)1 << 13)) { /* distance */
      }
      if (mask & ((uint32_t)1 << 14)) { /* distance absolute */
      }
      if (mask & ((uint32_t)1 << 15)) { /* PID pos */
      }
      if (mask & ((uint32_t)1 << 16)) {
        data.fault = message[ind];
      }
      // Others values are ignored. You can add them here accordingly to commands.c in VESC Firmware. Please add those
      // variables in "struct dataPackage" in VescUart.h file.

      return true;
    }

    case COMM_GET_DECODED_PPM:

      data.throttle = (float)(buffer_get_int32(message, &ind) / 10000.0);
      // data.rawValuePPM 	= buffer_get_float32(message, 100.0, &ind);
      return true;
      break;

    case COMM_GET_DECODED_CHUK:

      data.throttle = (float)(buffer_get_int32(message, &ind) / 10000.0);

      return true;
      break;

    default:
      return false;
      break;
    }
  }
  else { // device is DieBieMS
    packetIdDieBieMS = (COMM_PACKET_ID_DIEBIEMS)message[0];
    message++; // Removes the packetId from the actual message (payload)

    switch (packetIdDieBieMS) {

    case DBMS_COMM_GET_VALUES: // Structure defined here:
                               // https://github.com/DieBieEngineering/DieBieMS-Firmware/blob/master/Modules/Src/modCommands.c

      ind = 45;
      // DieBieMSdata.packVoltage = buffer_get_float32(message, 1000.0, &ind);
      // DieBieMSdata.packCurrent = buffer_get_float32(message, 1000.0, &ind);
      // DieBieMSdata.cellVoltageHigh = buffer_get_float32(message, 1000.0, &ind);
      // DieBieMSdata.cellVoltageAverage = buffer_get_float32(message, 1000.0, &ind);
      // DieBieMSdata.cellVoltageLow = buffer_get_float32(message, 1000.0, &ind);
      // DieBieMSdata.cellVoltageMisMatch = buffer_get_float32(message, 1000.0, &ind);
      // DieBieMSdata.loCurrentLoadVoltage = buffer_get_float16(message, 100.0, &ind);
      // DieBieMSdata.loCurrentLoadCurrent = buffer_get_float16(message, 100.0, &ind);
      // DieBieMSdata.hiCurrentLoadVoltage = buffer_get_float16(message, 100.0, &ind);
      // DieBieMSdata.hiCurrentLoadCurrent = buffer_get_float16(message, 100.0, &ind);
      // DieBieMSdata.auxVoltage = buffer_get_float16(message, 100.0, &ind);
      // DieBieMSdata.auxCurrent = buffer_get_float16(message, 100.0, &ind);
      // DieBieMSdata.tempBatteryHigh = buffer_get_float16(message, 10.0, &ind);
      // DieBieMSdata.tempBatteryAverage = buffer_get_float16(message, 10.0, &ind);
      // DieBieMSdata.tempBMSHigh = buffer_get_float16(message, 10.0, &ind);
      // DieBieMSdata.tempBMSAverage = buffer_get_float16(message, 10.0, &ind);
      DieBieMSdata.operationalState = message[ind++];
      // DieBieMSdata.chargeBalanceActive = message[ind++];
      // DieBieMSdata.faultState = message[ind++];

      return true;
      break;

    case DBMS_COMM_GET_BMS_CELLS: // Structure defined here:
                                  // https://github.com/DieBieEngineering/DieBieMS-Firmware/blob/master/Modules/Src/modCommands.c

      DieBieMScells.noOfCells = message[ind++];

      for (uint8_t i = 0; i < 12; i++) {
        DieBieMScells.cellsVoltage[i] = buffer_get_float16(message, 1000.0, &ind);
      }

      return true;
      break;

    default:
      return false;
      break;
    }
  }
}

bool VescComms::getVescValues(void) {
  uint8_t command[1];
  command[0] = {COMM_GET_VALUES};
  uint8_t payload[256];

  packSendPayload(command, 1);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 0) {                            // && lenPayload < 55) {
    bool read = processReadPacket(false, payload); // returns true if sucessful
    return read;
  }
  else {
    return false;
  }
}

bool VescComms::getVescValuesSelective(uint32_t mask) {
  uint8_t command[5];
  command[0] = {COMM_GET_VALUES_SELECTIVE};
  command[1] = {mask >> 24};        // mask MSB
  command[2] = {mask >> 16 & 0xFF}; // mask
  command[3] = {mask >> 8 & 0xFF};  // mask
  command[4] = {mask & 0xFF};       // mask LSB
  uint8_t payload[256];

  packSendPayload(command, 5);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 0 && lenPayload < 55) {
    bool read = processReadPacket(false, payload); // returns true if sucessful
    return read;
  }
  else {
    return false;
  }
}

bool VescComms::getVescValuesSetupSelective(uint32_t mask) {
  uint8_t command[5];
  command[0] = {COMM_GET_VALUES_SETUP_SELECTIVE};
  command[1] = {mask >> 24};        // mask MSB
  command[2] = {mask >> 16 & 0xFF}; // mask
  command[3] = {mask >> 8 & 0xFF};  // mask
  command[4] = {mask & 0xFF};       // mask LSB
  uint8_t payload[256];

  packSendPayload(command, 5);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 0 && lenPayload < 55) {
    bool read = processReadPacket(false, payload); // returns true if sucessful
    return read;
  }
  else {
    return false;
  }
}

bool VescComms::getLocalVescPPM(void) {

  uint8_t command[1] = {COMM_GET_DECODED_PPM};
  uint8_t payload[256];

  packSendPayload(command, 1);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 0) {                            //&& lenPayload < 55
    bool read = processReadPacket(false, payload); // returns true if sucessful
    return read;
  }
  else {
    return false;
  }
}

bool VescComms::getMasterVescPPM(uint8_t id) {

  uint8_t command[3];
  command[0] = {COMM_FORWARD_CAN};
  command[1] = id;
  command[2] = {COMM_GET_DECODED_PPM};

  uint8_t payload[256];

  packSendPayload(command, 3);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 0) {                            //&& lenPayload < 55
    bool read = processReadPacket(false, payload); // returns true if sucessful
    return read;
  }
  else {
    return false;
  }
}

bool VescComms::getLocalVescNun(void) {
  uint8_t command[1] = {COMM_GET_DECODED_CHUK};
  uint8_t payload[256];

  packSendPayload(command, 1);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 0) {                            //&& lenPayload < 55
    bool read = processReadPacket(false, payload); // returns true if sucessful
    return read;
  }
  else {
    return false;
  }
}

bool VescComms::getMasterVescNun(uint8_t id) {
  uint8_t command[3];
  command[0] = {COMM_FORWARD_CAN};
  command[1] = id;
  command[2] = {COMM_GET_DECODED_CHUK};

  uint8_t payload[256];

  packSendPayload(command, 3);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 0) {                            //&& lenPayload < 55
    bool read = processReadPacket(false, payload); // returns true if sucessful
    return read;
  }
  else {
    return false;
  }
}

bool VescComms::getFWversion(void) {

  uint8_t command[1] = {COMM_FW_VERSION};
  uint8_t payload[256];

  packSendPayload(command, 1);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 0) {                            //&& lenPayload < 55
    bool read = processReadPacket(false, payload); // returns true if sucessful
    return read;
  }
  else {
    return false;
  }
}

bool VescComms::getDieBieMSValues(uint8_t id) {
  uint8_t command[3];
  command[0] = {COMM_FORWARD_CAN}; // VESC command
  command[1] = id;
  command[2] = {DBMS_COMM_GET_VALUES}; // DieBieMS command

  uint8_t payload[256];

  packSendPayload(command, 3);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 0) {                           //&& lenPayload < 55
    bool read = processReadPacket(true, payload); // returns true if sucessful
    return read;
  }
  else {
    return false;
  }
}

bool VescComms::getDieBieMSCellsVoltage(uint8_t id) {
  uint8_t command[3];
  command[0] = {COMM_FORWARD_CAN}; // VESC command
  command[1] = id;
  command[2] = {DBMS_COMM_GET_BMS_CELLS}; // DieBieMS command

  uint8_t payload[256];

  packSendPayload(command, 3);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 0) {                           //&& lenPayload < 55
    bool read = processReadPacket(true, payload); // returns true if sucessful
    return read;
  }
  else {
    return false;
  }
}

void VescComms::setNunchuckValues() {
  int32_t ind = 0;
  uint8_t payload[11];

  payload[ind++] = COMM_SET_CHUCK_DATA;
  payload[ind++] = nunchuck.valueX;
  payload[ind++] = nunchuck.valueY;
  buffer_append_bool(payload, nunchuck.lowerButton, &ind);
  buffer_append_bool(payload, nunchuck.upperButton, &ind);

  // Acceleration Data. Not used, Int16 (2 byte)
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;
  payload[ind++] = 0;

  if (debugPort != NULL) {
    debugPort->println("Data reached at setNunchuckValues:");
    debugPort->print("valueX = ");
    debugPort->print(nunchuck.valueX);
    debugPort->print(" valueY = ");
    debugPort->println(nunchuck.valueY);
    debugPort->print("LowerButton = ");
    debugPort->print(nunchuck.lowerButton);
    debugPort->print(" UpperButton = ");
    debugPort->println(nunchuck.upperButton);
  }

  packSendPayload(payload, 11);
}

void VescComms::setCurrent(float current) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_CURRENT;
  buffer_append_int32(payload, (int32_t)(current * 1000), &index);

  packSendPayload(payload, 5);
}

void VescComms::setBrakeCurrent(float brakeCurrent) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_CURRENT_BRAKE;
  buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

  packSendPayload(payload, 5);
}

void VescComms::setRPM(float rpm) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_RPM;
  buffer_append_int32(payload, (int32_t)(rpm), &index);

  packSendPayload(payload, 5);
}

void VescComms::setDuty(float duty) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_DUTY;
  buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

  packSendPayload(payload, 5);
}

void VescComms::sendKeepAlive(void) {
    if (_useCAN) {
        uint8_t payload[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        comm_can_transmit_eid(0x0B57ED1F, payload, 8);
    }
}

void VescComms::setLocalProfile(bool store, bool forward_can, bool divide_by_controllers, float current_min_rel,
                                float current_max_rel, float speed_max_reverse, float speed_max, float duty_min,
                                float duty_max, float watt_min, float watt_max) {
  int32_t index = 0;
  uint8_t payload[38];

  bool ack = false;

  payload[index++] = COMM_SET_MCCONF_TEMP_SETUP; // set new profile with speed limitation in m/s
  payload[index++] = store ? 1 : 0;
  payload[index++] = forward_can ? 1 : 0;
  payload[index++] = ack ? 1 : 0;
  payload[index++] = divide_by_controllers ? 1 : 0;

  buffer_append_float32_auto(payload, current_min_rel, &index);
  buffer_append_float32_auto(payload, current_max_rel, &index);
  buffer_append_float32_auto(payload, speed_max_reverse, &index);
  buffer_append_float32_auto(payload, speed_max, &index);
  buffer_append_float32_auto(payload, duty_min, &index);
  buffer_append_float32_auto(payload, duty_max, &index);
  buffer_append_float32_auto(payload, watt_min, &index);
  buffer_append_float32_auto(payload, watt_max, &index);

  packSendPayload(payload, 38);
  if (debugPort != NULL) {
    debugPort->print("setLocalProfile package send: ");
    serialPrint(payload, 38);
  }
}

void VescComms::serialPrint(uint8_t *data, int len) {
  if (debugPort != NULL) {
    for (int i = 0; i <= len; i++) {
      debugPort->print(data[i]);
      debugPort->print(" ");
    }

    debugPort->println("");
  }
}

void VescComms::printVescValues() {
  if (debugPort != NULL) {
    debugPort->print("avgMotorCurrent: ");
    debugPort->println(data.avgMotorCurrent);
    debugPort->print("avgInputCurrent: ");
    debugPort->println(data.avgInputCurrent);
    debugPort->print("dutyCycleNow: ");
    debugPort->println(data.dutyCycleNow);
    debugPort->print("rpm: ");
    debugPort->println(data.rpm);
    debugPort->print("inputVoltage: ");
    debugPort->println(data.inpVoltage);
    debugPort->print("ampHours: ");
    debugPort->println(data.ampHours);
    debugPort->print("ampHoursCharges: ");
    debugPort->println(data.ampHoursCharged);
    debugPort->print("tachometer: ");
    debugPort->println(data.tachometer);
    debugPort->print("tachometerAbs: ");
    debugPort->println(data.tachometerAbs);
  }
}
