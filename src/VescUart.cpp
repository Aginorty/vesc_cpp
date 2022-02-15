#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

#include "VescUart.h"
#include "spdlog/spdlog.h"
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <chrono>
#include <cstring>

VescUart::VescUart() {
  spdlog::set_level(spdlog::level::debug);
  //  char *portname = "/dev/ttyS0";
  fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
  struct termios toptions;
  tcgetattr(fd, &toptions);

  cfsetispeed(&toptions, B115200);
  cfsetospeed(&toptions, B115200);
  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;

  toptions.c_cflag &= ~CRTSCTS;
  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;
  /* disable input/output flow control, disable restart chars */
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* disable canonical input, disable echo,
  disable visually erase chars,
  disable terminal-generated signals */
  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  /* disable output processing */
  toptions.c_oflag &= ~OPOST;

  /* wait for 12 characters to come in before read returns */
  /* WARNING! THIS CAUSES THE read() TO BLOCK UNTIL ALL */
  /* CHARACTERS HAVE COME IN! */
  //toptions.c_cc[VMIN] = 100;
  //  /* no minimum time to wait before read returns */
  //toptions.c_cc[VTIME] = 1;

  /* commit the options */
  tcsetattr(fd, TCSANOW, &toptions);

  /* Flush anything already in the serial buffer */
  tcflush(fd, TCIFLUSH);
  //  /* read up to 128 bytes from the fd */
  //  int n = read(fd, buf, 128);
}

int VescUart::receiveUartMessage(uint8_t *payloadReceived) {

  // Messages <= 255 starts with "2", 2nd byte is length
  // Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st
  // >>8 and then &0xFF
  SPDLOG_DEBUG("receiving message");
  tcflush(fd, TCIFLUSH);
  uint16_t counter = 0;
  uint16_t endMessage = 256;
  bool messageRead = false;
  bool message_read_error = false;
  bool first_read = true;
  uint8_t messageReceived[256];
  uint16_t lenPayload = 0;
  std::chrono::high_resolution_clock::time_point time_now_ms =
      std::chrono::high_resolution_clock::now();
  auto timeout =
      time_now_ms +
      std::chrono::milliseconds(
          100); // Defining the timestamp for timeout (100ms before timeout)
  size_t received_bytes_total = 0;
  size_t buffer_length = 100;
  while (std::chrono::high_resolution_clock::now() < timeout && !messageRead &&
         !message_read_error) {

    while (!message_read_error && !messageRead) {
      uint8_t buffer[buffer_length];
      size_t read_num = read(fd, messageReceived + received_bytes_total,
                             buffer_length - received_bytes_total);
      
      received_bytes_total += read_num;

      if (first_read) {
        first_read = false;
        switch (messageReceived[0]) {
        case 2:
          endMessage = messageReceived[1] +
                       5; // Payload size + 2 for sice + 3 for SRC and End.
          lenPayload = messageReceived[1];
          break;

        case 3:
          // ToDo: Add Message Handling > 255 (starting with 3)
          SPDLOG_DEBUG("Message is larger than 256 bytes - not supported");
          break;

        default:
          SPDLOG_DEBUG("Unvalid start bit");
          SPDLOG_DEBUG("first char received is : {}", buffer[0]);
          message_read_error = true;
          break;
        }
      }
      
      if (received_bytes_total >= endMessage + 1) {
        message_read_error = true;
        break;
      }

      if (received_bytes_total == endMessage && messageReceived[endMessage - 1] == 3) {
        messageReceived[endMessage] = 0;
        messageRead = true;

        break; // Exit if end of message is reached, even if there is still more
               // data in the buffer.
      }
    }
  }
  if (!messageRead) {
    SPDLOG_DEBUG("Timeout");
  }

  bool unpacked = false;

  if (messageRead) {
    unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
  }

  if (unpacked) {
    // Message was read
    return lenPayload;
  } else {
    // No Message Read
    return 0;
  }
}

bool VescUart::unpackPayload(uint8_t *message, int lenMes, uint8_t *payload) {

  uint16_t crcMessage = 0;
  uint16_t crcPayload = 0;

  // Rebuild crc:
  crcMessage = message[lenMes - 3] << 8;
  crcMessage &= 0xFF00;
  crcMessage += message[lenMes - 2];

  //SPDLOG_DEBUG("SRC received: ");
  //SPDLOG_DEBUG(crcMessage);

  // Extract payload:
  memcpy(payload, &message[2], message[1]);

  crcPayload = crc16(payload, message[1]);

  //SPDLOG_DEBUG("SRC calc: ");
  //SPDLOG_DEBUG(crcPayload);

  if (crcPayload == crcMessage) {
    /*SPDLOG_DEBUG("Received: ");
    serialPrint(message, lenMes);
    SPDLOG_DEBUG("");
    SPDLOG_DEBUG("Payload :      ");
    serialPrint(payload, message[1] - 1);
    */

    return true;
  } else {
    return false;
  }
}

int VescUart::packSendPayload(uint8_t *payload, int lenPay) {

  uint16_t crcPayload = crc16(payload, lenPay);
  int count = 0;
  uint8_t messageSend[256];

  if (lenPay <= 256) {
    messageSend[count++] = 2;
    messageSend[count++] = lenPay;
  } else {
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

  // SPDLOG_DEBUG("UART package send: ");
  // serialPrint(messageSend, count);

  // Sending package
  int n = write(fd, messageSend, count);
  // Returns number of send bytes
  return count;
}

bool VescUart::processReadPacket(uint8_t *message) {

  COMM_PACKET_ID packetId;
  int32_t ind = 0;

  packetId = (COMM_PACKET_ID)message[0];
  message++; // Removes the packetId from the actual message (payload)

  switch (packetId) {
  case COMM_GET_VALUES: // Structure defined here:
                        // https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

    data.tempFET = buffer_get_float16(message, 10.0, &ind);
    data.tempMotor = buffer_get_float16(message, 10.0, &ind);
    data.avgMotorCurrent = buffer_get_float32(message, 100.0, &ind);
    data.avgInputCurrent = buffer_get_float32(message, 100.0, &ind);
    ind += 8; // Skip the next 8 bytes
    data.dutyCycleNow = buffer_get_float16(message, 1000.0, &ind);
    data.rpm = buffer_get_int32(message, &ind);
    data.inpVoltage = buffer_get_float16(message, 10.0, &ind);
    data.ampHours = buffer_get_float32(message, 10000.0, &ind);
    data.ampHoursCharged = buffer_get_float32(message, 10000.0, &ind);
    ind += 8; // Skip the next 8 bytes
    data.tachometer = buffer_get_int32(message, &ind);
    data.tachometerAbs = buffer_get_int32(message, &ind);
    return true;

    break;

  default:
    return false;
    break;
  }
}

bool VescUart::getVescValues() {

  uint8_t command[1] = {COMM_GET_VALUES};
  uint8_t payload[256];

  packSendPayload(command, 1);
  // delay(1); //needed, otherwise data is not read

  int lenPayload = receiveUartMessage(payload);

  if (lenPayload > 55) {
    bool read = processReadPacket(payload); // returns true if successful
    return read;
  } else {
    return false;
  }
}

void VescUart::setCurrent(float current) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_CURRENT;
  buffer_append_int32(payload, (int32_t)(current * 1000), &index);

  packSendPayload(payload, 5);
}

void VescUart::setBrakeCurrent(float brakeCurrent) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_CURRENT_BRAKE;
  buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

  packSendPayload(payload, 5);
}

void VescUart::setRPM(float rpm) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_RPM;
  buffer_append_int32(payload, (int32_t)(rpm), &index);

  packSendPayload(payload, 5);
}

void VescUart::setDuty(float duty) {
  int32_t index = 0;
  uint8_t payload[5];

  payload[index++] = COMM_SET_DUTY;
  buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

  packSendPayload(payload, 5);
}

void VescUart::serialPrint(uint8_t *data, int len) {
  for (int i = 0; i <= len; i++) {
    SPDLOG_DEBUG(data[i]);
    SPDLOG_DEBUG(" ");
  }

  SPDLOG_DEBUG("");
}

void VescUart::printVescValues() {
  SPDLOG_DEBUG("avgMotorCurrent: {}", data.avgMotorCurrent);
  SPDLOG_DEBUG("avgInputCurrent: {}", data.avgInputCurrent);
  SPDLOG_DEBUG("dutyCycleNow: {}", data.dutyCycleNow);
  SPDLOG_DEBUG("rpm: {}", data.rpm);
  SPDLOG_DEBUG("inputVoltage: {}", data.inpVoltage);
  SPDLOG_DEBUG("ampHours: {}", data.ampHours);
  SPDLOG_DEBUG("ampHoursCharges: {}", data.ampHoursCharged);
  SPDLOG_DEBUG("tachometer: {}", data.tachometer);
  SPDLOG_DEBUG("tachometerAbs: {}", data.tachometerAbs);
}

void VescUart::sendAlive() {
  int32_t index = 0;
  uint8_t payload[1];

  payload[0] = COMM_ALIVE;

  packSendPayload(payload, 5);
}
