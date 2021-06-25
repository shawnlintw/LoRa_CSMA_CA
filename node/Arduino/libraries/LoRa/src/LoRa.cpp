// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <LoRa.h>

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE 			 0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_SYMB_TIMEOUT_LSB     0x1f
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MAX_PAYLOAD_LENGTH   0x23
#define REG_MODEM_CONFIG_3       0x26
#define REG_PPM_CORRECTION       0x27
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_DIO_MAPPING_2        0x41
#define REG_VERSION              0x42
#define REG_TCXO                 0x4b
#define REG_PA_DAC               0x4d

// PA config
#define PA_BOOST                 0x80

#define MAX_PKT_LENGTH           255

#if (ESP8266 || ESP32)
    #define ISR_PREFIX ICACHE_RAM_ATTR
#else
    #define ISR_PREFIX
#endif

LoRaClass::LoRaClass() :
  _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
  _spi(&LORA_DEFAULT_SPI),
  _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
  _frequency(434E6),
  _bandWidth(125E3),
  _packetIndex(0),
  _implicitHeaderMode(0),
  _onReceive(NULL),
  _onTxDone(NULL)
{
  // overide Stream timeout value
  setTimeout(0);
}

int LoRaClass::begin(long frequency)
{
#if defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310)
  pinMode(LORA_IRQ_DUMB, OUTPUT);
  digitalWrite(LORA_IRQ_DUMB, LOW);

  // Hardware reset
  pinMode(LORA_BOOT0, OUTPUT);
  digitalWrite(LORA_BOOT0, LOW);

  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, HIGH);
  delay(200);
  digitalWrite(LORA_RESET, LOW);
  delay(200);
  digitalWrite(LORA_RESET, HIGH);
  delay(50);
#endif

  // setup pins
  pinMode(_ss, OUTPUT);
  // set SS high
  digitalWrite(_ss, HIGH);

  if (_reset != -1) {
    pinMode(_reset, OUTPUT);

    // perform reset
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }

  // start SPI
  _spi->begin();

  // check version
  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    return 0;
  }

  // put in sleep mode
  sleep();

  // set frequency
  setFrequency(frequency);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  // set auto AGC
  writeRegister(REG_MODEM_CONFIG_3, 0x04);

  // set output power to 17 dBm
  setTxPower(17);

  // put in standby mode
  idle();

  return 1;
}

void LoRaClass::end()
{
  // put in sleep mode
  sleep();

  // stop SPI
  _spi->end();
}

int LoRaClass::beginPacket(int implicitHeader)
{
  if (isTransmitting()) {
    return 0;
  }

  // put in standby mode
  idle();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int LoRaClass::endPacket(bool async, const unsigned long &timeoutMs)
{
  // BW: 7800 kHz, SF: 12, Preamble: 12 => Whole packet time: 16 935.4 ms

  tx();

  if (!async) {
    // wait for TX done
    const unsigned long startMs = millis();
    while ((readRegister(REG_IRQ_FLAGS) & LORA_IRQ_FLAG_TX_DONE) == 0) {
      if ((millis() - startMs) >= timeoutMs) {
        return 0;
      }

      yield();
    }
    clearInterrupts(LORA_IRQ_FLAG_TX_DONE);
  }

  return 1;
}

bool LoRaClass::isTransmitting()
{
  if (getDeviceMode() == DeviceMode::Transmit) {
    return true;
  }

  clearInterrupts(LORA_IRQ_FLAG_TX_DONE);
  return false;
}

int LoRaClass::parsePacket(int size)
{
  int payloadLength = 0;
  const uint8_t irqFlags = readInterrupts();

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  if (irqFlags) {
    clearInterrupts(irqFlags);
  }

  if ((irqFlags & LORA_IRQ_FLAG_RX_DONE) && (irqFlags & LORA_IRQ_FLAG_PAYLOAD_CRC_ERROR) == 0) {
    // received a packet
    _packetIndex = 0;

    // read payload length
    payloadLength = getPayloadLength();

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    idle();
  } else if (getDeviceMode() != DeviceMode::ReceiveSingle) {
    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    rxSingle();
  }

  return payloadLength;
}

int LoRaClass::Rssi_val()
{
  return (_frequency <= 525000000 ? -164 : -157) + readRegister(REG_RSSI_VALUE);
}


int LoRaClass::packetRssi()
{
  return (_frequency <= 525000000 ? -164 : -157) + readRegister(REG_PKT_RSSI_VALUE);
}

float LoRaClass::packetSnr()
{
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

float LoRaClass::packetFrequencyError()
{
  int32_t freqError = 0;
  freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & B111);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

  if (readRegister(REG_FREQ_ERROR_MSB) & B1000) { // Sign bit is on
     freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

  return fError;
}

void LoRaClass::compensateFrequencyOffset(const float &fError)
{
  const int8_t ppmOffset = static_cast<int8_t>(0.95f * (fError * 10E6f / _frequency));

  setFrequency(static_cast<long>(_frequency - fError));
  writeRegister(REG_PPM_CORRECTION, reinterpret_cast<const uint8_t&>(ppmOffset));
}

size_t LoRaClass::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

int LoRaClass::available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRaClass::read()
{
  if (!available()) {
    return -1;
  }

  _packetIndex++;

  return readRegister(REG_FIFO);
}

int LoRaClass::peek()
{
  if (!available()) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = readRegister(REG_FIFO);

  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

void LoRaClass::flush()
{
}

#ifndef ARDUINO_SAMD_MKRWAN1300
void LoRaClass::onReceive(void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {
    pinMode(_dio0, INPUT);
    setInterruptMode(0, LORA_IRQ_DIO0_RXDONE);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
  }
}

void LoRaClass::onTxDone(void(*callback)())
{
  _onTxDone = callback;

  if (callback) {
    pinMode(_dio0, INPUT);
    setInterruptMode(0, LORA_IRQ_DIO0_TXDONE);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
#endif // SPI_HAS_NOTUSINGINTERRUPT
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif // SPI_HAS_NOTUSINGINTERRUPT
  }
}

void LoRaClass::receive(int size)
{

  writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  rxContinuous();
}

uint8_t LoRaClass::getPayloadLength()
{
  return _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);
}
#endif // !ARDUINO_SAMD_MKRWAN1300

void LoRaClass::idle()
{
  writeRegister(REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_STDBY);
  delayMicroseconds(210); // SLEEP -> IDLE takes about ~210 µs (RXCONT -> IDLE: ~80 µs)
}

void LoRaClass::sleep()
{
  writeRegister(REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_SLEEP);
  delayMicroseconds(90); // RXCONTINUOUS -> SLEEP takes about ~70-90 µs
}

void LoRaClass::cad()
{
  idle();
  writeRegister(REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_CAD);
  delayMicroseconds(120); // IDLE -> CAD takes about ~120 µs
}

void LoRaClass::tx()
{
  writeRegister(REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_TX);
  delayMicroseconds(220); // IDLE -> TX takes about ~220 µs
}

void LoRaClass::rxSingle()
{
  writeRegister(REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_RX_SINGLE);
  delayMicroseconds(120); // IDLE -> RXSINGLE takes about ~120 µs
}

void LoRaClass::rxContinuous()
{
  writeRegister(REG_OP_MODE, LORA_MODE_LONG_RANGE_MODE | LORA_MODE_RX_CONTINUOUS);
  delayMicroseconds(115); // IDLE -> RXCONTINUOUS takes about ~115 µs
}

void LoRaClass::setTxPower(int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < -1) {
      level = -1;
    } else if (level > 14) {
      level = 14;
    }

    writeRegister(REG_PA_CONFIG, 0x70 | (level + 1));
  } else {
    // PA BOOST
    if (level > 17) {
      if (level > 20) {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      writeRegister(REG_PA_DAC, 0x87);
      setOCP(140);
    } else {
      if (level < 2) {
        level = 2;
      }
      //Default value PA_HF/LF or +17dBm
      writeRegister(REG_PA_DAC, 0x84);
      setOCP(100);
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

void LoRaClass::setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int LoRaClass::getSpreadingFactor()
{
  return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

void LoRaClass::setSpreadingFactor(int sf)
{
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  setLdoFlag();
}

long LoRaClass::getSignalBandwidth()
{
  byte bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

  switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
  }

  return -1;
}

void LoRaClass::setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    _bandWidth = 7.8E3;
    bw = 0;
  } else if (sbw <= 10.4E3) {
    _bandWidth = 10.4E3;
    bw = 1;
  } else if (sbw <= 15.6E3) {
    _bandWidth = 15.6E3;
    bw = 2;
  } else if (sbw <= 20.8E3) {
    _bandWidth = 20.8E3;
    bw = 3;
  } else if (sbw <= 31.25E3) {
    _bandWidth = 31.25E3;
    bw = 4;
  } else if (sbw <= 41.7E3) {
    _bandWidth = 41.7E3;
    bw = 5;
  } else if (sbw <= 62.5E3) {
    _bandWidth = 62.5E3;
    bw = 6;
  } else if (sbw <= 125E3) {
    _bandWidth = 125E3;
    bw = 7;
  } else if (sbw <= 250E3) {
    _bandWidth = 250E3;
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    _bandWidth = 500E3;
    bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  setLdoFlag();
}

void LoRaClass::setLdoFlag()
{
  // Section 4.1.1.5
  long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

  // Section 4.1.1.6
  boolean ldoOn = symbolDuration > 16;

  uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
  bitWrite(config3, 3, ldoOn);
  writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LoRaClass::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRaClass::setPreambleLength(uint16_t length)
{
  if (length < 6) { // p. 29
    length = 6;
  }

  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRaClass::setSymbolTimeout(uint16_t symbols)
{
  if (symbols > 1023) { // p. 40
    symbols = 1023;
  } else if (symbols < 4) {
    symbols = 4;
  }

  const uint8_t currentMCValue = readRegister(REG_MODEM_CONFIG_2) & B11111100;
  writeRegister(REG_MODEM_CONFIG_2, currentMCValue | ((uint8_t)(symbols >> 8)));
  writeRegister(REG_SYMB_TIMEOUT_LSB, (uint8_t)(symbols >> 0));
}

void LoRaClass::setSyncWord(uint8_t sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

void LoRaClass::setMaxPayloadLength(const uint8_t payloadLength)
{
  writeRegister(REG_MAX_PAYLOAD_LENGTH, payloadLength);
}

void LoRaClass::enableLowDataRateOptimize(bool enabled)
{
  uint8_t regValue = readRegister(REG_MODEM_CONFIG_3);
  writeRegister(REG_MODEM_CONFIG_3, bitWrite(regValue, 3, enabled));
}

void LoRaClass::enableTcxo(const bool enabled)
{
    uint8_t regValue = readRegister(REG_TCXO);
    writeRegister(REG_TCXO, bitWrite(regValue, 4, enabled));
}

void LoRaClass::enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRaClass::disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRaClass::enableInvertIQ()
{
  writeRegister(REG_INVERTIQ,  0x66);
  writeRegister(REG_INVERTIQ2, 0x19);
}

void LoRaClass::disableInvertIQ()
{
  writeRegister(REG_INVERTIQ,  0x27);
  writeRegister(REG_INVERTIQ2, 0x1d);
}

void LoRaClass::setOCP(uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }

  writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRaClass::setInterruptMode(byte pin, byte mode)
{
  if (pin <= 3) {
    uint8_t mapping = readRegister(REG_DIO_MAPPING_1);
    bitWrite(mapping, 6 - (pin * 2), bitRead(mode, 0));
    bitWrite(mapping, 6 - (pin * 2) + 1, bitRead(mode, 1));
    writeRegister(REG_DIO_MAPPING_1, mapping);
  } else if (pin <= 5){
    uint8_t mapping = readRegister(REG_DIO_MAPPING_2);
    bitWrite(mapping, 14 - (pin * 2), bitRead(mode, 0));
    bitWrite(mapping, 14 - (pin * 2) + 1, bitRead(mode, 1));
    writeRegister(REG_DIO_MAPPING_2, mapping);
  }
}

uint8_t LoRaClass::readInterrupts()
{
  return readRegister(REG_IRQ_FLAGS);
}

void LoRaClass::clearInterrupts(uint8_t irqFlags)
{
    writeRegister(REG_IRQ_FLAGS, irqFlags);
}

byte LoRaClass::random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}

void LoRaClass::setPins(int ss, int reset, int dio0)
{
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
}

void LoRaClass::setSPI(SPIClass& spi)
{
  _spi = &spi;
}

void LoRaClass::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void LoRaClass::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}

LoRaClass::DeviceMode LoRaClass::getDeviceMode()
{
  const uint8_t mode = readRegister(REG_OP_MODE) & B111;
  return static_cast<DeviceMode>(mode);
}

void LoRaClass::explicitHeaderMode()
{
  _implicitHeaderMode = 0;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRaClass::implicitHeaderMode()
{
  _implicitHeaderMode = 1;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRaClass::handleDio0RiseRx()
{
  const uint8_t irqFlags = readInterrupts();

  if (irqFlags) {
    clearInterrupts(irqFlags);
  }

  if ((irqFlags & LORA_IRQ_FLAG_PAYLOAD_CRC_ERROR) == 0) {

    if ((irqFlags & LORA_IRQ_FLAG_RX_DONE) != 0) {
      // received a packet
      _packetIndex = 0;

      // read packet length
    int packetLength = getPayloadLength();

      // set FIFO address to current RX address
      writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

      if (_onReceive) {
        _onReceive(packetLength);
      }
    }
    else if ((irqFlags & LORA_IRQ_FLAG_TX_DONE) != 0) {
      if (_onTxDone) {
        _onTxDone();
      }
    }
  }
}

uint8_t LoRaClass::readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;

  digitalWrite(_ss, LOW);

  _spi->beginTransaction(_spiSettings);
  _spi->transfer(address);
  response = _spi->transfer(value);
  _spi->endTransaction();

  digitalWrite(_ss, HIGH);

  return response;
}

ISR_PREFIX void LoRaClass::onDio0Rise()
{
  Serial.println(F("Not implemented!"));
}
LoRaClass LoRa;
