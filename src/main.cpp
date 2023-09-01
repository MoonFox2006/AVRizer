#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "isp.h"

#define USE_HEAP

#define FPSTR(s)  ((__FlashStringHelper*)(s))

enum hextype_t : uint8_t { HEX_BIN = 0, HEX_END, HEX_SEGMENT, HEX_START, HEX_EXTADDR, HEX_START32 };
enum hexparse_t : uint8_t { HEX_OK,
#ifdef USE_HEAP
  HEX_NOMEM,
#endif
  HEX_TOOSHORT, HEX_WRONGSTART, HEX_WRONGLEN, HEX_WRONGADDRHI, HEX_WRONGADDRLO, HEX_WRONGTYPE, HEX_WRONGDATA, HEX_WRONGCRC };

constexpr uint8_t BTN_PIN = 9;
constexpr uint8_t LED1_PIN = 7;
constexpr uint8_t LED2_PIN = 8;
constexpr bool LED_LEVEL = LOW;

constexpr uint8_t SD_PIN = 10;

constexpr uint8_t HEX_PAGE_SIZE = 16;

constexpr uint32_t BLINK_TIME = 50; // 50 ms.

static const char FUSES_NAME[] PROGMEM = "fuses.txt";
static const char FUSES_BACKUP_NAME[] PROGMEM = "fuses.bak";
static const char EEPROM_NAME[] PROGMEM = "eeprom.hex";
static const char EEPROM_BACKUP_NAME[] PROGMEM = "eeprom.bak";
static const char FIRMWARE_NAME[] PROGMEM = "firmware.hex";
static const char FIRMWARE_BACKUP_NAME[] PROGMEM = "firmware.bak";

static const char FAIL_OR_OK[][6] PROGMEM = { "FAIL!", "Done" };
static const char HEX_LINE_HAS[] PROGMEM = "\r\nHEX line has ";

File f;
bool error = false;

static bool fexists(PGM_P fileName) {
  char name[13];

  strcpy_P(name, fileName);
  return SD.exists(name);
}

static uint8_t freadUntil(char *str, uint8_t size, char terminator, char tail) {
  size = f.readBytesUntil(terminator, str, size - 1);
  if (size && (str[size - 1] == tail))
    --size;
  str[size] = '\0';
  return size;
}

static bool parseHexNum(const char *str, uint8_t &num) {
  uint8_t result;

  if ((*str >= '0') && (*str <= '9'))
    result = *str - '0';
  else if ((*str >= 'A') && (*str <= 'F'))
    result = *str - 'A' + 10;
  else if ((*str >= 'a') && (*str <= 'f'))
    result = *str - 'a' + 10;
  else
    return false;
  result <<= 4;
  ++str;
  if ((*str >= '0') && (*str <= '9'))
    result |= *str - '0';
  else if ((*str >= 'A') && (*str <= 'F'))
    result |= *str - 'A' + 10;
  else if ((*str >= 'a') && (*str <= 'f'))
    result |= *str - 'a' + 10;
  else
    return false;
  num = result;
  return true;
}

static hexparse_t parseHexLine(uint8_t &len, uint16_t &addr, hextype_t &type, uint8_t *data) {
  constexpr uint8_t LINE_SIZE = HEX_PAGE_SIZE * 2 + 13 + 1;

#ifdef USE_HEAP
  char *str;
#else
  char str[LINE_SIZE];
#endif
  uint8_t l, crc;
#ifdef USE_HEAP
  hexparse_t result = HEX_NOMEM;
#else
  hexparse_t result;
#endif

#ifdef USE_HEAP
  str = new char[LINE_SIZE];
  if (str) {
#endif
    l = freadUntil(str, LINE_SIZE, '\n', '\r');
    if (l < 11) {
      result = HEX_TOOSHORT;
      goto free_and_return;
    }
    if (str[0] != ':') {
      result = HEX_WRONGSTART;
      goto free_and_return;
    }
    if ((! parseHexNum(&str[1], len)) || (len > HEX_PAGE_SIZE)) {
      result = HEX_WRONGLEN;
      goto free_and_return;
    }
    crc = len;
    if (! parseHexNum(&str[3], l)) {
      result = HEX_WRONGADDRHI;
      goto free_and_return;
    }
    addr = l << 8;
    crc += l;
    if (! parseHexNum(&str[5], l)) {
      result = HEX_WRONGADDRLO;
      goto free_and_return;
    }
    addr |= l;
    crc += l;
    if ((! parseHexNum(&str[7], l)) || (l > HEX_START32)) {
      result = HEX_WRONGTYPE;
      goto free_and_return;
    }
    type = (hextype_t)l;
    crc += l;
    for (uint8_t i = 0; i < len; ++i) {
      if (! parseHexNum(&str[9 + i * 2], data[i])) {
        result = HEX_WRONGDATA;
        goto free_and_return;
      }
      crc += data[i];
    }
    crc = 0 - crc;
    if ((! parseHexNum(&str[9 + len * 2], l)) || (l != crc)) {
      result = HEX_WRONGCRC;
      goto free_and_return;
    }
    result = HEX_OK;
free_and_return:
#ifdef USE_HEAP
    delete[] str;
  }
#endif
  return result;
}

static void printParseError(hexparse_t parse) {
  switch (parse) {
#ifdef USE_HEAP
    case HEX_NOMEM:
      Serial.println(F("\r\nNot enough memory!"));
      break;
#endif
    case HEX_TOOSHORT:
      Serial.println(F("\r\nHEX line too short!"));
      break;
    case HEX_WRONGSTART:
      Serial.print(FPSTR(HEX_LINE_HAS));
      Serial.println(F("wrong start!"));
      break;
    case HEX_WRONGLEN:
      Serial.print(FPSTR(HEX_LINE_HAS));
      Serial.println(F("wrong length!"));
      break;
    case HEX_WRONGADDRHI:
    case HEX_WRONGADDRLO:
      Serial.print(FPSTR(HEX_LINE_HAS));
      Serial.println(F("wrong address!"));
      break;
    case HEX_WRONGTYPE:
      Serial.print(FPSTR(HEX_LINE_HAS));
      Serial.println(F("wrong type!"));
      break;
    case HEX_WRONGDATA:
      Serial.print(FPSTR(HEX_LINE_HAS));
      Serial.println(F("wrong data!"));
      break;
    case HEX_WRONGCRC:
      Serial.print(FPSTR(HEX_LINE_HAS));
      Serial.println(F("wrong CRC!"));
      break;
    default:
      break;
  }
}

static const char *hex(char *str, uint8_t value) {
  uint8_t d;

  d = value / 16;
  if (d > 9)
    str[0] = 'A' + d - 10;
  else
    str[0] = '0' + d;
  d = value & 0x0F;
  if (d > 9)
    str[1] = 'A' + d - 10;
  else
    str[1] = '0' + d;
  str[2] = '\0';
  return str;
}

static bool isEmpty(const uint8_t *data, uint8_t size) {
  while (size--) {
    if (*data++ != 0xFF)
      return false;
  }
  return true;
}

static void printPercent(uint8_t percent) {
  if (percent < 100)
    Serial.write(' ');
  if (percent < 10)
    Serial.write(' ');
  Serial.print(percent);
  Serial.print(F("%\b\b\b\b"));
}

static bool dumpFuses(PGM_P fileName) {
  char name[13];

  strcpy_P(name, fileName);
  f = SD.open(name, O_WRITE | O_CREAT | O_TRUNC);
  if (f) {
    f.print(F("LB:"));
    f.println(hex(name, ispReadLockBits()));
    f.print(F("L:"));
    f.print(hex(name, ispReadLowFuseBits()));
    f.print(F(";H:"));
    f.print(hex(name, ispReadHighFuseBits()));
    f.print(F(";E:"));
    f.println(hex(name, ispReadExtFuseBits()));
    f.close();
    return true;
  }
  return false;
}

static bool programFuses(PGM_P fileName, bool lock = false) {
  constexpr uint8_t STR_SIZE = 17;

  char name[13];
  bool result = false;

  strcpy_P(name, fileName);
  f = SD.open(name, O_READ);
  if (f) {
#ifdef USE_HEAP
    char *str;
#else
    char str[STR_SIZE];
#endif
    uint8_t lb, lf, hf, ef;

#ifdef USE_HEAP
    str = new char[STR_SIZE];
    if (str) {
#endif
      if ((freadUntil(str, STR_SIZE, '\n', '\r') == 5) &&
        (! strncmp_P(str, PSTR("LB:"), 3)) && parseHexNum(&str[3], lb)) { // "LB:XX"
        if ((freadUntil(str, STR_SIZE, '\n', '\r') == 14) &&
          (! strncmp_P(str, PSTR("L:"), 2)) && parseHexNum(&str[2], lf) &&
          (! strncmp_P(&str[5], PSTR("H:"), 2)) && parseHexNum(&str[7], hf) &&
          (! strncmp_P(&str[10], PSTR("E:"), 2)) && parseHexNum(&str[12], ef)) { // "L:XX;H:XX;E:XX"
          ispWriteLowFuseBits(lf);
          ispReset();
          result = ispBegin();
          if (result) {
            ispWriteHighFuseBits(hf);
            ispReset();
            result = ispBegin();
          }
          if (result) {
            ispWriteExtFuseBits(ef);
            ispReset();
            result = ispBegin();
          }
          if (result && lock) {
            ispWriteLockBits(lb);
            ispReset();
            result = ispBegin();
          }
        }
      }
#ifdef USE_HEAP
      delete[] str;
    }
#endif
    f.close();
  }
  return result;
}

static bool dumpEeprom(PGM_P fileName) {
  char name[13];
  bool result = false;

  strcpy_P(name, fileName);
  f = SD.open(name, O_WRITE | O_CREAT | O_TRUNC);
  if (f) {
#ifdef USE_HEAP
    uint8_t *data;
#else
    uint8_t data[HEX_PAGE_SIZE];
#endif
    uint8_t crc;

#ifdef USE_HEAP
    data = new uint8_t[HEX_PAGE_SIZE];
    if (data) {
#endif
      for (uint16_t addr = 0; addr < EEPROM_SIZE; addr += HEX_PAGE_SIZE) {
        crc = 16 + (addr / 256) + (addr & 0xFF);
        for (uint8_t i = 0; i < HEX_PAGE_SIZE; ++i) {
          data[i] = ispReadEeprom(addr + i);
          crc += data[i];
        }
        if (! isEmpty(data, HEX_PAGE_SIZE)) {
          f.print(F(":10"));
          f.print(hex(name, addr / 256));
          f.print(hex(name, addr));
          f.print(F("00"));
          for (uint8_t i = 0; i < HEX_PAGE_SIZE; ++i) {
            f.print(hex(name, data[i]));
          }
          f.println(hex(name, 0 - crc));
        }
        if (! (addr % 128))
          printPercent((uint32_t)addr * 100 / EEPROM_SIZE);
      }
#ifdef USE_HEAP
      delete[] data;
#endif
      f.println(F(":00000001FF"));
      result = true;
      Serial.println(FPSTR(FAIL_OR_OK[1]));
#ifdef USE_HEAP
    }
#endif
    f.close();
  }
  return result;
}

static bool programEeprom(PGM_P fileName) {
  char name[13];
  bool result = false;

  strcpy_P(name, fileName);
  f = SD.open(name, O_READ);
  if (f) {
#ifdef USE_HEAP
    uint8_t *data;
#else
    uint8_t data[HEX_PAGE_SIZE];
#endif
    uint16_t addr;
    hexparse_t parse;
    hextype_t type;
    uint8_t len;
    bool ok;

#ifdef USE_HEAP
    data = new uint8_t[HEX_PAGE_SIZE];
    if (data) {
#endif
      do {
        parse = parseHexLine(len, addr, type, data);
        if ((ok = (parse == HEX_OK))) {
          if (type == HEX_BIN) {
            if (addr + len <= EEPROM_SIZE) {
              for (uint8_t i = 0; i < len; ++i) {
                if (! ispWriteEeprom(addr + i, data[i], true)) {
                  Serial.println(F("\r\nEEPROM write error!"));
                  ok = false;
                  break;
                }
              }
            } else {
              Serial.print(FPSTR(HEX_LINE_HAS));
              Serial.println(F("wrong EEPROM address!"));
              ok = false;
            }
          } else if (type == HEX_END) {
            if ((len == 0) && (addr == 0))
              break;
            else {
              Serial.print(FPSTR(HEX_LINE_HAS));
              Serial.println(F("wrong END!"));
              ok = false;
            }
          } else {
            Serial.print(FPSTR(HEX_LINE_HAS));
            Serial.println(F("unexpected type!"));
            ok = false;
          }
          digitalWrite(LED2_PIN, LED_LEVEL == (millis() % 500 < BLINK_TIME));
        } else {
          printParseError(parse);
        }
      } while (ok);
#ifdef USE_HEAP
      delete[] data;
#endif
      digitalWrite(LED2_PIN, ! LED_LEVEL);
      result = ok;
#ifdef USE_HEAP
    }
#endif
    f.close();
  }
  return result;
}

static bool dumpFlash(PGM_P fileName) {
  char name[13];
  bool result = false;

  strcpy_P(name, fileName);
  f = SD.open(name, O_WRITE | O_CREAT | O_TRUNC);
  if (f) {
#ifdef USE_HEAP
    uint8_t *data;
#else
    uint8_t data[HEX_PAGE_SIZE];
#endif
    uint16_t flashTail;
    uint8_t crc;

#ifdef USE_HEAP
    data = new uint8_t[HEX_PAGE_SIZE];
    if (data) {
#endif
      crc = ispReadHighFuseBits() & 0x06;
      if (crc == 0x06)
        flashTail = 0x3F00;
      else if (crc == 0x04)
        flashTail = 0x3E00;
      else if (crc == 0x02)
        flashTail = 0x3C00;
      else // if (crc == 0x00)
        flashTail = 0x3800;
      for (uint16_t addr = 0; addr < flashTail; addr += HEX_PAGE_SIZE) {
        crc = 16 + (addr / 256) + (addr & 0xFF);
        for (uint8_t i = 0; i < HEX_PAGE_SIZE; ++i) {
          data[i] = ispReadFlash(addr + i);
          crc += data[i];
        }
        if (! isEmpty(data, HEX_PAGE_SIZE)) {
          f.print(F(":10"));
          f.print(hex(name, addr / 256));
          f.print(hex(name, addr));
          f.print(F("00"));
          for (uint8_t i = 0; i < HEX_PAGE_SIZE; ++i) {
            f.print(hex(name, data[i]));
          }
          f.println(hex(name, 0 - crc));
        }
        if (! (addr % 1024))
          printPercent((uint32_t)addr * 100 / flashTail);
      }
#ifdef USE_HEAP
      delete[] data;
#endif
      f.println(F(":00000001FF"));
      result = true;
      Serial.println(FPSTR(FAIL_OR_OK[1]));
#ifdef USE_HEAP
    }
#endif
    f.close();
  }
  return result;
}

static bool programFlash(PGM_P fileName) {
  static const char FLASH_WRITE_ERROR[] PROGMEM = "\r\nFlash write error!";

  char name[13];
  bool result = false;

  strcpy_P(name, fileName);
  f = SD.open(name, O_READ);
  if (f) {
#ifdef USE_HEAP
    uint8_t *page, *data;
#else
    uint8_t page[FLASH_PAGE_SIZE * 2], data[HEX_PAGE_SIZE];
#endif
    uint16_t pageAddr, addr;
    hexparse_t parse;
    hextype_t type;
    uint8_t len;
    bool ok;

#ifdef USE_HEAP
    page = new uint8_t[FLASH_PAGE_SIZE * 2];
    if (page) {
      data = new uint8_t[HEX_PAGE_SIZE];
      if (data) {
#endif
        pageAddr = 0xFFFF;
        ispChipErase();
        do {
          parse = parseHexLine(len, addr, type, data);
          if ((ok = (parse == HEX_OK))) {
            if (type == HEX_EXTADDR) {
              if ((len == 2) && (addr == 0)) {
                ispCommand(0x4D, 0x00, data[1], 0x00);
              } else {
                Serial.print(FPSTR(HEX_LINE_HAS));
                Serial.println(F("wrong EXTADDR!"));
                ok = false;
              }
            } else if (type == HEX_BIN) {
              if (addr + len <= FLASH_SIZE) {
                if ((addr & 0xFF80) != pageAddr) {
                  if (pageAddr != 0xFFFF) {
                    if (! ispWriteFlashPage(pageAddr, page, true)) {
                      Serial.println(FPSTR(FLASH_WRITE_ERROR));
                      ok = false;
                      break;
                    }
                  }
                  pageAddr = addr & 0xFF80;
                  memset(page, 0xFF, FLASH_PAGE_SIZE * 2);
                }
                for (uint8_t i = 0; i < len; ++i) {
                  page[(addr + i) & 0x7F] = data[i];
                }
              } else {
                Serial.print(FPSTR(HEX_LINE_HAS));
                Serial.println(F("wrong flash address!"));
                ok = false;
              }
            } else if (type == HEX_END) {
              if ((len == 0) && (addr == 0)) {
                if (pageAddr != 0xFFFF) {
                  ok = ispWriteFlashPage(pageAddr, page, true);
                  if (! ok)
                    Serial.println(FPSTR(FLASH_WRITE_ERROR));
                }
                break;
              } else {
                Serial.print(FPSTR(HEX_LINE_HAS));
                Serial.println(F("wrong END!"));
                ok = false;
              }
            } else {
              Serial.print(FPSTR(HEX_LINE_HAS));
              Serial.println(F("unexpected type!"));
              ok = false;
            }
            digitalWrite(LED2_PIN, LED_LEVEL == (millis() % 500 < BLINK_TIME));
          } else {
            printParseError(parse);
          }
        } while (ok);
#ifdef USE_HEAP
        delete[] data;
#endif
        digitalWrite(LED2_PIN, ! LED_LEVEL);
        result = ok;
#ifdef USE_HEAP
      }
      delete[] page;
    }
#endif
    f.close();
  }
  return result;
}

void setup() {
  Serial.begin(115200);

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, ! LED_LEVEL);
  digitalWrite(LED2_PIN, ! LED_LEVEL);

  if (! SD.begin(1000000, SD_PIN)) {
    Serial.println(F("No SD card found!"));
    error = true;
    return;
  }

  ispInit();

  while (digitalRead(BTN_PIN)) { // Wait for button click
    digitalWrite(LED2_PIN, LED_LEVEL == (millis() % 1000 < BLINK_TIME));
    delay(10);
  }

  if (ispBegin()) {
    uint8_t sign[3];

    ispReadSignature(sign);
    Serial.print(F("AVR signature: "));
    for (uint8_t i = 0; i < 3; ++i) {
      if (i)
        Serial.print(F(", "));
      if (sign[i] < 0x10)
        Serial.print('0');
      Serial.print(sign[i], HEX);
    }
    Serial.println();
    if ((sign[0] == 0x1E) && (sign[1] == 0x95) && ((sign[2] == 0x0F) || (sign[2] == 0x14))) {
      Serial.print(F("Dump fuses: "));
      Serial.println(FPSTR(FAIL_OR_OK[dumpFuses(FUSES_BACKUP_NAME)]));
      Serial.print(F("Dump EEPROM: "));
      if (! dumpEeprom(EEPROM_BACKUP_NAME))
        Serial.println(FPSTR(FAIL_OR_OK[0]));
      Serial.print(F("Dump flash: "));
      if (! dumpFlash(FIRMWARE_BACKUP_NAME))
        Serial.println(FPSTR(FAIL_OR_OK[0]));

      if (fexists(FIRMWARE_NAME)) {
        Serial.print(F("Flash burning... "));
        if (programFlash(FIRMWARE_NAME)) {
          Serial.println(FPSTR(FAIL_OR_OK[1]));
        } else {
          Serial.println(FPSTR(FAIL_OR_OK[0]));
          error = true;
        }
      }
      if ((! error) && fexists(EEPROM_NAME)) {
        Serial.print(F("EEPROM burning... "));
        if (programEeprom(EEPROM_NAME))
          Serial.println(FPSTR(FAIL_OR_OK[1]));
        else {
          Serial.println(FPSTR(FAIL_OR_OK[0]));
          error = true;
        }
      }
      if ((! error) && fexists(FUSES_NAME)) {
        Serial.print(F("Fuses burning... "));
        if (programFuses(FUSES_NAME))
          Serial.println(FPSTR(FAIL_OR_OK[1]));
        else {
          Serial.println(FPSTR(FAIL_OR_OK[0]));
          error = true;
        }
      }
    } else {
      Serial.println(F("Unexpected AVR signature!"));
      error = true;
    }
  } else {
    Serial.println(F("AVR ISP init fail!"));
    error = true;
  }
  ispDone();
}

void loop() {
  digitalWrite(LED1_PIN, LED_LEVEL == error);
  digitalWrite(LED2_PIN, LED_LEVEL == (! error));
  delay(1000);
  digitalWrite(LED1_PIN, ! LED_LEVEL);
  digitalWrite(LED2_PIN, ! LED_LEVEL);
//  Serial.flush();
  SPI.end();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_bod_disable();
  cli();
  sleep_cpu();
}
