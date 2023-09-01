#pragma once

#include <avr/pgmspace.h>
#include <Arduino.h>

#define ISP_DDR     DDRC
#define ISP_PORT    PORTC
#define ISP_PIN     PINC

#define ISP_RST     PC3
#define ISP_DO      PC0
#define ISP_DI      PC1
#define ISP_SCK     PC2

constexpr uint16_t EEPROM_SIZE = 1024;
constexpr uint8_t EEPROM_PAGE_SIZE = 4;
constexpr uint16_t FLASH_SIZE = 32768;
constexpr uint8_t FLASH_PAGE_SIZE = 64;

static void ispInit() {
  ISP_DDR |= (1 << ISP_RST) | (1 << ISP_DO) | (1 << ISP_SCK); // RST, DO and SCK as OUTPUT
  ISP_DDR &= ~(1 << ISP_DI); // DI as INPUT
  ISP_PORT &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_DI)); // RST = LOW, SCK = LOW, DI pullup disabled
}

static void ispDone() {
  ISP_PORT &= ~((1 << ISP_RST) | (1 << ISP_DO) | (1 << ISP_SCK));
  ISP_DDR &= ~((1 << ISP_RST) | (1 << ISP_DI) | (1 << ISP_DO) | (1 << ISP_SCK)); // RST, DI, DO and SCK as INTPUT
}

static void ispReset() {
  ISP_PORT |= (1 << ISP_RST);
  delay(1);
  ISP_PORT &= ~(1 << ISP_RST);
}

static uint8_t ispTransfer(uint8_t data) {
  uint8_t result;

  for (uint8_t bit = 0; bit < 8; ++bit) {
    if (data & 0x80)
      ISP_PORT |= (1 << ISP_DO);
    else
      ISP_PORT &= ~(1 << ISP_DO);
    ISP_PORT |= (1 << ISP_SCK);
    asm volatile ("nop\n"
      "nop\n");
    result <<= 1;
    if (ISP_PIN & (1 << ISP_DI))
      result |= 1;
    ISP_PORT &= ~(1 << ISP_SCK);
    asm volatile ("nop\n"
      "nop\n");
    data <<= 1;
  }
  return result;
}

static bool ispBegin() {
  bool result;
  uint8_t retry = 5;

  while (retry--) {
    ISP_PORT &= ~(1 << ISP_RST);
    delay(20);
    ispTransfer(0xAC);
    ispTransfer(0x53);
    result = ispTransfer(0x00) == 0x53;
    ispTransfer(0x00);
    if (result)
      break;
    ispReset();
  }
  return result;
}

static uint8_t ispCommand(uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t cmd4 = 0x00) {
  ispTransfer(cmd1);
  ispTransfer(cmd2);
  ispTransfer(cmd3);
  return ispTransfer(cmd4);
}

static void ispWait() {
  while (ispCommand(0xF0, 0x00, 0x00) & 0x01) {
    delay(1);
  }
}

static inline uint8_t ispReadLockBits() {
  return ispCommand(0x58, 0x00, 0x00);
}

static inline void ispWriteLockBits(uint8_t bits) {
  ispCommand(0xAC, 0xE0, 0x00, bits);
}

static void ispReadSignature(uint8_t *sign) {
  for (uint8_t i = 0; i < 3; ++i) {
    sign[i] = ispCommand(0x30, 0x00, i);
  }
}

static inline uint8_t ispReadLowFuseBits() {
  return ispCommand(0x50, 0x00, 0x00);
}

static inline void ispWriteLowFuseBits(uint8_t bits) {
  ispCommand(0xAC, 0xA0, 0x00, bits);
}

static inline uint8_t ispReadHighFuseBits() {
  return ispCommand(0x58, 0x08, 0x00);
}

static inline void ispWriteHighFuseBits(uint8_t bits) {
  ispCommand(0xAC, 0xA8, 0x00, bits);
}

static inline uint8_t ispReadExtFuseBits() {
  return ispCommand(0x50, 0x08, 0x00);
}

static inline void ispWriteExtFuseBits(uint8_t bits) {
  ispCommand(0xAC, 0xA4, 0x00, bits);
}

static void ispChipErase() {
  ispCommand(0xAC, 0x80, 0x00, 0x00);
//  delay(9);
  ispWait();
}

static inline uint8_t ispReadEeprom(uint16_t addr) {
  return ispCommand(0xA0, addr / 256, addr);
}

static bool ispWriteEeprom(uint16_t addr, uint8_t data, bool verify = false) {
  ispCommand(0xC0, addr / 256, addr, data);
//  delay(4);
  ispWait();
  if (verify) {
    return ispReadEeprom(addr) == data;
  }
  return true;
}

static bool ispWriteEepromPage(uint16_t addr, const uint8_t *page, bool verify = false) {
  addr &= 0xFFFC;
  for (uint8_t i = 0; i < EEPROM_PAGE_SIZE; ++i) {
    ispCommand(0xC1, 0x00, i, page[i]);
  }
  ispCommand(0xC2, addr / 256, addr, 0);
//  delay(4);
  ispWait();
  if (verify) {
    for (uint8_t i = 0; i < EEPROM_PAGE_SIZE; ++i) {
      if (ispReadEeprom(addr + i) != page[i])
        return false;
    }
  }
  return true;
}

static bool ispWriteEepromPage_P(uint16_t addr, const uint8_t *page, bool verify = false) {
  addr &= 0xFFFC;
  for (uint8_t i = 0; i < EEPROM_PAGE_SIZE; ++i) {
    ispCommand(0xC1, 0x00, i, pgm_read_byte(&page[i]));
  }
  ispCommand(0xC2, addr / 256, addr, 0);
//  delay(4);
  ispWait();
  if (verify) {
    for (uint8_t i = 0; i < EEPROM_PAGE_SIZE; ++i) {
      if (ispReadEeprom(addr + i) != pgm_read_byte(&page[i]))
        return false;
    }
  }
  return true;
}

static inline uint8_t ispReadFlash(uint16_t addr) {
  return ispCommand(0x20 + 0x08 * (addr & 0x01), addr / 512, addr / 2);
}

static void ispFillFlashPage(uint16_t addr, uint16_t data = 0xFFFF) {
  addr /= 2;
  addr &= 0xFFC0;
  for (uint8_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
    ispCommand(0x40, 0x00, i, data);
    ispCommand(0x48, 0x00, i, data / 256);
  }
  ispCommand(0x4C, addr / 256, addr, 0);
//  delay(5);
  ispWait();
}

static bool ispWriteFlashPage(uint16_t addr, const uint8_t *page, bool verify = false) {
  addr /= 2;
  addr &= 0xFFC0;
  for (uint8_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
    ispCommand(0x40, 0x00, i, page[i * 2]);
    ispCommand(0x48, 0x00, i, page[i * 2 + 1]);
  }
  ispCommand(0x4C, addr / 256, addr, 0);
//  delay(5);
  ispWait();
  if (verify) {
    for (uint8_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
      if ((ispReadFlash(addr * 2 + i * 2) != page[i * 2]) || (ispReadFlash(addr * 2 + i * 2 + 1) != page[i * 2 + 1]))
        return false;
    }
  }
  return true;
}

static bool ispWriteFlashPage_P(uint16_t addr, const uint8_t *page, bool verify = false) {
  addr /= 2;
  addr &= 0xFFC0;
  for (uint8_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
    ispCommand(0x40, 0x00, i, pgm_read_byte(&page[i * 2]));
    ispCommand(0x48, 0x00, i, pgm_read_byte(&page[i * 2 + 1]));
  }
  ispCommand(0x4C, addr / 256, addr, 0);
//  delay(5);
  ispWait();
  if (verify) {
    for (uint8_t i = 0; i < FLASH_PAGE_SIZE; ++i) {
      if ((ispReadFlash(addr * 2 + i * 2) != pgm_read_byte(&page[i * 2])) || (ispReadFlash(addr * 2 + i * 2 + 1) != pgm_read_byte(&page[i * 2 + 1])))
        return false;
    }
  }
  return true;
}
