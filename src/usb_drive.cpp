// =====================================================================================
// usb_drive.cpp — USB Mass Storage поверх SD-карты
// =====================================================================================
// Общее описание — в usb_drive.h. USB-накопитель — класс USBMSC из Arduino-ядра ESP32
// (внутри библиотека TinyUSB). Компьютер шлёт команды «прочитай/запиши такие-то сектора»,
// а мы выполняем их функциями SD.readRAW / SD.writeRAW — прямым доступом к секторам карты
// в обход файловой системы.
//
// Функции onRead/onWrite/onStartStop вызываются не из loop(), а из отдельной задачи USB.
// =====================================================================================
#include "usb_drive.h"

#include <SD.h>

#include "USB.h"
#include "USBMSC.h"

namespace {

constexpr uint32_t SECTOR_BYTES = 512;  // размер сектора SD-карты

// Объект накопителя создаётся до setup(), вместе с остальными глобальными объектами.
// Это важно: компьютер узнаёт состав USB-устройства один раз, в момент подключения,
// и накопитель должен быть в этом описании с самого начала.
USBMSC msc;

// Флаг «компьютер извлёк диск». Пишется из задачи USB, читается из loop(), поэтому volatile.
volatile bool ejected = false;

// Компьютер просит bufsize байт, начиная с сектора lba. Возвращаем число отданных байт или −1.
// Запросы всегда кратны сектору, поэтому смещение внутри сектора (второй параметр) не нужно.
int32_t onRead(uint32_t lba, uint32_t /*offset*/, void *buffer, uint32_t bufsize) {
  uint8_t *out = (uint8_t *)buffer;
  for (uint32_t i = 0; i < bufsize / SECTOR_BYTES; i++) {
    if (!SD.readRAW(out + i * SECTOR_BYTES, lba + i)) return -1;
  }
  return bufsize;
}

// Компьютер хочет записать bufsize байт, начиная с сектора lba
int32_t onWrite(uint32_t lba, uint32_t /*offset*/, uint8_t *buffer, uint32_t bufsize) {
  for (uint32_t i = 0; i < bufsize / SECTOR_BYTES; i++) {
    if (!SD.writeRAW(buffer + i * SECTOR_BYTES, lba + i)) return -1;
  }
  return bufsize;
}

// Команда «старт/стоп носителя». «Извлечь» в Windows приходит как стоп с выталкиванием.
bool onStartStop(uint8_t /*powerCondition*/, bool start, bool loadEject) {
  if (loadEject && !start) ejected = true;
  return true;
}

}  // namespace

void usbDriveSetup() {
  // Эти строки компьютер показывает как название устройства
  msc.vendorID("ESP32");          // до 8 символов
  msc.productID("Recorder SD");   // до 16 символов
  msc.productRevision("1.0");     // до 4 символов
  msc.onRead(onRead);
  msc.onWrite(onWrite);
  msc.onStartStop(onStartStop);
  msc.isWritable(true);           // с компьютера можно и удалять файлы
  msc.begin(SD.numSectors(), SD.sectorSize());
  msc.mediaPresent(false);        // до usbDriveEnable() — «носителя нет»
}

void usbDriveEnable() { msc.mediaPresent(true); }

void usbDriveDisable() { msc.mediaPresent(false); }

bool usbDriveEjected() { return ejected; }
