// =====================================================================================
// Диктофон на ESP32-S3-Zero: микрофон INMP441 → WAV-файлы на microSD
// =====================================================================================
// Управление — кнопкой BOOT на плате:
//   короткое нажатие      — начать / остановить запись;
//   долгое нажатие (2 с)  — режим USB-флешки: карта видна компьютеру, можно забрать файлы.
//                           Выход — «Извлечь» в Windows или ещё одно долгое нажатие,
//                           после чего плата перезагружается.
//
// Светодиод на плате:
//   синий                 — готов к записи;
//   мигает красным        — идёт запись;
//   голубой               — режим USB-флешки;
//   часто мигает белым    — ошибка, причина в мониторе порта; после устранения нажать RESET.
// На некоторых ревизиях платы красный и зелёный перепутаны.
//
// Из каких файлов состоит прошивка:
//   config.h      — пины и настройки (частота, фильтр, усиление, тайминги);
//   mic.h/.cpp    — микрофон: шина I2S, DMA, фильтр, перевод в 16 бит;
//   recorder.h/.cpp — SD-карта и WAV-файлы;
//   usb_drive.h/.cpp — режим USB-флешки;
//   main.cpp      — этот файл: режимы работы, кнопка, светодиод.
//
// Логи идут в монитор порта (115200). USB работает в режиме OTG, поэтому Serial пишет только
// тогда, когда монитор порта открыт; сообщения, отправленные до этого, теряются. Чтобы не
// гадать, что было при запуске, плата при каждом открытии монитора печатает своё состояние.
// =====================================================================================

#include <Arduino.h>

#include "config.h"
#include "mic.h"
#include "recorder.h"
#include "usb_drive.h"

// ---------------------------------------------------------------------------------------
// Режимы работы. В каждый момент устройство находится ровно в одном из них.
// ---------------------------------------------------------------------------------------
enum class Mode {
  Ready,      // ждём нажатия кнопки
  Recording,  // пишем звук в файл
  UsbDrive,   // карта отдана компьютеру, прошивка её не трогает
  Error,      // SD-карта или микрофон не работают
};

static Mode mode = Mode::Error;
static bool sdReady = false;               // карта смонтирована (USB-флешка доступна даже без микрофона)
static bool micReady = false;              // драйвер I2S запущен
static const char *errorReason = nullptr;  // причина ошибки, для вывода в монитор порта

static int16_t chunk[MIC_CHUNK_SAMPLES];  // последняя порция звука от микрофона (30 мс)
static uint32_t lostAtStart = 0;          // счётчик потерь микрофона в момент начала записи

// ---------------------------------------------------------------------------------------
// Переходы между режимами
// ---------------------------------------------------------------------------------------

static void fail(const char *reason) {
  errorReason = reason;
  Serial.printf("ОШИБКА: %s\n", reason);
  mode = Mode::Error;
}

static void startRecording() {
  if (!recorderStart()) {
    fail("не удалось создать файл на SD-карте");
    return;
  }
  lostAtStart = micLostBuffers();
  mode = Mode::Recording;
}

static void stopRecording() {
  recorderStop();
  mode = Mode::Ready;

  // Потери означают, что основной цикл не успевал забирать звук: в файле есть «дырки» по 30 мс.
  // На исправном макете их не бывает; если появились — карта слишком медленная или сбоит.
  const uint32_t lost = micLostBuffers() - lostAtStart;
  if (lost > 0) Serial.printf("! Потеряно %u кусков звука по 30 мс\n", (unsigned)lost);
}

static void enterUsbDrive() {
  if (mode == Mode::Recording) stopRecording();  // файл должен быть закрыт до передачи карты
  usbDriveEnable();
  mode = Mode::UsbDrive;
  Serial.println("USB-флешка включена. Выход: «Извлечь» в Windows или долгое нажатие BOOT");
}

static void leaveUsbDrive() {
  usbDriveDisable();
  Serial.println("USB-флешка выключена, перезагрузка...");
  Serial.flush();
  delay(500);      // дать компьютеру заметить, что носитель вынут
  esp_restart();   // при старте карта смонтируется заново, с изменениями, сделанными компьютером
}

// ---------------------------------------------------------------------------------------
// Кнопка BOOT
// ---------------------------------------------------------------------------------------
enum class Press { None, Short, Long };

// Опросить кнопку. Короткое нажатие засчитывается при отпускании, долгое — сразу, как только
// кнопку продержали LONG_PRESS_MS (не дожидаясь отпускания, чтобы светодиод сразу сменил цвет).
// После долгого нажатия отпускание уже не считается коротким.
static Press readButton() {
  static bool lastRaw = false;      // сырое состояние на прошлом опросе
  static uint32_t changedMs = 0;    // когда сырое состояние последний раз менялось
  static bool pressed = false;      // состояние после защиты от дребезга
  static uint32_t pressedMs = 0;    // когда кнопку нажали
  static bool longReported = false; // долгое нажатие уже засчитано

  const uint32_t now = millis();
  const bool raw = digitalRead(PIN_BUTTON) == LOW;  // LOW = нажата
  if (raw != lastRaw) {
    lastRaw = raw;
    changedMs = now;
  }

  Press result = Press::None;

  // Новое состояние принимаем, только если оно продержалось DEBOUNCE_MS
  if (raw != pressed && now - changedMs >= DEBOUNCE_MS) {
    pressed = raw;
    if (pressed) {
      pressedMs = now;
      longReported = false;
    } else if (!longReported) {
      result = Press::Short;
    }
  }

  if (pressed && !longReported && now - pressedMs >= LONG_PRESS_MS) {
    longReported = true;
    result = Press::Long;
  }
  return result;
}

// ---------------------------------------------------------------------------------------
// Светодиод
// ---------------------------------------------------------------------------------------

// Показать текущий режим цветом. Цвет — число 0xRRGGBB; яркость небольшая (0x18 из 0xFF),
// иначе светодиод слепит. Мигание делается по millis(): полпериода горит, полпериода нет.
static void showMode() {
  static uint32_t shownColor = 0xFFFFFFFF;  // какой цвет уже горит — чтобы не обновлять зря
  const uint32_t now = millis();

  uint32_t color = 0;
  switch (mode) {
    case Mode::Ready:
      color = 0x000018;  // синий
      break;
    case Mode::Recording:
      color = (now / 500) % 2 ? 0x300000 : 0;  // красный, мигает раз в секунду
      break;
    case Mode::UsbDrive:
      color = 0x001818;  // голубой
      break;
    case Mode::Error:
      color = (now / 100) % 2 ? 0x202020 : 0;  // белый, мигает 5 раз в секунду
      break;
  }

  if (color != shownColor) {
    rgbLedWrite(PIN_LED, color >> 16, (color >> 8) & 0xFF, color & 0xFF);
    shownColor = color;
  }
}

// ---------------------------------------------------------------------------------------
// Монитор порта
// ---------------------------------------------------------------------------------------

// Напечатать текущее состояние: что с картой, в каком режиме устройство, какая ошибка
static void printStatus() {
  Serial.println("\n=== Диктофон ESP32-S3-Zero ===");
  Serial.printf("SD: %s\n", sdInfo());
  switch (mode) {
    case Mode::Ready:
      Serial.println("Готов. BOOT — запись старт/стоп, BOOT 2 с — USB-флешка");
      break;
    case Mode::Recording:
      Serial.println("Идёт запись. BOOT — стоп");
      break;
    case Mode::UsbDrive:
      Serial.println("Включена USB-флешка. Выход: «Извлечь» в Windows или долгое нажатие BOOT");
      break;
    case Mode::Error:
      Serial.printf("ОШИБКА: %s\n", errorReason ? errorReason : "неизвестна");
      break;
  }
}

// Если монитор порта только что открыли — показать состояние.
// Serial в режиме USB-OTG считается подключённым, когда программа на компьютере открыла порт
// с сигналами DTR и RTS (монитор PlatformIO так и делает).
static void printStatusOnConnect() {
  static bool wasConnected = false;
  const bool connected = Serial;
  if (connected && !wasConnected) printStatus();
  wasConnected = connected;
}

// ---------------------------------------------------------------------------------------
// setup() и loop()
// ---------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);  // в режиме USB-OTG скорость роли не играет

  pinMode(PIN_BUTTON, INPUT_PULLUP);  // на плате есть внешняя подтяжка, внутренняя не помешает

  sdReady = sdBegin();
  if (sdReady) usbDriveSetup();
  micReady = micBegin();

  if (!sdReady) {
    fail("SD-карта не отвечает: проверьте, что она вставлена и в FAT32, и провода SPI");
  } else if (!micReady) {
    fail("не запустился драйвер I2S микрофона");
  } else {
    mode = Mode::Ready;
  }
}

void loop() {
  // 0. Открыли монитор порта — показать состояние
  printStatusOnConnect();

  // 1. Кнопка
  switch (readButton()) {
    case Press::Short:
      if (mode == Mode::Ready) startRecording();
      else if (mode == Mode::Recording) stopRecording();
      break;
    case Press::Long:
      if (mode == Mode::UsbDrive) leaveUsbDrive();
      else if (sdReady) enterUsbDrive();  // из любого режима, в том числе из ошибки микрофона
      break;
    case Press::None:
      break;
  }

  // 2. В Windows нажали «Извлечь»
  if (mode == Mode::UsbDrive && usbDriveEjected()) leaveUsbDrive();

  // 3. Звук. Забираем его с микрофона всегда, даже когда не пишем: иначе кольцо DMA
  //    переполнится и счётчик потерь будет расти зря. micRead() ждёт до 30 мс свежих данных,
  //    поэтому цикл крутится примерно 33 раза в секунду.
  if (micReady && mode != Mode::Error) {
    const size_t count = micRead(chunk);

    if (mode == Mode::Recording) {
      if (recorderWrite(chunk, count)) {
        recorderTick();
      } else {
        recorderStop();
        fail("не удалось записать на SD-карту");
      }
    }

    // Здесь удобно подключать собственную обработку звука: в chunk лежат count свежих
    // отсчётов (30 мс), уже отфильтрованных и усиленных. Обработка должна укладываться
    // в ~30 мс на порцию, иначе микрофон начнёт терять звук (см. micLostBuffers()).
  } else {
    delay(20);  // микрофон не читаем — просто не крутим цикл вхолостую
  }

  // 4. Светодиод
  showMode();
}
