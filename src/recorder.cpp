// =====================================================================================
// recorder.cpp — SD-карта по SPI и WAV-файлы
// =====================================================================================
// Общее описание — в recorder.h. Работа с картой и FAT32 — через стандартные библиотеки
// Arduino SD и SPI.
// =====================================================================================
#include "recorder.h"

#include <SD.h>
#include <SPI.h>

#include "config.h"
#include "driver/gpio.h"

namespace {

constexpr size_t WAV_HEADER_BYTES = 44;
constexpr size_t BUFFER_SAMPLES = 4096;  // 4096 отсчётов × 2 байта = 8 КБ = 0,25 с звука

File wavFile;                   // открытый файл текущей записи
bool recording = false;         // идёт ли запись
char fileName[16];              // имя текущего файла, например "/REC_0001.WAV"
uint32_t dataBytes = 0;         // сколько байт звука уже записано на карту
uint32_t lastCheckpointMs = 0;  // когда последний раз сбрасывали данные на карту

int16_t buffer[BUFFER_SAMPLES];  // отсчёты, которые ещё не ушли на карту
size_t buffered = 0;             // сколько отсчётов сейчас в буфере

// Описание карты, собранное один раз при включении. Повторно не считаем: подсчёт свободного
// места на 32 ГБ может идти секунды, а во время записи столько ждать нельзя.
char info[96] = "карта не подключена";

// Записать заголовок WAV в начало файла. Все числа в WAV хранятся младшим байтом вперёд
// (little-endian), как и в памяти ESP32, поэтому их можно копировать в заголовок memcpy.
void writeHeader() {
  uint8_t h[WAV_HEADER_BYTES];
  auto put16 = [&](size_t offset, uint16_t value) { memcpy(h + offset, &value, 2); };
  auto put32 = [&](size_t offset, uint32_t value) { memcpy(h + offset, &value, 4); };

  memcpy(h + 0, "RIFF", 4);    // признак RIFF-файла
  put32(4, 36 + dataBytes);    // размер всего файла без первых 8 байт
  memcpy(h + 8, "WAVE", 4);    // тип: звук
  memcpy(h + 12, "fmt ", 4);   // блок описания формата
  put32(16, 16);               //   его размер, байт
  put16(20, 1);                //   1 = PCM, несжатые отсчёты
  put16(22, 1);                //   число каналов: 1 (моно)
  put32(24, SAMPLE_RATE);      //   отсчётов в секунду
  put32(28, SAMPLE_RATE * 2);  //   байт в секунду
  put16(32, 2);                //   байт на один отсчёт всех каналов
  put16(34, 16);               //   бит на отсчёт
  memcpy(h + 36, "data", 4);   // блок со звуком
  put32(40, dataBytes);        //   его размер, байт

  wavFile.seek(0);
  wavFile.write(h, sizeof(h));
}

// Отправить накопленный буфер на карту. false — карта записала не всё.
bool flushBuffer() {
  if (buffered == 0) return true;
  const size_t bytes = buffered * sizeof(int16_t);
  const size_t written = wavFile.write((const uint8_t *)buffer, bytes);
  dataBytes += written;
  buffered = 0;
  return written == bytes;
}

}  // namespace

bool sdBegin() {
  SPI.begin(PIN_SD_CLK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  // Пока карта не переключилась в режим SPI, она не управляет линией MISO. На простых модулях
  // без резисторов линия тогда «висит в воздухе»; встроенная подтяжка ESP32 делает старт надёжнее.
  gpio_pullup_en((gpio_num_t)PIN_SD_MISO);

  // Сначала рабочая частота, при неудаче — запасная
  uint32_t spiHz = SD_SPI_HZ;
  if (!SD.begin(PIN_SD_CS, SPI, spiHz)) {
    SD.end();
    spiHz = SD_SPI_HZ_SLOW;
    if (!SD.begin(PIN_SD_CS, SPI, spiHz)) return false;
  }

  snprintf(info, sizeof(info), "%llu МБ, свободно %llu МБ, шина %u МГц%s", SD.cardSize() >> 20,
           (SD.totalBytes() - SD.usedBytes()) >> 20, (unsigned)(spiHz / 1000000),
           spiHz == SD_SPI_HZ ? "" : " (на 20 МГц не завелась: длинные провода или контакт)");
  return true;
}

const char *sdInfo() { return info; }

bool recorderStart() {
  // Первый свободный номер: REC_0001.WAV, REC_0002.WAV, …
  for (int i = 1; i <= 9999; i++) {
    snprintf(fileName, sizeof(fileName), "/REC_%04d.WAV", i);
    if (!SD.exists(fileName)) break;
  }

  wavFile = SD.open(fileName, FILE_WRITE);
  if (!wavFile) return false;

  dataBytes = 0;
  buffered = 0;
  writeHeader();  // пока с нулевой длиной звука, исправится в recorderTick и recorderStop
  lastCheckpointMs = millis();
  recording = true;
  Serial.printf("● Запись: %s\n", fileName);
  return true;
}

bool recorderWrite(const int16_t *samples, size_t count) {
  if (!recording) return true;

  // Копируем отсчёты в буфер; как только он заполнился — отправляем его на карту
  while (count > 0) {
    const size_t space = BUFFER_SAMPLES - buffered;
    const size_t n = count < space ? count : space;
    memcpy(buffer + buffered, samples, n * sizeof(int16_t));
    buffered += n;
    samples += n;
    count -= n;
    if (buffered == BUFFER_SAMPLES && !flushBuffer()) return false;
  }
  return true;
}

void recorderTick() {
  if (!recording || millis() - lastCheckpointMs < CHECKPOINT_MS) return;
  lastCheckpointMs = millis();

  writeHeader();                              // заголовок с текущей длиной звука
  wavFile.seek(WAV_HEADER_BYTES + dataBytes);  // вернуться в конец файла, к месту записи
  wavFile.flush();                            // сбросить на карту данные и таблицу FAT
}

void recorderStop() {
  if (!recording) return;
  recording = false;

  if (!flushBuffer()) Serial.println("! Не удалось дописать конец записи на карту");
  writeHeader();  // окончательная длина звука
  wavFile.close();

  Serial.printf("■ Стоп: %s, %.1f с, %u КБ\n", fileName, dataBytes / (SAMPLE_RATE * 2.0f),
                (unsigned)(dataBytes / 1024));
}
