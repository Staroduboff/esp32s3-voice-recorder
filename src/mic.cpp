// =====================================================================================
// mic.cpp — микрофон INMP441 по I2S: настройка драйвера, фильтр, перевод в 16 бит
// =====================================================================================
// Общее описание — в mic.h. Здесь используется I2S-драйвер ESP-IDF 5.x (driver/i2s_std.h),
// он доступен из Arduino 3.x напрямую.
// =====================================================================================
#include "mic.h"

#include "config.h"
#include "driver/i2s_std.h"

// Всё в безымянном пространстве имён видно только внутри этого файла
namespace {

// Сколько DMA-буферов в кольце. 16 буферов по 30 мс = 0,48 с звука в запасе: столько основной
// цикл может не забирать данные, пока SD-карта занята записью. На макете самая долгая запись
// блока на карту — 45 мс, так что запас десятикратный.
constexpr size_t DMA_BUFFERS = 16;

i2s_chan_handle_t rxChannel = nullptr;  // «ручка» канала приёма I2S, её выдаёт драйвер
volatile uint32_t lostBuffers = 0;      // volatile: меняется в прерывании, читается в программе

// Приёмный массив. Шина настроена как стерео, поэтому на каждый отсчёт приходят два 32-битных
// слова: [левый, правый, левый, правый, …]. Микрофон говорит только в левом. Читать стерео
// и брать левое слово надёжнее, чем моно-режим драйвера, и именно так проверено на макете.
int32_t frames[MIC_CHUNK_SAMPLES * 2];

// Драйвер вызывает эту функцию из прерывания, когда кольцо DMA переполнилось и самый старый
// буфер затёрт новыми данными. IRAM_ATTR кладёт код во внутреннюю память: обработчик
// прерывания не должен ждать чтения из флеш-памяти.
bool IRAM_ATTR onDmaOverflow(i2s_chan_handle_t, i2s_event_data_t *, void *) {
  lostBuffers = lostBuffers + 1;
  return false;  // false: обработчику не нужно переключать задачи FreeRTOS
}

// ---------------------------------------------------------------------------------------
// Фильтр высоких частот (ФВЧ) Баттерворта 2-го порядка, он же «биквад»
// ---------------------------------------------------------------------------------------
// Пропускает всё выше частоты среза, а ниже плавно ослабляет (в 4 раза на каждую октаву вниз).
// Каждый новый выходной отсчёт считается из нового входного, двух предыдущих входных и двух
// предыдущих выходных — отсюда x1, x2, y1, y2. Формулы коэффициентов взяты из
// «Audio EQ Cookbook» (Robert Bristow-Johnson), раздел HPF.
class HighPass {
 public:
  HighPass(float cutoffHz, float sampleRate) {
    const float w = 2.0f * PI * cutoffHz / sampleRate;  // частота среза в радианах на отсчёт
    const float q = 0.7071f;                            // добротность 1/√2 — фильтр Баттерворта
    const float alpha = sinf(w) / (2.0f * q);
    const float a0 = 1.0f + alpha;
    // Все коэффициенты сразу делим на a0, чтобы в process() не делить на каждом отсчёте
    b0 = (1.0f + cosf(w)) / 2.0f / a0;
    b1 = -(1.0f + cosf(w)) / a0;
    b2 = b0;
    a1 = -2.0f * cosf(w) / a0;
    a2 = (1.0f - alpha) / a0;
  }

  // Пропустить через фильтр один отсчёт
  float process(float x) {
    const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;  // сдвигаем «историю» на один отсчёт
    x1 = x;
    y2 = y1;
    y1 = y;
    return y;
  }

 private:
  float b0, b1, b2, a1, a2;              // коэффициенты фильтра
  float x1 = 0, x2 = 0, y1 = 0, y2 = 0;  // два предыдущих входных и два выходных отсчёта
};

HighPass highPass(HIGHPASS_HZ, SAMPLE_RATE);

// Отсчёт с шины I2S → 16-битный отсчёт WAV.
// Микрофон кладёт 24-битное число в старшие разряды 32-битного слова, поэтому полная шкала
// слова (±2^31) — это и полная шкала микрофона.
int16_t toWavSample(int32_t raw) {
  float v = raw / 2147483648.0f;  // в доли полной шкалы: от −1 до +1
  v = highPass.process(v);        // убрать постоянное смещение и низкочастотный гул
  v *= GAIN * 32768.0f;           // усилить и перевести в шкалу 16 бит (±32768)
  // Громкий звук после усиления может выйти за пределы 16 бит. Без ограничения число
  // «перевернулось» бы через ноль, и вместо хрипа был бы громкий треск.
  if (v > 32767.0f) v = 32767.0f;
  if (v < -32768.0f) v = -32768.0f;
  return (int16_t)v;
}

}  // namespace

bool micBegin() {
  // 1. Канал I2S: ESP32 — ведущий (сама выдаёт такты SCK и WS), только приём.
  //    DMA: DMA_BUFFERS буферов по MIC_CHUNK_SAMPLES кадров.
  i2s_chan_config_t channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channelConfig.dma_desc_num = DMA_BUFFERS;
  channelConfig.dma_frame_num = MIC_CHUNK_SAMPLES;
  if (i2s_new_channel(&channelConfig, nullptr, &rxChannel) != ESP_OK) return false;

  // 2. Формат шины. INMP441 работает в стандартном формате I2S (Philips): слот 32 бита,
  //    два слота на период WS (стерео). Отсюда SCK = 16 000 × 2 × 32 = 1,024 МГц.
  i2s_std_config_t stdConfig = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,  // INMP441 не нужна отдельная опорная частота
      .bclk = PIN_MIC_SCK,
      .ws = PIN_MIC_WS,
      .dout = I2S_GPIO_UNUSED,  // ничего не передаём, только принимаем
      .din = PIN_MIC_SD,
      .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
    },
  };
  if (i2s_channel_init_std_mode(rxChannel, &stdConfig) != ESP_OK) return false;

  // 3. Подтяжка линии данных к земле. В правом полуслоте микрофон отпускает линию SD,
  //    и без подтяжки там читался бы случайный мусор.
  gpio_pulldown_en(PIN_MIC_SD);

  // 4. Подписаться на переполнение DMA (для счётчика потерь) и запустить приём
  i2s_event_callbacks_t callbacks = {};
  callbacks.on_recv_q_ovf = onDmaOverflow;
  i2s_channel_register_event_callback(rxChannel, &callbacks, nullptr);
  if (i2s_channel_enable(rxChannel) != ESP_OK) return false;

  // 5. Первые ~85 мс после появления тактов INMP441 выдаёт нули, а фильтру нужно
  //    «устояться». Выбрасываем 0,3 с, чтобы в начале первой записи не было щелчка.
  int16_t discard[MIC_CHUNK_SAMPLES];
  const uint32_t startMs = millis();
  while (millis() - startMs < 300) micRead(discard);
  return true;
}

size_t micRead(int16_t (&out)[MIC_CHUNK_SAMPLES]) {
  // Ждём один заполненный DMA-буфер. Он наполняется за 30 мс, тайм-аут 100 мс с запасом.
  size_t bytesRead = 0;
  if (i2s_channel_read(rxChannel, frames, sizeof(frames), &bytesRead, 100) != ESP_OK) return 0;

  const size_t count = bytesRead / (2 * sizeof(int32_t));  // 2 слова (левое и правое) на отсчёт
  for (size_t i = 0; i < count; i++) {
    out[i] = toWavSample(frames[2 * i]);  // [2·i] — левый полуслот, там данные микрофона
  }
  return count;
}

uint32_t micLostBuffers() { return lostBuffers; }
