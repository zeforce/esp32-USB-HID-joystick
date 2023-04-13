#include <HID.h>

#define JOYSTICK_REPORT_ID 1

// HID Joystick 类
class Joystick : public HIDJoystick {
public:
  Joystick() : HIDJoystick(JOYSTICK_REPORT_ID) {}

  // 发送 Joystick 状态信息
  void sendState() {
    joyState.xAxis = map(joystickState[0], 0, 4095, -512, 511);
    joyState.yAxis = map(joystickState[1], 0, 4095, -512, 511);
    joyState.yAxis2 = map(joystickState[2], 0, 4095, -512, 511);
    joyState.zAxis = map(joystickState[3], 0, 4095, -512, 511);
    HID().SendReport(JOYSTICK_REPORT_ID, &joyState, sizeof(joyState));
  }

private:
  JoystickReport joyState;
};

// 自定义的 USB 设备
USBHID HID(1);
Joystick joystick;

// DMA 采样配置
const int adcPins[] = {34, 35, 36, 39}; // 四个 ADC 引脚
const int dmaChannel = 0;
const int dmaSize = 4;
const int dmaPrio = 1;
const int dmaNum = 1;
static DRAM_ATTR lldesc_t dmaDesc[dmaNum];

void dmaInit() {
  for (int i = 0; i < 4; i++) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten((adc1_channel_t)adcPins[i], ADC_ATTEN_DB_11);
  }
  adc1_dma_enable();

  for (int i = 0; i < dmaNum; i++) {
    dmaDesc[i].size = dmaSize * 2; // 采样结果为 12 位，因此需要 *2
    dmaDesc[i].length = dmaSize;
    dmaDesc[i].buf = (uint8_t *)heap_caps_malloc(dmaDesc[i].size, MALLOC_CAP_DMA);
    dmaDesc[i].eof = 1;
    dmaDesc[i].owner = 1;
    dmaDesc[i].sosf = 0;
    dmaDesc[i].buf = &joystickState;
    dmaDesc[i].empty = &dmaDesc[(i + 1) % dmaNum];
    dmaDesc[i].next = NULL;
  }
  dmaDesc[dmaNum - 1].empty = &dmaDesc[0];

  periph_module_enable(PERIPH_LEDC_MODULE);

  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .bit_num = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t ch_conf = {
    .channel = LEDC_CHANNEL_0,
    .duty = 0,
    .gpio_num = -1,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_sel = LEDC_TIMER_0,
    .hpoint = 0,
    .flags = LEDC_INTR_DISABLE,
  };
  ledc_channel_config(&ch_conf);

  ledc_fade_func_install(0);
  ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
  ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
}

void dmaStart() {
  dmaInit();

  int rc = dma_desc_init(dmaChannel, &dmaDesc[0], dmaNum);
  if (rc != ESP_OK) {
    Serial.println("Failed to initialize ADC DMA");
  }

  rc = dma_start(dmaChannel, &dmaDesc[0], dmaNum);
  if (rc != ESP_OK) {
    Serial.println("Failed to start ADC DMA");
  }
}

void setup() {
  // 启动 USB
  HID().begin();

  // 启动 DMA 采样
  dmaStart();
}

void loop() {
  // 发送 Joystick 状态信息
  joystick.sendState();

  // 延迟以实现报告刷新率
  delay(1);
}
