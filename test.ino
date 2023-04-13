#include <HID.h>
#include <HardwareSerial.h>

#define JOYSTICK_REPORT_ID 1
#define CRSF_PACKET_SIZE 28

// HID Joystick 类
class Joystick : public HIDJoystick {
public:
  Joystick() : HIDJoystick(JOYSTICK_REPORT_ID) {}

  // 发送 Joystick 状态信息
  void sendState() {
    joyState.xAxis = map(joystickState[0], 0, 4095, -512, 511);
    joyState.yAxis = map(joystickState[1], 0, 4095, -512, 511);
    joyState.zAxis = map(joystickState[2], 0, 4095, -512, 511);
    joyState.zAxis2 = map(joystickState[3], 0, 4095, -512, 511);
    HID().SendReport(JOYSTICK_REPORT_ID, &joyState, sizeof(joyState));
  }

  // 将 Joystick 状态信息封装为 CRSF 数据包
  void sendCRSF() {
    uint8_t packet[CRSF_PACKET_SIZE] = {0};
    packet[0] = 0xC8; // 帧头
    packet[1] = 0xC0; // 帧类型
    packet[2] = 0x1C; // 数据长度
    packet[3] = 0x01; // 设备 ID
    packet[4] = 0x10; // 附加标识
    packet[5] = 0x01; // 数据 ID
    packet[6] = (joyState.xAxis >> 8) & 0xFF; // x 轴数据高位
    packet[7] = joyState.xAxis & 0xFF; // x 轴数据低位
    packet[8] = (joyState.yAxis >> 8) & 0xFF; // y 轴数据高位
    packet[9] = joyState.yAxis & 0xFF; // y 轴数据低位
    packet[10] = (joyState.zAxis >> 8) & 0xFF; // z 轴数据高位
    packet[11] = joyState.zAxis & 0xFF; // z 轴数据低位
    packet[12] = (joyState.zAxis2 >> 8) & 0xFF; // z2 轴数据高位
    packet[13] = joyState.zAxis2 & 0xFF; // z2 轴数据低位
    uint16_t crc = crc16_ccitt(packet, CRSF_PACKET_SIZE - 2);
    packet[CRSF_PACKET_SIZE - 2] = crc & 0xFF; // CRC 校验低位
    packet[CRSF_PACKET_SIZE - 1] = (crc >> 8) & 0xFF; // CRC 校验高位
    Serial.write(packet, CRSF_PACKET_SIZE);
  }

private:
  JoystickReport joyState;
};

// 自定义的 USB 设备
USBHID HID(1);
Joystick joystick;

// ADC 采样
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
  // 初始化 UART
  Serial.begin(115200);

  // 启动 USB
  HID().begin();

  // 启动 DMA 采样
  dmaStart();
}

void loop() {
  // 发送 Joystick 状态信息
  joystick.sendState();

  // 将 Joystick 状态信息封装为 CRSF 数据包并输出到 UART 端口
  joystick.sendCRSF();

  //
  // 延迟以实现报告刷新率
  delay(1);
}
