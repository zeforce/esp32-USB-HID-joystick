#include <HardwareSerial.h>

#define CRSF_PACKET_SIZE 28

// ADC 引脚定义
const int adcPins[] = {34, 35, 36, 39};

// UART 配置
HardwareSerial SerialCRSF(1);
const int CRSF_BAUDRATE = 420000;
const int CRSF_TX_PIN = 26;
const int CRSF_RX_PIN = 27;

// 将 ADC 采集到的四个轴的值封装为 CRSF 数据包并输出到 UART
void sendCRSF(int x, int y, int z, int t) {
  uint8_t packet[CRSF_PACKET_SIZE] = {0};
  packet[0] = 0xC8; // 帧头
  packet[1] = 0xC0; // 帧类型
  packet[2] = 0x1C; // 数据长度
  packet[3] = 0x01; // 设备 ID
  packet[4] = 0x10; // 附加标识
  packet[5] = 0x01; // 数据 ID
  packet[6] = (x >> 8) & 0xFF; // x 轴数据高位
  packet[7] = x & 0xFF; // x 轴数据低位
  packet[8] = (y >> 8) & 0xFF; // y 轴数据高位
  packet[9] = y & 0xFF; // y 轴数据低位
  packet[10] = (z >> 8) & 0xFF; // z 轴数据高位
  packet[11] = z & 0xFF; // z 轴数据低位
  packet[12] = (t >> 8) & 0xFF; // t 轴数据高位
  packet[13] = t & 0xFF; // t 轴数据低位
  uint16_t crc = crc16_ccitt(packet, CRSF_PACKET_SIZE - 2);
  packet[CRSF_PACKET_SIZE - 2] = crc & 0xFF; // CRC 校验低位
  packet[CRSF_PACKET_SIZE - 1] = (crc >> 8) & 0xFF; // CRC 校验高位
  SerialCRSF.write(packet, CRSF_PACKET_SIZE);
}

void dmaInit() {
  // 配置 ADC 通道
  for (int i = 0; i < 4; i++) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten((adc1_channel_t)adcPins[i], ADC_ATTEN_DB_11);
  }

  // 配置 DMA
  adc1_dma_enable();
  periph_module_enable(PERIPH_LEDC_MODULE);
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .bit_num = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&timer_conf);
  ledc_channel_config_t ch_conf =
  // 配置 DMA 描述符
  const int dmaSize = 4;
  const int dmaPrio = 1;
  const int dmaNum = 1;
  static DRAM_ATTR lldesc_t dmaDesc[dmaNum];
  for (int i = 0; i < dmaNum; i++) {
    dmaDesc[i].size = dmaSize * 2;
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

  // 初始化 DMA
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
  // 初始化 UART CRSF
  SerialCRSF.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);

  // 初始化 DMA 采样
  dmaInit();
}

void loop() {
  // 读取 ADC 采样结果并转换为 Joystick 轴数据
  int x = map(joystickState[0], 0, 4095, -512, 511);
  int y = map(joystickState[1], 0, 4095, -512, 511);
  int z = map(joystickState[2], 0, 4095, -512, 511);
  int t = map(joystickState[3], 0, 4095, -512, 511);

  // 将 Joystick 轴数据封装为 CRSF 数据包并输出到 UART
  sendCRSF(x, y, z, t);

  // 延迟以实现报告刷新率
  delay(1);
}
