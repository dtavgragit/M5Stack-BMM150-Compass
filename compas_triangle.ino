/*
*******************************************************************************
* Copyright (c) 2023 by M5Stack
*                  Equipped with M5Core sample source code
*                          配套  M5Core 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/gray
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/gray
*
* Describe: bmm150--Magnetometer 三轴磁力计
* Date: 2021/7/21
*******************************************************************************
*/

#include <Arduino.h>

#include "M5Stack.h"
#include "M5_BMM150.h"
#include "M5_BMM150_DEFS.h"
#include "Preferences.h"
#include "math.h"
#include <stddef.h>  // size_t
#include <stdbool.h> // bool

#define CIRC_NEXT(idx)   (((idx) + 1) % CIRCULAR_BUFFER_LEN)
#define CIRCULAR_BUFFER_LEN 4

typedef struct {
  int head;
  int tail;
  float values[CIRCULAR_BUFFER_LEN];
} circular_buffer;

/* Vacía el buffer: índices a 0 y valores a 0.0f (opcional pero útil). */
void value_clear(circular_buffer *buf) {
  if (!buf) return;
  buf->head = 0;
  buf->tail = 0;
  for (int i = 0; i < CIRCULAR_BUFFER_LEN; ++i) {
    buf->values[i] = 0.0f;
  }
}

/* Inserta un valor en el buffer. Si está “lleno” (capacidad efectiva LEN-1), sobrescribe el más antiguo. */
void value_queue(circular_buffer *buf, float value) {
  if (!buf) return;
  buf->values[buf->head] = value;
  int next_head = CIRC_NEXT(buf->head);
  if (next_head == buf->tail) {
    buf->tail = CIRC_NEXT(buf->tail);
  }
  buf->head = next_head;
}

/* Devuelve la media de los valores actualmente almacenados. Si está vacío, devuelve 0.0f. */
float value_average(const circular_buffer *buf) {
  if (!buf) return 0.0f;
  if (buf->head == buf->tail) {
    return 0.0f;
  }
  float sum = 0.0f;
  int count = 0;
  for (int i = buf->tail; i != buf->head; i = CIRC_NEXT(i)) {
    sum += buf->values[i];
    ++count;
  }
  return (count > 0) ? (sum / (float)count) : 0.0f;
}

circular_buffer buffer;
Preferences prefs;

struct bmm150_dev dev;
bmm150_mag_data mag_offset;  // Compensation magnetometer float data storage
bmm150_mag_data mag_max;
bmm150_mag_data mag_min;
TFT_eSprite img = TFT_eSprite(&M5.Lcd);

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data,
                uint16_t len) {
  if (M5.I2C.readBytes(dev_id, reg_addr, len, read_data)) {
    return BMM150_OK;
  } else {
    return BMM150_E_DEV_NOT_FOUND;
  }
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data,
                 uint16_t len) {
  if (M5.I2C.writeBytes(dev_id, reg_addr, read_data, len)) {
    return BMM150_OK;
  } else {
    return BMM150_E_DEV_NOT_FOUND;
  }
}

int8_t bmm150_initialization() {
  int8_t rslt = BMM150_OK;

  dev.dev_id = 0x10;
  dev.intf = BMM150_I2C_INTF;
  dev.read     = i2c_read;
  dev.write    = i2c_write;
  dev.delay_ms = delay;

  mag_max.x = -2000;
  mag_max.y = -2000;
  mag_max.z = -2000;

  mag_min.x = 2000;
  mag_min.y = 2000;
  mag_min.z = 2000;

  rslt = bmm150_init(&dev);
  dev.settings.pwr_mode = BMM150_NORMAL_MODE;
  rslt |= bmm150_set_op_mode(&dev);
  dev.settings.preset_mode = BMM150_PRESETMODE_ENHANCED;
  rslt |= bmm150_set_presetmode(&dev);
  return rslt;
}

void bmm150_offset_save() {
  prefs.begin("bmm150", false);
  prefs.putBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
  prefs.end();
}

void bmm150_offset_load() {
  if (prefs.begin("bmm150", true)) {
    prefs.getBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
    prefs.end();
    Serial.println("bmm150 load offset finish....");
  } else {
    Serial.println("bmm150 load offset failed....");
  }
}

void setup() {
  M5.begin(true, false, true, false);
  M5.Power.begin();
  Wire.begin(21, 22, 400000UL);

  // Colores reales y fondo negro
  img.setColorDepth(8);
  img.setTextColor(TFT_WHITE);
  img.createSprite(320, 240);
  img.setBitmapColor(TFT_WHITE, TFT_BLACK);

  if (bmm150_initialization() != BMM150_OK) {
    img.fillSprite(0);
    img.drawCentreString("BMM150 init failed", 160, 110, 4);
    img.pushSprite(0, 0);
    for (;;) { delay(100); }
  }

  bmm150_offset_load();
}

void bmm150_calibrate(uint32_t calibrate_time) {
  uint32_t calibrate_timeout = 0;

  calibrate_timeout = millis() + calibrate_time;
  Serial.printf("Go calibrate, use %d ms \r\n", calibrate_time);
  Serial.printf("running ...");

  while (calibrate_timeout > millis()) {
    bmm150_read_mag_data(&dev);
    if (dev.data.x) {
      mag_min.x = (dev.data.x < mag_min.x) ? dev.data.x : mag_min.x;
      mag_max.x = (dev.data.x > mag_max.x) ? dev.data.x : mag_max.x;
    }
    if (dev.data.y) {
      mag_max.y = (dev.data.y > mag_max.y) ? dev.data.y : mag_max.y;
      mag_min.y = (dev.data.y < mag_min.y) ? dev.data.y : mag_min.y;
    }
    if (dev.data.z) {
      mag_min.z = (dev.data.z < mag_min.z) ? dev.data.z : mag_min.z;
      mag_max.z = (dev.data.z > mag_max.z) ? dev.data.z : mag_max.z;
    }
    delay(100);
  }

  mag_offset.x = (mag_max.x + mag_min.x) / 2;
  mag_offset.y = (mag_max.y + mag_min.y) / 2;
  mag_offset.z = (mag_max.z + mag_min.z) / 2;
  bmm150_offset_save();

  Serial.printf("\n calibrate finish ... \r\n");
  Serial.printf("mag_max.x: %.2f x_min: %.2f \t", mag_max.x, mag_min.x);
  Serial.printf("y_max: %.2f y_min: %.2f \t", mag_max.y, mag_min.y);
  Serial.printf("z_max: %.2f z_min: %.2f \r\n", mag_max.z, mag_min.z);
}

float ultima_direccion;

/* =================== NUEVO: Aguja triangular =================== */
static inline void draw_compass_triangle(float head_dir_deg) {
  const int CX = 160, CY = 120;
  const int R_OUT = 90;        // radio del círculo
  const int R_TIP = 75;        // distancia del vértice
  const int R_BASE = 48;       // distancia del centro de la base
  const int BASE_HALF = 12;    // media anchura de la base

  // Fondo negro y dial
  img.fillSprite(TFT_BLACK);
  img.fillCircle(CX, CY, R_OUT, TFT_WHITE);
  img.fillCircle(CX, CY, 1, TFT_GREEN);

  // Cardinales
  img.setTextColor(TFT_GREEN, TFT_BLACK);
  img.drawCentreString("N", CX, CY - R_OUT - 18, 4);
  img.drawCentreString("S", CX, CY + R_OUT + 6,  4);
  img.drawCentreString("O", CX - R_OUT - 16, CY - 10, 4);
  img.drawCentreString("E", CX + R_OUT + 16, CY - 10, 4);

  // Dirección: 0° = Norte (arriba)
  float rad = (90.0f - head_dir_deg) * DEG_TO_RAD;
  float c = cosf(rad), s = sinf(rad);

  // Vértice (punta)
  int x_tip = CX + (int)(R_TIP  * c);
  int y_tip = CY - (int)(R_TIP  * s);

  // Centro de la base
  int x_base_c = CX + (int)(R_BASE * c);
  int y_base_c = CY - (int)(R_BASE * s);

  // Vector perpendicular (para los dos vértices de la base)
  // perpendicular a (c, -s) en coords pantalla es (s, c)
  int x_base_l = x_base_c + (int)( BASE_HALF * s);
  int y_base_l = y_base_c + (int)( BASE_HALF * c);
  int x_base_r = x_base_c - (int)( BASE_HALF * s);
  int y_base_r = y_base_c - (int)( BASE_HALF * c);

  // Triángulo rojo sólido
  img.fillTriangle(x_tip, y_tip, x_base_l, y_base_l, x_base_r, y_base_r, TFT_RED);

  // Lectura (opcional)
  img.setTextColor(TFT_RED, TFT_BLACK);
  img.drawString(String(head_dir_deg, 1), 8, 215, 4);

  img.pushSprite(0, 0);
}
/* =============================================================== */

void loop() {
  M5.update();
  bmm150_read_mag_data(&dev);

  float head_dir =
      atan2(dev.data.x - mag_offset.x, dev.data.y - mag_offset.y) * 180.0 / M_PI;

  Serial.printf("Magnetometer data, heading %.2f\n", head_dir);
  Serial.printf("MAG X : %.2f \t MAG Y : %.2f \t MAG Z : %.2f \n", dev.data.x,
                dev.data.y, dev.data.z);
  Serial.printf("MID X : %.2f \t MID Y : %.2f \t MID Z : %.2f \n",
                mag_offset.x, mag_offset.y, mag_offset.z);

  if (ultima_direccion < -100 && head_dir > 100)  value_clear(&buffer);
  if (ultima_direccion >  100 && head_dir < -100) value_clear(&buffer);

  value_queue(&buffer, head_dir);
  ultima_direccion = head_dir;
  head_dir = value_average(&buffer);

  /* ======== Sustituye el bloque antiguo por la aguja triangular ======== */
  draw_compass_triangle(head_dir);
  /* ===================================================================== */

  if (M5.BtnA.wasPressed()) {
    img.fillSprite(TFT_BLACK);
    img.drawCentreString("Flip + rotate core calibration", 160, 110, 4);
    img.pushSprite(0, 0);
    bmm150_calibrate(10000);
  }

  delay(100);
}
