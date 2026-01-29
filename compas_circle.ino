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

/* Inserta un valor en el buffer.
   Si está “lleno” (capacidad efectiva LEN-1), sobrescribe el más antiguo. */
void value_queue(circular_buffer *buf, float value) {
    if (!buf) return;

    buf->values[buf->head] = value;
    int next_head = CIRC_NEXT(buf->head);

    // Si al avanzar la cabeza alcanzamos la cola, movemos la cola (descartamos el más antiguo)
    if (next_head == buf->tail) {
        buf->tail = CIRC_NEXT(buf->tail);
    }
    buf->head = next_head;
}

/* Devuelve la media de los valores actualmente almacenados.
   Si está vacío, devuelve 0.0f. */
float value_average(const circular_buffer *buf) {
    if (!buf) return 0.0f;

    if (buf->head == buf->tail) {
        // Vacío
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
                             // 储存补偿磁强计浮子数据
bmm150_mag_data mag_max;
bmm150_mag_data mag_min;
TFT_eSprite img = TFT_eSprite(&M5.Lcd);

int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data,
                uint16_t len) {
    if (M5.I2C.readBytes(
            dev_id, reg_addr, len,
            read_data)) {  // Check whether the device ID, address, data exist.
        return BMM150_OK;  //判断器件的Id、地址、数据是否存在
    } else {
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *read_data,
                 uint16_t len) {
    if (M5.I2C.writeBytes(dev_id, reg_addr, read_data,
                          len)) {  // Writes data of length len to the specified
                                   // device address.
        return BMM150_OK;  //向指定器件地址写入长度为len的数据
    } else {
        return BMM150_E_DEV_NOT_FOUND;
    }
}

int8_t bmm150_initialization() {
    int8_t rslt = BMM150_OK;

    dev.dev_id = 0x10;           // Device address setting.  设备地址设置
    dev.intf = BMM150_I2C_INTF;  // SPI or I2C interface setup. SPI或I2C接口设置
    dev.read     = i2c_read;   // Read the bus pointer.  读总线指针
    dev.write    = i2c_write;  // Write the bus pointer.  写总线指针
    dev.delay_ms = delay;

    // Set the maximum range range
    //设置最大范围区间
    mag_max.x = -2000;
    mag_max.y = -2000;
    mag_max.z = -2000;

    // Set the minimum range
    //设置最小范围区间
    mag_min.x = 2000;
    mag_min.y = 2000;
    mag_min.z = 2000;

    rslt = bmm150_init(&dev);  // Memory chip ID.  存储芯片ID
    dev.settings.pwr_mode = BMM150_NORMAL_MODE;
    rslt |= bmm150_set_op_mode(
        &dev);  // Set the sensor power mode.  设置传感器电源工作模式
    dev.settings.preset_mode = BMM150_PRESETMODE_ENHANCED;
    rslt |= bmm150_set_presetmode(
        &dev);  // Set the preset mode of .  设置传感器的预置模式
    return rslt;
}

void bmm150_offset_save() {  // Store the data.  存储bmm150的数据
    prefs.begin("bmm150", false);
    prefs.putBytes("offset", (uint8_t *)&mag_offset, sizeof(bmm150_mag_data));
    prefs.end();
}

void bmm150_offset_load() {  // load the data.  加载bmm150的数据
    if (prefs.begin("bmm150", true)) {
        prefs.getBytes("offset", (uint8_t *)&mag_offset,
                       sizeof(bmm150_mag_data));
        prefs.end();
        Serial.println("bmm150 load offset finish....");
    } else {
        Serial.println("bmm150 load offset failed....");
    }
}

void setup() {
    M5.begin(true, false, true,
             false);   // Init M5Core(Initialize LCD, serial port).  初始化
                       // M5Core（初始化LCD、串口）
    M5.Power.begin();  // Init Power module.  初始化电源设置
    Wire.begin(
        21, 22,
        400000UL);  // Set the frequency of the SDA SCL.  设置SDA和SCL的频率

    // === Cambios de color/ fondo para círculo blanco y letras verdes ===
    img.setColorDepth(8);                 // colores reales
    img.setTextColor(TFT_WHITE);
    img.createSprite(320, 240);
    img.setBitmapColor(TFT_WHITE, TFT_BLACK);  // fondo negro
    // ===================================================================

    if (bmm150_initialization() != BMM150_OK) {
        img.fillSprite(0);  // Fill the whole sprite with defined colour.
                            // 用定义的颜色填充整个Sprite图
        img.drawCentreString("BMM150 init failed", 160, 110,
                             4);  // Use font 4 in (160,110)draw string.
                                  // 使用字体4在(160,110)处绘制字符串
        img.pushSprite(
            0,
            0);  // Push the sprite to the TFT at 0, 0.  将Sprite图打印在(0,0)处
        for (;;) {
            delay(100);  // delay 100ms.  延迟100ms
        }
    }

    bmm150_offset_load();
}

void bmm150_calibrate(
    uint32_t calibrate_time) {  // bbm150 data calibrate.  bbm150数据校准
    uint32_t calibrate_timeout = 0;

    calibrate_timeout = millis() + calibrate_time;
    Serial.printf("Go calibrate, use %d ms \r\n",
                  calibrate_time);  // The serial port outputs formatting
                                    // characters.  串口输出格式化字符
    Serial.printf("running ...");

    while (calibrate_timeout > millis()) {
        bmm150_read_mag_data(&dev);  // read the magnetometer data from
                                     // registers.  从寄存器读取磁力计数据
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
void loop() {
    M5.update();  // Read the press state of the key.  读取按键的状态
    bmm150_read_mag_data(&dev);
    float head_dir =
        atan2(dev.data.x - mag_offset.x, dev.data.y - mag_offset.y) * 180.0 /
        M_PI;
    Serial.printf("Magnetometer data, heading %.2f\n", head_dir);
    Serial.printf("MAG X : %.2f \t MAG Y : %.2f \t MAG Z : %.2f \n", dev.data.x,
                  dev.data.y, dev.data.z);
    Serial.printf("MID X : %.2f \t MID Y : %.2f \t MID Z : %.2f \n",
                  mag_offset.x, mag_offset.y, mag_offset.z);
    if(ultima_direccion <-100 && head_dir >100){
      value_clear(&buffer);
    }
    if(ultima_direccion >100 && head_dir <-100){
      value_clear(&buffer);
    }
    value_queue(&buffer, head_dir);
    ultima_direccion=head_dir;
    head_dir = value_average(&buffer);

    // =================== DIBUJO ESTILO REFERENCIA ===================
    img.fillSprite(TFT_BLACK);                         // limpia fondo

    // Disco blanco y punto central verde
    img.fillCircle(160, 120, 90, TFT_WHITE);
    img.fillCircle(160, 120, 1,  TFT_GREEN);

    // Cardinales en verde (Oeste con "O")
    img.setTextColor(TFT_GREEN, TFT_BLACK);
    img.drawCentreString("N", 160, 120 - 90 - 18, 4);
    img.drawCentreString("S", 160, 120 + 90 + 6,  4);
    img.drawCentreString("O", 160 - 90 - 16, 120 - 10, 4);
    img.drawCentreString("E", 160 + 90 + 16, 120 - 10, 4);

    // Punto rojo: 0°=N, 90°=E (convertimos a coords de pantalla)
    float rad = (90.0f - head_dir) * DEG_TO_RAD;
    int punto_x = 160 + (int)(75 * cos(rad));
    int punto_y = 120 - (int)(75 * sin(rad));
    img.fillCircle(punto_x, punto_y, 8, TFT_RED);

    // (Opcional) lectura en rojo como en la captura
    img.setTextColor(TFT_RED, TFT_BLACK);
    img.drawString(String(head_dir, 1), 8, 215, 4);

    img.pushSprite(0,0);
    // ================================================================

    if (M5.BtnA.wasPressed()) {
        img.fillSprite(0);
        img.drawCentreString("Flip + rotate core calibration", 160, 110, 4);
        img.pushSprite(0, 0);
        bmm150_calibrate(10000);
    }

    delay(100);
}
