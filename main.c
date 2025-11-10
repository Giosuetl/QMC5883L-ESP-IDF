#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <math.h>
#define QMC5883L_ADDRESS    0x0D // Dirección I2C del QMC5883L

#define QMC5883L_REG_OUT_X_LSB  0x00
#define QMC5883L_REG_OUT_X_MSB  0x01
#define QMC5883L_REG_OUT_Y_LSB  0x02
#define QMC5883L_REG_OUT_Y_MSB  0x03
#define QMC5883L_REG_OUT_Z_LSB  0x04
#define QMC5883L_REG_OUT_Z_MSB  0x05
#define QMC5883L_REG_STATUS     0x06
#define QMC5883L_REG_CONFIG_1   0x09
#define QMC5883L_REG_CONFIG_2   0x0A

#define REG_XOUT_L 0x00
#define REG_XOUT_H 0x01
#define REG_YOUT_L 0x02
#define REG_YOUT_H 0x03
#define REG_ZOUT_L 0x04
#define REG_ZOUT_H 0x05
#define REG_STATE  0x06
#define REG_TOUT_L 0x07
#define REG_TOUT_H 0x08
#define REG_CTRL1  0x09
#define REG_CTRL2  0x0a
#define REG_FBR    0x0b
#define REG_ID     0x0d

#define MASK_MODE  0xfe
#define MASK_ODR   0xf3

// Ejemplo para ESP-IDF (I2C master). Ajusta QMC5883L_ADDRESS y macros según tu proyecto.
   // Control1: OSR=64 (11), RNG=8G (01), ODR=200Hz (11), MODE=Continuous (01) DD
       // Control2: leave default (INT_ENB=0, POL_PNT=0, SOFT_RST=0)
#define QMC5883L_REG_SETRESET 0x0B
// OSR=512 (00), RNG=2G (00), ODR=10Hz (00), MODE=Continuous (01) 1D
bool qmc5883l_init() {
    esp_err_t ret;
    
    // Configuración recomendada según datasheet
    const uint8_t config1 = 0xDD; 
    const uint8_t config2 = 0x00; // Default
    const uint8_t setreset = 0x01;

    // Verificar comunicación con el sensor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        printf("Error: No se puede comunicar con QMC5883L\n");
        return false;
    }

    // Reset suave primero
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, QMC5883L_REG_CONFIG_2, true);
    i2c_master_write_byte(cmd, 0x80, true); // SOFT_RST = 1
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    vTaskDelay(50 / portTICK_PERIOD_MS); // Delay después del reset

    // Configurar SET/RESET
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, QMC5883L_REG_SETRESET, true);
    i2c_master_write_byte(cmd, setreset, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        printf("Error configurando SET/RESET\n");
        return false;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Configurar Control Register 1
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, QMC5883L_REG_CONFIG_1, true);
    i2c_master_write_byte(cmd, config1, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        printf("Error configurando CTRL1\n");
        return false;
    }

    printf("QMC5883L inicializado correctamente\n");
    return true;
}


void qmc5883l_read(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];

    // Leer datos del sensor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, QMC5883L_REG_OUT_X_LSB, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    float f = ( 8000.0) / 32768;
    // Convertir los datos a valores de 16 bits
    *x = (int16_t)(data[0] | (data[1] << 8))*f;
    *y = (int16_t)(data[2] | (data[3] << 8))*f;
    *z = (int16_t)(data[4] | (data[5] << 8))*f;
}
bool qmc5883l_data_ready() {
    uint8_t status = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, QMC5883L_REG_STATUS, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &status, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return (status & 0x01); // DRDY = bit0
}

double get_heading(int16_t x, int16_t y) {
    double heading = atan2((double)y, (double)x) * 180.0 / M_PI;
    if (heading < 0) {
        heading += 360.0;
    }
    return heading;
}

double get_heading_compensated(int16_t mx, int16_t my, int16_t mz, 
                              float roll_rad, float pitch_rad) {
    // Convertir a valores flotantes
    float x = (float)mx;
    float y = (float)my;
    float z = (float)mz;
    
    // Compensación por tilt (roll y pitch)
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);
    
    // Rotar el vector magnético para compensar inclinación
    float x_comp = x * cos_pitch + z * sin_pitch;
    float y_comp = x * sin_roll * sin_pitch + y * cos_roll - z * sin_roll * cos_pitch;
    
    // Calcular heading (0° = Norte, 90° = Este)
    double heading = atan2(-y_comp, x_comp) * 180.0 / M_PI;
    
    // Normalizar a 0-360°
    if (heading < 0) {
        heading += 360.0;
    }
    
    return heading;
}

void qmc5883l_task(void *pvParameters) {
    int16_t x, y, z;

    qmc5883l_init();

    while (1) {
        qmc5883l_read(&x, &y, &z);
        printf("Magnetometer Data: X=%d, Y=%d, Z=%d\n", x, y, z);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay de 1 segundo
    }
}
