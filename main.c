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

void qmc5883l_init() {


   // Configurar el QMC5883L
    uint8_t data1[] = {0x01, 0x00}; // Modo de operación continuo
    uint8_t data2[] = {0x01, 0x20}; // Ganancia magnética: 8G
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (QMC5883L_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, QMC5883L_REG_CONFIG_1, true);
    i2c_master_write(cmd, data1, sizeof(data1), true);
    i2c_master_write_byte(cmd, QMC5883L_REG_CONFIG_2, true);
    i2c_master_write(cmd, data2, sizeof(data2), true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
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

    // Convertir los datos a valores de 16 bits
    *x = (int16_t)(data[0] | (data[1] << 8));
    *y = (int16_t)(data[2] | (data[3] << 8));
    *z = (int16_t)(data[4] | (data[5] << 8));
}
double get_heading(int16_t x, int16_t y) {
    double heading = atan2((double)y, (double)x) * 180.0 / M_PI;
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

void app_main() {
    xTaskCreate(qmc5883l_task, "qmc5883l_task", 2048, NULL, 5, NULL);
}
