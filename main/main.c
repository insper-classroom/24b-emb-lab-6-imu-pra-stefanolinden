#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include <Fusion.h>

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
#define SAMPLE_PERIOD (0.01f)

QueueHandle_t xQueueData;

typedef struct {
    int axis;
    int value;
} imu_data_t;

static void mpu6050_init() {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Reseta o MPU6050
    uint8_t buf[] = {0x6B, 0x00}; // Reset
    int result = i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
    if (result == PICO_ERROR_GENERIC) {
        printf("Erro ao inicializar MPU6050!\n");
    }
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;

    // Lê os dados de aceleração
    if (i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true) == PICO_ERROR_GENERIC) {
        printf("Erro ao escrever no MPU6050\n");
        return;
    }
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Lê os dados do giroscópio
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Lê a temperatura
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = buffer[0] << 8 | buffer[1];
}

void data_task(void *p) {
    imu_data_t data;

    while (1) {
        if (xQueueReceive(xQueueData, &data, portMAX_DELAY) == pdPASS) {
            // Enviar os dados pela UART
            uart_putc(UART0_BASE, data.axis);
            uart_putc(UART0_BASE, (data.value >> 8) & 0xFF);
            uart_putc(UART0_BASE, data.value & 0xFF);
            uart_putc(UART0_BASE, 0xFF); // Byte de fim de pacote
        }
    }
}

void mpu6050_task(void *p) {
    mpu6050_init();

    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    while (1) {
        // Lê os dados da IMU
        mpu6050_read_raw(acceleration, gyro, &temp);


        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f,
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };
        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        imu_data_t data;
        data.axis = 0;
        data.value = (int)euler.angle.yaw;
        xQueueSend(xQueueData, &data, portMAX_DELAY);

        data.axis = 1;
        data.value = (int)euler.angle.roll;
        xQueueSend(xQueueData, &data, portMAX_DELAY);


        if (accelerometer.axis.y > 1.5f) {
                data.axis = 2;
            data.value = 1;
            xQueueSend(xQueueData, &data, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Atraso para a próxima leitura
    }
}

int main() {
    stdio_init_all();
    xQueueData = xQueueCreate(10, sizeof(imu_data_t));  // Fila para armazenar os dados do IMU
    xTaskCreate(mpu6050_task, "mpu6050_task", 8192, NULL, 1, NULL);
    xTaskCreate(data_task, "data_task", 1024, NULL, 1, NULL);
    vTaskStartScheduler();
    while (true);
}
