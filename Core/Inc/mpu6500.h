#ifndef __MPU6500_H
#define __MPU6500_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define MPU6500_I2C_ADDR_7BIT_1   0x68U
#define MPU6500_I2C_ADDR_7BIT_2   0x69U
#define MPU6500_I2C_ADDR_8BIT_1   (MPU6500_I2C_ADDR_7BIT_1 << 1)
#define MPU6500_I2C_ADDR_8BIT_2   (MPU6500_I2C_ADDR_7BIT_2 << 1)
#define MPU6500_REG_WHO_AM_I      0x75U
#define MPU6500_REG_PWR_MGMT_1    0x6BU
#define MPU6500_REG_GYRO_CONFIG   0x1BU
#define MPU6500_REG_GYRO_XOUT_H   0x43U
#define MPU6500_REG_CONFIG        0x1AU  /* 配置寄存器 */
#define MPU6500_REG_SMPLRT_DIV    0x19U  /* 采样率分频 */

/* PWR_MGMT_1 寄存器位定义 */
#define MPU6500_DEVICE_RESET      0x80U  /* 设备复位 */
#define MPU6500_CLKSEL_PLL_XGYRO  0x01U  /* 时钟源：X轴陀螺仪PLL */
#define MPU6500_WHO_AM_I_VALUE    0x70U
#define MPU9250_WHO_AM_I_VALUE    0x71U
#define MPU6500_GYRO_LSB_PER_DPS  131.0f

typedef struct
{
  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z;
  float dps_x;
  float dps_y;
  float dps_z;
  bool data_ready;
  bool who_am_i_ok;
} MPU6500GyroData;

typedef struct
{
  float pitch;
  float roll;
  float yaw;
} MPU6500EulerAngles;

/* I2C 错误类型统计 */
typedef struct
{
  uint32_t timeout;      /* 超时错误计数 */
  uint32_t nack;         /* NACK 错误计数 */
  uint32_t bus_error;    /* 总线错误计数 */
  uint32_t other_error;  /* 其他错误计数 */
} MPU6500ErrorStats;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint8_t i2c_addr_8bit;
  uint8_t who_am_i;
  uint8_t gyro_raw_bytes[6];
  uint32_t dma_read_count;
  uint32_t total_error_count;
  MPU6500ErrorStats error_stats;  /* 分类错误统计 */
  MPU6500GyroData gyro;
  MPU6500EulerAngles euler;
  bool dmp_initialized;
  /* I2C 总线锁回调（用于与 OLED 共享 I2C1） */
  void (*i2c_lock)(void);
  void (*i2c_unlock)(void);
  void (*i2c_unlock_from_isr)(void);
} MPU6500Context;

void MPU6500_Init(MPU6500Context *ctx, I2C_HandleTypeDef *hi2c);
void MPU6500_SetI2CLockCallbacks(MPU6500Context *ctx,
                                  void (*lock_fn)(void),
                                  void (*unlock_fn)(void),
                                  void (*unlock_isr_fn)(void));
HAL_StatusTypeDef MPU6500_Configure(MPU6500Context *ctx);
HAL_StatusTypeDef MPU6500_ReadWhoAmI(MPU6500Context *ctx);
HAL_StatusTypeDef MPU6500_StartGyroReadDMA(MPU6500Context *ctx);
void MPU6500_OnGyroReadComplete(MPU6500Context *ctx);

HAL_StatusTypeDef MPU6500_InitDMP(MPU6500Context *ctx);
void MPU6500_UpdateYaw(MPU6500Context *ctx);
float MPU6500_GetYaw(MPU6500Context *ctx);
void MPU6500_ResetGyroData(MPU6500Context *ctx);
void MPU6500_DiagnosticPrint(MPU6500Context *ctx, void (*debug_print)(const char *));
void MPU6500_ResetErrorStats(MPU6500Context *ctx);
/* ISR 安全版本：从中断回调中调用 */
void MPU6500_UpdateErrorStatsFromISR(MPU6500Context *ctx, HAL_StatusTypeDef status);

/* 诊断函数 */
void MPU6500_ScanI2CBus(I2C_HandleTypeDef *hi2c, void (*debug_print)(const char *));

#ifdef __cplusplus
}
#endif

#endif
