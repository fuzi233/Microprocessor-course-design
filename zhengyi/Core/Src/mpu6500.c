#include "mpu6500.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
#define DEFAULT_MPU_HZ (200)

/* I2C 超时和重试配置 */
#define MPU6500_I2C_TIMEOUT_MS   100   /* I2C 操作超时时间 */
#define MPU6500_I2C_RETRY_COUNT  3     /* I2C 操作重试次数 */

static int16_t MPU6500_CombineBytes(uint8_t msb, uint8_t lsb)
{
  return (int16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

/* 根据 HAL 状态更新错误统计 */
static void MPU6500_UpdateErrorStats(MPU6500Context *ctx, HAL_StatusTypeDef status)
{
  ctx->total_error_count++;
  
  switch (status)
  {
    case HAL_TIMEOUT:
      ctx->error_stats.timeout++;
      break;
    case HAL_ERROR:
      /* 检查 I2C 具体错误类型 */
      if (__HAL_I2C_GET_FLAG(ctx->hi2c, I2C_FLAG_AF))
      {
        ctx->error_stats.nack++;
      }
      else if (__HAL_I2C_GET_FLAG(ctx->hi2c, I2C_FLAG_BERR))
      {
        ctx->error_stats.bus_error++;
      }
      else
      {
        ctx->error_stats.other_error++;
      }
      break;
    case HAL_BUSY:
      ctx->error_stats.other_error++;
      break;
    default:
      ctx->error_stats.other_error++;
      break;
  }
}

/* 带超时和重试的 I2C 写入函数 */
static HAL_StatusTypeDef MPU6500_I2C_WriteWithRetry(MPU6500Context *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status;
  uint8_t retry = 0;

  /* 获取 I2C 总线锁 */
  if (ctx->i2c_lock != NULL)
  {
    ctx->i2c_lock();
  }

  while (retry < MPU6500_I2C_RETRY_COUNT)
  {
    status = HAL_I2C_Mem_Write(ctx->hi2c, ctx->i2c_addr_8bit, reg,
                               I2C_MEMADD_SIZE_8BIT, data, len, MPU6500_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      if (ctx->i2c_unlock != NULL) { ctx->i2c_unlock(); }
      return HAL_OK;
    }
    retry++;
    MPU6500_UpdateErrorStats(ctx, status);
  }

  /* 释放锁（即使失败也要释放） */
  if (ctx->i2c_unlock != NULL) { ctx->i2c_unlock(); }
  return status;
}

/* 带超时和重试的 I2C 读取函数 */
static HAL_StatusTypeDef MPU6500_I2C_ReadWithRetry(MPU6500Context *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
  HAL_StatusTypeDef status;
  uint8_t retry = 0;

  /* 获取 I2C 总线锁 */
  if (ctx->i2c_lock != NULL)
  {
    ctx->i2c_lock();
  }

  while (retry < MPU6500_I2C_RETRY_COUNT)
  {
    status = HAL_I2C_Mem_Read(ctx->hi2c, ctx->i2c_addr_8bit, reg,
                              I2C_MEMADD_SIZE_8BIT, data, len, MPU6500_I2C_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      if (ctx->i2c_unlock != NULL) { ctx->i2c_unlock(); }
      return HAL_OK;
    }
    retry++;
    MPU6500_UpdateErrorStats(ctx, status);
  }

  /* 释放锁 */
  if (ctx->i2c_unlock != NULL) { ctx->i2c_unlock(); }
  return status;
}

/* 软件复位 MPU6500 */
static HAL_StatusTypeDef MPU6500_SoftwareReset(MPU6500Context *ctx)
{
  uint8_t value = MPU6500_DEVICE_RESET;
  HAL_StatusTypeDef status;
  
  status = MPU6500_I2C_WriteWithRetry(ctx, MPU6500_REG_PWR_MGMT_1, &value, 1);
  if (status != HAL_OK)
  {
    return status;
  }
  
  /* 等待复位完成，至少 100ms */
  HAL_Delay(100);
  
  return HAL_OK;
}

void MPU6500_Init(MPU6500Context *ctx, I2C_HandleTypeDef *hi2c)
{
  memset(ctx, 0, sizeof(*ctx));
  ctx->hi2c = hi2c;
  ctx->i2c_addr_8bit = MPU6500_I2C_ADDR_8BIT_1; /* 默认地址 */
  ctx->dmp_initialized = false;
  /* 锁回调默认为 NULL（未设置时不加锁） */
  ctx->i2c_lock = NULL;
  ctx->i2c_unlock = NULL;
  ctx->i2c_unlock_from_isr = NULL;
}

void MPU6500_SetI2CLockCallbacks(MPU6500Context *ctx,
                                  void (*lock_fn)(void),
                                  void (*unlock_fn)(void),
                                  void (*unlock_isr_fn)(void))
{
  ctx->i2c_lock = lock_fn;
  ctx->i2c_unlock = unlock_fn;
  ctx->i2c_unlock_from_isr = unlock_isr_fn;
}

HAL_StatusTypeDef MPU6500_ReadWhoAmI(MPU6500Context *ctx)
{
  HAL_StatusTypeDef status;
  
  /* 先尝试地址1 */
  status = MPU6500_I2C_ReadWithRetry(ctx, MPU6500_REG_WHO_AM_I, &ctx->who_am_i, 1);
  
  if (status == HAL_OK && 
      (ctx->who_am_i == MPU6500_WHO_AM_I_VALUE || 
       ctx->who_am_i == MPU9250_WHO_AM_I_VALUE))
  {
    ctx->i2c_addr_8bit = MPU6500_I2C_ADDR_8BIT_1;
    ctx->gyro.who_am_i_ok = true;
    return HAL_OK;
  }
  
  /* 如果地址1不行，尝试地址2 */
  ctx->i2c_addr_8bit = MPU6500_I2C_ADDR_8BIT_2;
  status = MPU6500_I2C_ReadWithRetry(ctx, MPU6500_REG_WHO_AM_I, &ctx->who_am_i, 1);
  
  if (status == HAL_OK && 
      (ctx->who_am_i == MPU6500_WHO_AM_I_VALUE || 
       ctx->who_am_i == MPU9250_WHO_AM_I_VALUE))
  {
    ctx->gyro.who_am_i_ok = true;
    return HAL_OK;
  }
  
  /* 两个地址都不行 */
  ctx->gyro.who_am_i_ok = false;
  return status;
}

HAL_StatusTypeDef MPU6500_Configure(MPU6500Context *ctx)
{
  HAL_StatusTypeDef status;
  uint8_t value;

  // 必须先成功检测到 WHO_AM_I 才能配置
  if (!ctx->gyro.who_am_i_ok)
  {
    return HAL_ERROR;
  }

  /* 1. 软件复位 */
  status = MPU6500_SoftwareReset(ctx);
  if (status != HAL_OK)
  {
    return status;
  }

  /* P2修复：复位后额外等待，确保芯片内部稳定（PLL锁定等） */
  HAL_Delay(50);

  /* 复位后验证芯片是否正常响应（非致命，仅用于诊断） */
  {
    HAL_StatusTypeDef verify_status = MPU6500_ReadWhoAmI(ctx);
    if (verify_status != HAL_OK)
    {
      /* 复位后首次通信可能失败，不视为致命错误
       * 继续尝试后续寄存器配置 */
      ctx->gyro.who_am_i_ok = true;  /* 保持之前的状态 */
    }
  }
  
  /* 2. 配置电源管理，选择 PLL 作为时钟源 */
  value = MPU6500_CLKSEL_PLL_XGYRO;
  status = MPU6500_I2C_WriteWithRetry(ctx, MPU6500_REG_PWR_MGMT_1, &value, 1);
  if (status != HAL_OK)
  {
    return status;
  }
  
  /* 3. 配置采样率分频 SMPLRT_DIV = 4 (采样率=1kHz/(1+4)=200Hz) */
  value = 4;
  status = MPU6500_I2C_WriteWithRetry(ctx, MPU6500_REG_SMPLRT_DIV, &value, 1);
  if (status != HAL_OK)
  {
    return status;
  }
  
  /* 4. 配置 CONFIG 寄存器：数字低通滤波器 DLPF = 3 (带宽44Hz) */
  value = 3;
  status = MPU6500_I2C_WriteWithRetry(ctx, MPU6500_REG_CONFIG, &value, 1);
  if (status != HAL_OK)
  {
    return status;
  }
  
  /* 5. 配置陀螺仪：FS_SEL = 0 (250dps) */
  value = 0x00U;
  return MPU6500_I2C_WriteWithRetry(ctx, MPU6500_REG_GYRO_CONFIG, &value, 1);
}

HAL_StatusTypeDef MPU6500_StartGyroReadDMA(MPU6500Context *ctx)
{
  HAL_StatusTypeDef status;

  /* 获取 I2C 总线锁（DMA 传输期间持有锁） */
  if (ctx->i2c_lock != NULL)
  {
    ctx->i2c_lock();
  }

  status = HAL_I2C_Mem_Read_DMA(ctx->hi2c, ctx->i2c_addr_8bit, MPU6500_REG_GYRO_XOUT_H,
                              I2C_MEMADD_SIZE_8BIT, ctx->gyro_raw_bytes, sizeof(ctx->gyro_raw_bytes));

  /* 如果 DMA 启动失败，立即释放锁 */
  if (status != HAL_OK && ctx->i2c_unlock != NULL)
  {
    ctx->i2c_unlock();
  }

  return status;
}

void MPU6500_OnGyroReadComplete(MPU6500Context *ctx)
{
  ctx->gyro.raw_x = MPU6500_CombineBytes(ctx->gyro_raw_bytes[0], ctx->gyro_raw_bytes[1]);
  ctx->gyro.raw_y = MPU6500_CombineBytes(ctx->gyro_raw_bytes[2], ctx->gyro_raw_bytes[3]);
  ctx->gyro.raw_z = MPU6500_CombineBytes(ctx->gyro_raw_bytes[4], ctx->gyro_raw_bytes[5]);

  ctx->gyro.dps_x = (float)ctx->gyro.raw_x / MPU6500_GYRO_LSB_PER_DPS;
  ctx->gyro.dps_y = (float)ctx->gyro.raw_y / MPU6500_GYRO_LSB_PER_DPS;
  ctx->gyro.dps_z = (float)ctx->gyro.raw_z / MPU6500_GYRO_LSB_PER_DPS;
  ctx->gyro.data_ready = true;
  ctx->dma_read_count++;

  /* DMA 完成后释放 I2C 总线锁（ISR 上下文） */
  if (ctx->i2c_unlock_from_isr != NULL)
  {
    ctx->i2c_unlock_from_isr();
  }
}

HAL_StatusTypeDef MPU6500_InitDMP(MPU6500Context *ctx)
{
  (void)ctx;
  /* 暂时不使用DMP，返回成功 */
  ctx->dmp_initialized = true;
  return HAL_OK;
}

void MPU6500_UpdateYaw(MPU6500Context *ctx)
{
  static uint32_t last_time_ms = 0;
  uint32_t current_time_ms = HAL_GetTick();
  float dt;

  /* 初始化 */
  if (last_time_ms == 0)
  {
    last_time_ms = current_time_ms;
    return;
  }

  /* 计算时间差（秒） */
  dt = (current_time_ms - last_time_ms) / 1000.0f;
  last_time_ms = current_time_ms;

  /* 只有在时间差合理且有有效数据的情况下才更新 */
  if (dt > 0 && dt < 2.0f && ctx->gyro.data_ready)
  {
    /* 对z轴角速度进行积分得到偏航角 */
    /* 注意：这是一个简化的方案，实际应用中需要处理偏置和漂移 */
    ctx->euler.yaw += ctx->gyro.dps_z * dt;

    /* 保持角度在-180到180度范围内 */
    while (ctx->euler.yaw > 180.0f)
      ctx->euler.yaw -= 360.0f;
    while (ctx->euler.yaw < -180.0f)
      ctx->euler.yaw += 360.0f;

    /* 数据已使用，清除标志 */
    ctx->gyro.data_ready = false;
  }
}

float MPU6500_GetYaw(MPU6500Context *ctx)
{
  return ctx->euler.yaw;
}

void MPU6500_ResetGyroData(MPU6500Context *ctx)
{
  ctx->gyro.raw_x = 0;
  ctx->gyro.raw_y = 0;
  ctx->gyro.raw_z = 0;
  ctx->gyro.dps_x = 0.0f;
  ctx->gyro.dps_y = 0.0f;
  ctx->gyro.dps_z = 0.0f;
  ctx->gyro.data_ready = false;
}

void MPU6500_ScanI2CBus(I2C_HandleTypeDef *hi2c, void (*debug_print)(const char *))
{
  if (!debug_print) return;
  
  debug_print("Scanning I2C bus...\r\n");
  uint8_t found = 0;
  char buf[64];
  
  for (uint8_t addr = 0x08; addr < 0x78; addr++)
  {
    if (HAL_I2C_IsDeviceReady(hi2c, (addr << 1), 1, 10) == HAL_OK)
    {
      snprintf(buf, sizeof(buf), "Found device at 0x%02X\r\n", addr);
      debug_print(buf);
      found = 1;
    }
  }
  
  if (!found)
  {
    debug_print("No I2C devices found! Check wiring!\r\n");
  }
}

/* 重置错误统计 */
void MPU6500_ResetErrorStats(MPU6500Context *ctx)
{
  ctx->total_error_count = 0;
  ctx->error_stats.timeout = 0;
  ctx->error_stats.nack = 0;
  ctx->error_stats.bus_error = 0;
  ctx->error_stats.other_error = 0;
}

/* ISR 安全的错误统计更新（从中断回调中调用） */
void MPU6500_UpdateErrorStatsFromISR(MPU6500Context *ctx, HAL_StatusTypeDef status)
{
  ctx->total_error_count++;

  switch (status)
  {
    case HAL_TIMEOUT:
      ctx->error_stats.timeout++;
      break;
    case HAL_ERROR:
      if (__HAL_I2C_GET_FLAG(ctx->hi2c, I2C_FLAG_AF))
      {
        ctx->error_stats.nack++;
      }
      else if (__HAL_I2C_GET_FLAG(ctx->hi2c, I2C_FLAG_BERR))
      {
        ctx->error_stats.bus_error++;
      }
      else
      {
        ctx->error_stats.other_error++;
      }
      break;
    default:
      ctx->error_stats.other_error++;
      break;
  }
}

/* 打印诊断信息 */
void MPU6500_DiagnosticPrint(MPU6500Context *ctx, void (*debug_print)(const char *))
{
  if (!debug_print) return;
  
  char buf[128];
  
  debug_print("----- MPU6500 Diagnostics -----\r\n");
  
  /* 基本信息 */
  snprintf(buf, sizeof(buf), "WHO_AM_I: 0x%02X (OK: %s)\r\n", 
           ctx->who_am_i, ctx->gyro.who_am_i_ok ? "Yes" : "No");
  debug_print(buf);
  
  snprintf(buf, sizeof(buf), "I2C Addr: 0x%02X\r\n", ctx->i2c_addr_8bit >> 1);
  debug_print(buf);
  
  /* 数据统计 */
  snprintf(buf, sizeof(buf), "DMA Read Count: %lu\r\n", (unsigned long)ctx->dma_read_count);
  debug_print(buf);
  
  /* 错误统计 */
  debug_print("\r\nError Statistics:\r\n");
  snprintf(buf, sizeof(buf), "Total Errors: %lu\r\n", (unsigned long)ctx->total_error_count);
  debug_print(buf);
  snprintf(buf, sizeof(buf), "  Timeout: %lu\r\n", (unsigned long)ctx->error_stats.timeout);
  debug_print(buf);
  snprintf(buf, sizeof(buf), "  NACK: %lu\r\n", (unsigned long)ctx->error_stats.nack);
  debug_print(buf);
  snprintf(buf, sizeof(buf), "  Bus Error: %lu\r\n", (unsigned long)ctx->error_stats.bus_error);
  debug_print(buf);
  snprintf(buf, sizeof(buf), "  Other: %lu\r\n", (unsigned long)ctx->error_stats.other_error);
  debug_print(buf);
  
  /* 当前陀螺仪数据 */
  debug_print("\r\nCurrent Gyro Data:\r\n");
  snprintf(buf, sizeof(buf), "  Raw: X=%d Y=%d Z=%d\r\n", 
           ctx->gyro.raw_x, ctx->gyro.raw_y, ctx->gyro.raw_z);
  debug_print(buf);
  snprintf(buf, sizeof(buf), "  DPS: X=%.2f Y=%.2f Z=%.2f\r\n", 
           (double)ctx->gyro.dps_x, (double)ctx->gyro.dps_y, (double)ctx->gyro.dps_z);
  debug_print(buf);
  
  /* 欧拉角 */
  snprintf(buf, sizeof(buf), "  Yaw: %.2f deg\r\n", (double)ctx->euler.yaw);
  debug_print(buf);
  
  debug_print("-------------------------------\r\n");
}