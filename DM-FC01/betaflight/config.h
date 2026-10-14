/*
 * DM-FC01（达妙科技）Betaflight 板级配置，非官方。
 *
 * 引脚来源：达妙 PX4 板级源码 CaFeZn/PX4-Autopilot 分支 damiao_dm-fc01
 * （boards/damiao/dm-fc01，与出厂 PX4 固件一致）+《DM-FC01 开发手册》。
 * 说明书里的 FirstFlightControler.ioc 是早期版本（ICM45686 标成双 CS、缺 PD4/PE12/PE15），不作依据。
 *
 * BMI088 依赖未合入的 betaflight PR #15771（ACC_n_CS_PIN 是该 PR 新增的宏）。
 * 构建与验证状态见同目录 README.md。
 */

#pragma once

#define FC_TARGET_MCU        STM32H743

#define BOARD_NAME           DM_FC01
#define MANUFACTURER_ID      DAMI    // 自拟，未在 betaflight/config 登记

#define SYSTEM_HSE_MHZ       16      // X1 16 MHz 晶振（PX4 board.h STM32_BOARD_XTAL）

#define USE_ACC
#define USE_GYRO
#define USE_ACCGYRO_ICM45686
#define USE_ACCGYRO_BMI088
#define USE_BARO
#define USE_BARO_SPA06_003           // 走 DPS310 驱动的 SPL07_003 分支，芯片 ID 为 0x11 时 status 显示 SPA06_003
#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_SDCARD
#define USE_SDCARD_SDIO
#define USE_IMU_HEATER

#ifndef USE_MAG
#define USE_MAG
#endif

#ifndef USE_GPS
#define USE_GPS
#endif

// 电机：PWM 输出接口 1（M1-M4）与接口 2（M5-M8）
#define MOTOR1_PIN           PE9     // TIM1_CH1
#define MOTOR2_PIN           PE11    // TIM1_CH2
#define MOTOR3_PIN           PE13    // TIM1_CH3
#define MOTOR4_PIN           PE14    // TIM1_CH4
#define MOTOR5_PIN           PB1     // TIM3_CH4
#define MOTOR6_PIN           PB0     // TIM3_CH3
#define MOTOR7_PIN           PB10    // TIM2_CH3
#define MOTOR8_PIN           PB11    // TIM2_CH4

#define UART1_TX_PIN         PA9     // GPS1
#define UART1_RX_PIN         PA10
#define UART2_TX_PIN         PD5     // TELEM1
#define UART2_RX_PIN         PD6
#define UART3_TX_PIN         PD8     // TELEM2 / DJI O3
#define UART3_RX_PIN         PD9
#define UART4_TX_PIN         PB9     // TELEM3
#define UART4_RX_PIN         PB8
#define UART5_TX_PIN         PB13    // RC_INPUT（DJI 口的 SBUS 也接 PB12）
#define UART5_RX_PIN         PB12
#define UART6_TX_PIN         PC6     // TELEM4
#define UART6_RX_PIN         PC7
#define UART7_RX_PIN         PE7     // 电调回传，接口只引出 RX
#define UART8_TX_PIN         PE1     // DEBUG
#define UART8_RX_PIN         PE0

#define I2C1_SCL_PIN         PB6     // 板载气压计
#define I2C1_SDA_PIN         PB7
#define I2C4_SCL_PIN         PD12    // GPS1 口外接（罗盘）
#define I2C4_SDA_PIN         PD13

#define SPI1_SCK_PIN         PA5     // BMI088
#define SPI1_SDI_PIN         PA6
#define SPI1_SDO_PIN         PA7
#define SPI2_SCK_PIN         PD3     // W25Q128
#define SPI2_SDI_PIN         PB14
#define SPI2_SDO_PIN         PC3
#define SPI3_SCK_PIN         PB3     // 焊盘对外预留
#define SPI3_SDI_PIN         PB4
#define SPI3_SDO_PIN         PB2
#define SPI4_SCK_PIN         PE2     // ICM45686
#define SPI4_SDI_PIN         PE5
#define SPI4_SDO_PIN         PE6

#define SDIO_CK_PIN          PC12
#define SDIO_CMD_PIN         PD2
#define SDIO_D0_PIN          PC8
#define SDIO_D1_PIN          PC9
#define SDIO_D2_PIN          PC10
#define SDIO_D3_PIN          PC11

// 三色灯均低电平点亮（PX4 定义为 nLED），与 Betaflight 默认极性一致
#define LED0_PIN             PB15    // 蓝
#define LED1_PIN             PD11    // 绿
#define LED2_PIN             PE12    // 红

#define ADC_VBAT_PIN         PC4     // ADC12_INP4
#define ADC_CURR_PIN         PC5     // ADC12_INP8

#define USB_DETECT_PIN       PA15

#define FLASH_CS_PIN         PD4

#define GYRO_1_CS_PIN        PC13    // ICM45686
#define GYRO_1_EXTI_PIN      PE4
#define GYRO_2_CS_PIN        PA3     // BMI088 陀螺仪
#define GYRO_2_EXTI_PIN      PA1
#define ACC_2_CS_PIN         PA2     // BMI088 加速度计（独立片选）

// 这两个脚 Betaflight 本身不管，用 PINIO 在传感器探测前钉住电平
#define PINIO1_PIN           PB5     // SPA06 SDO 地址选择，高 = 0x77
#define PINIO2_PIN           PE15    // SBUS 反相器使能，拉低关闭（H7 由 UART 内部反相）

// IMU 加热器（本地补丁 patches/0001-imu-heater.patch，非上游功能）：第 n 路用 GYRO_n 的芯片温度闭环
#define IMU_HEATER_1_PIN     PD15    // ICM45686
#define IMU_HEATER_2_PIN     PD14    // BMI088

#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, MOTOR1_PIN, 1, 0) \
    TIMER_PIN_MAP( 1, MOTOR2_PIN, 1, 1) \
    TIMER_PIN_MAP( 2, MOTOR3_PIN, 1, 2) \
    TIMER_PIN_MAP( 3, MOTOR4_PIN, 1, 3) \
    TIMER_PIN_MAP( 4, MOTOR5_PIN, 2, 4) \
    TIMER_PIN_MAP( 5, MOTOR6_PIN, 2, 5) \
    TIMER_PIN_MAP( 6, MOTOR7_PIN, 1, 6) \
    TIMER_PIN_MAP( 7, MOTOR8_PIN, 1, 7)
// PB0/PB1 在 H7 定时器表里第 1 项是 TIM1_CHxN，第 2 项才是 TIM3

#define ADC1_DMA_OPT         8
#define ADC3_DMA_OPT         12      // H743 的 VREFINT/芯片温度在 ADC3，缺了它 Vref 读数无效，电压也跟着错
#define TIMUP1_DMA_OPT       9
#define TIMUP2_DMA_OPT       10
#define TIMUP3_DMA_OPT       11

#define BARO_I2C_INSTANCE    I2CDEV_1
#define DEFAULT_BARO_I2C_ADDRESS 119   // 0x77，驱动默认 0x76
#define MAG_I2C_INSTANCE     I2CDEV_4

#define DEFAULT_BLACKBOX_DEVICE      BLACKBOX_DEVICE_FLASH
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
// 由达妙给的 ArduPilot 参数换算（BATT_VOLT_MULT 10.2、BATT_AMP_PERVOLT 20.4），需实测校准
#define DEFAULT_VOLTAGE_METER_SCALE  102
#define DEFAULT_CURRENT_METER_SCALE  490

#define DEFAULT_RX_FEATURE   FEATURE_RX_SERIAL
#define SERIALRX_UART        SERIAL_PORT_UART5
#define GPS_UART             SERIAL_PORT_USART1
#define MSP_DISPLAYPORT_UART SERIAL_PORT_USART3
#define ESC_SENSOR_UART      SERIAL_PORT_UART7

#define FLASH_SPI_INSTANCE   SPI2
#define SDIO_DEVICE          SDIODEV_1
#define SDIO_USE_4BIT        1
#define SDCARD_DETECT_PIN    NONE

// 以板上箭头为机头实测（2026-10-14 手持倾斜）：ICM45686、BMI088 都是 CW90_DEG。
// 按 PX4 rc.board_sensors 的 -R 4（YAW_180）推出的 CW180_DEG 会差 90°：压机头显示成右倾，压右侧显示成抬头。
#define DEFAULT_GYRO_TO_USE  GYRO_CONFIG_USE_GYRO_1
#define GYRO_1_SPI_INSTANCE  SPI4
#define GYRO_1_ALIGN         CW90_DEG
#define GYRO_2_SPI_INSTANCE  SPI1
#define GYRO_2_ALIGN         CW90_DEG

// 0x81 = 推挽输出 + 反相（上电即为高），0x01 = 推挽输出（上电为低）
#define PINIO1_CONFIG        129
#define PINIO2_CONFIG        1

// 达妙 PX4 rc.board_defaults 的 HEATERn_TEMP*，换算到本补丁单位（P %/°C，I 0.1%/(°C·s)，FF %）。
// PX4 积分每 10 ms 累加一次，HEATER2_TEMP_I 0.001 → 0.1/(°C·s) → 100。
#define DEFAULT_IMU_HEATER_TARGET    48
#define DEFAULT_IMU_HEATER_1_P       65      // HEATER2_TEMP_P  0.65（ICM45686）
#define DEFAULT_IMU_HEATER_1_I       100     // HEATER2_TEMP_I  0.001
#define DEFAULT_IMU_HEATER_1_FF      2       // HEATER2_TEMP_FF 0.02
#define DEFAULT_IMU_HEATER_2_P       20      // HEATER1_TEMP_P  0.20（BMI088）
#define DEFAULT_IMU_HEATER_2_I       0       // HEATER1_TEMP_I  0
#define DEFAULT_IMU_HEATER_2_FF      4       // HEATER1_TEMP_FF 0.04
