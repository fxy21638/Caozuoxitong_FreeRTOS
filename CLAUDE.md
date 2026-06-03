# CLAUDE.md

## Project Overview

物联网总线通信项目 —— 基于 FreeRTOS 的 RS485 多终端传感采集与轮询通信系统。

- **MCU**: STM32F429IGTx (野火 挑战者 开发板)
- **RTOS**: FreeRTOS V9.0.0
- **IDE/Toolchain**: Keil MDK V5 (RVMDK uv5), ARM Compiler (CC_ARM)
- **StdPeriph Library**: STM32F4xx StdPeriph Driver V1.5.0
- **Project location**: `Project/RVMDK（uv5）/Fire_FreeRTOS.uvprojx`

## System Architecture

系统包含 **1 个管理端 + 2 个采集前端**，通过 RS485 总线互联。

```
┌─────────────────────────────────────────────────────────────┐
│                     RS485 Bus (双绞线 A/B)                    │
├──────────┬──────────────────────┬────────────────────────────┤
│  管理端   │    采集前端1          │    采集前端2               │
│ (地址0x01)│   (地址0x02)         │   (地址0x03)               │
│          │                      │                            │
│ LCD+触摸  │  温湿度传感器(DHTxx)  │  MPU6050 姿态传感器(I2C)   │
│ LED控制  │  采集间隔: 5秒       │  采集间隔: 2秒(DMP解算)    │
│          │  本地存储10组         │  本地存储10组               │
│ 轮询周期: │  GUI显示+触摸        │  GUI显示+触摸              │
│ 10秒     │                      │                            │
│ 存储2区  │                      │                            │
│ 各10组   │                      │                            │
└──────────┴──────────────────────┴────────────────────────────┘
```

### 通信机制

管理端每 10 秒轮询一次：依次读取前端1温湿度 → 前端2姿态角 → 循环。500ms 内未收到应答则重读该帧。

前端收到 LED 控制帧 (0x03) 时立即执行并应答。

### 设备角色配置

通过 `User/config.h` 中的宏切换角色：

```c
#define DEVICE_ROLE  ROLE_MANAGER      // 管理端
//#define DEVICE_ROLE  ROLE_COLLECTOR_1  // 采集前端1 (温湿度)
//#define DEVICE_ROLE  ROLE_COLLECTOR_2  // 采集前端2 (MPU6050)

#define DEVICE_ADDR  0x01  // 本机 RS485 地址
```

## Protocol Design

### 帧格式

```
┌──────┬──────┬──────┬──────┬──────────┬──────┬──────┐
│ 0xAA │ Addr │ Len  │ Type │  Data…   │ CRC  │ 0x55 │
│ 帧头  │ 地址 │ 长度  │ 类型  │ 传感数据  │ 校验  │ 帧尾  │
└──────┴──────┴──────┴──────┴──────────┴──────┴──────┘
```

- **帧头**: 0xAA (1 byte)
- **地址字节**: 目标地址，管理端=0x01，前端1=0x02，前端2=0x03
- **长度字节**: 从消息类型到 CRC 前的数据总字节数
- **消息类型**: 0x01=温湿度, 0x02=MPU6050姿态, 0x03=LED控制
- **CRC**: 1 byte，校验范围从帧头(0xAA)到 CRC 前所有字节，CRC-8 (多项式 0x07)
- **帧尾**: 0x55 (1 byte)

### 消息类型详解

**Type 0x01 — 温湿度 (采集前端1)**

```
请求: AA 02 01 01 CRC 55         (Len=1, Data=0x01=读取)
应答: AA 01 05 01 TempH TempL HumH HumL CRC 55
     温度 = ((TempH<<8)|TempL) / 10.0 (摄氏度)
     湿度 = ((HumH<<8)|HumL) / 10.0 (%)
```

**Type 0x02 — MPU6050 姿态角 (采集前端2)**

```
请求: AA 03 01 02 CRC 55         (Len=1, Data=0x02=读取)
应答: AA 01 0D 02 PitchH PitchL RollH RollL YawH YawL CRC 55
     姿态角为 DMP 解算后的欧拉角 (pitch/roll/yaw)，每个 float 4 字节，共 12 字节数据
```

**Type 0x03 — LED 控制**

```
控制: AA 02 02 03 0x01 CRC 55   (0x01=点亮) 或 AA 02 02 03 0x00 CRC 55 (0x00=熄灭)
应答: AA 01 02 03 0x01 CRC 55   (回显当前 LED 状态)
```

### 协议规则

1. **100ms 超时丢弃**: 帧间字节间隔超过 100ms 则丢弃不完整帧，重置状态机
2. **地址过滤**: 前端只处理 `Addr == 自身地址` 的帧，其余丢弃
3. **500ms 重读**: 管理端发出请求后 500ms 内未收到正确应答帧，重发同一帧
4. **CRC 校验**: 校验失败直接丢弃，管理端不计入 500ms 计时（等待超时后重读）

## Pin Mapping

### RS485 接口 (USART2 + DE/RE 控制)

| 功能 | GPIO | AF | 说明 |
|------|------|----|------|
| USART2 TX | PD5 | GPIO_AF_USART2 | 485 数据发送 |
| USART2 RX | PD6 | GPIO_AF_USART2 | 485 数据接收 |
| 485 DE/RE | PG14 | (GPIO 输出) | 高=发送模式, 低=接收模式 |

> 注意: 若 PD6 被 LCD(LTDC B2) 占用，RS485 改用 USART3: PB10(TX)/PB11(RX), DE=PG14。以实际原理图为准。

### 传感器接口

**温湿度 (采集前端1 — 单总线 DHT11/DHT22)**

| 功能 | GPIO | 说明 |
|------|------|------|
| DATA | PG9 | 单总线数据，需 4.7kΩ 上拉 |

**MPU6050 (采集前端2 — I2C)**

| 功能 | GPIO | AF | 说明 |
|------|------|----|------|
| I2C1 SCL | PB6 | GPIO_AF_I2C1 | 400kHz |
| I2C1 SDA | PB7 | GPIO_AF_I2C1 | |
| INT | PC8 | (GPIO 输入) | 可选，DMP 数据就绪中断 |

### 触摸屏 (全部设备均需)

| 功能 | GPIO | 说明 |
|------|------|------|
| I2C3 SCL | PA8 | GT9157/FT5x06 触摸控制器 |
| I2C3 SDA | PC9 | |

> 需根据实际触摸芯片型号选择 GT9157 或 FT5x06 驱动。GT9157 地址 0x14/0x5D，FT5x06 地址 0x38。

### 原有外设 (保持不变)

| 功能 | GPIO | 说明 |
|------|------|------|
| USART1 TX | PA9 | 调试串口 (115200) |
| USART1 RX | PA10 | |
| KEY1 | PA0 | 按下低电平 |
| KEY2 | PC13 | 按下低电平 |
| LED R | PH10 | 低电平亮 |
| LED G | PH11 | 低电平亮 |
| LED B | PH12 | 低电平亮 |
| LED4 | PD11 | 低电平亮 |
| LCD (LTDC RGB565) | 28 pin | 800x480, 帧缓冲 @ 0xD0000000 |

> 注意: USART1 PA9/PA10 与 LTDC HSYNC/VSYNC 复用。调试串口在启用 LCD 后不可用，改用 RS485 接口或 SWO 调试输出。

## FreeRTOS Task Layout

### 管理端 (ROLE_MANAGER)

| 任务 | 优先级 | 栈 | 职责 |
|------|--------|-----|------|
| RS485RxTask | 6 | 512 | RS485 帧接收：ISR 任务通知逐字节→状态机解析→CRC校验 |
| RS485TxTask | 5 | 256 | 组帧(含CRC)→发送→等待TC中断→切换接收模式 |
| PollTask | 4 | 512 | 10s 周期轮询两个前端，管理 500ms 超时重读 |
| GUITask | 3 | 1024 | 触摸屏 GUI+传感器显示+历史记录+LED控制按钮 |
| Tmr Svc | 31 | 256 | FreeRTOS 软件定时器 (100ms帧超时, 500ms重读) |

### 采集前端1 — 温湿度 (ROLE_COLLECTOR_1)

| 任务 | 优先级 | 栈 | 职责 |
|------|--------|-----|------|
| RS485RxTask | 6 | 512 | RS485 帧接收→地址过滤(仅处理0x02)→协议解析 |
| RS485TxTask | 5 | 256 | 组帧(含CRC)→发送→切换接收模式 |
| SensorTask | 4 | 512 | 5s 周期读取 DHTxx→存储→通知 GUI 更新 |
| GUITask | 3 | 1024 | 触摸屏 GUI：显示温湿度/历史10组 |
| Tmr Svc | 31 | 256 | FreeRTOS 软件定时器 |

### 采集前端2 — MPU6050 (ROLE_COLLECTOR_2)

| 任务 | 优先级 | 栈 | 职责 |
|------|--------|-----|------|
| RS485RxTask | 6 | 512 | RS485 帧接收→地址过滤(仅处理0x03)→协议解析 |
| RS485TxTask | 5 | 256 | 组帧(含CRC)→发送→切换接收模式 |
| SensorTask | 4 | 512 | 2s 周期读取 MPU6050 DMP→存储→通知 GUI 更新 |
| GUITask | 3 | 1024 | 触摸屏 GUI：显示姿态角/历史10组 |
| Tmr Svc | 31 | 256 | FreeRTOS 软件定时器 |

## FreeRTOS Config Highlights

- **抢占式调度**: `configUSE_PREEMPTION = 1`
- **Tick**: 1000Hz
- **最大优先级**: 32
- **堆大小**: 36KB (GUI 需求大，可能需增大到 48KB+)
- **软件定时器**: `configUSE_TIMERS = 1`
- **定时器任务**: 优先级 31, 栈 256, 队列深度 10
- **中断优先级分组**: NVIC_PriorityGroup_4
- **RS485 USART 中断优先级**: 抢占 6 (FreeRTOS 安全范围 5~15)
- **MPU6050 I2C**: 在任务上下文轮询/中断操作，不在 ISR 中

## Protocol State Machine

```
STATE_WAIT_HEADER → 收到 0xAA → STATE_WAIT_ADDR
STATE_WAIT_ADDR   → 收到地址   → 地址过滤(前端) → STATE_WAIT_LEN
STATE_WAIT_LEN    → 收到长度 N → 长度合法性检查 → STATE_WAIT_TYPE
STATE_WAIT_TYPE   → 收到类型   → STATE_WAIT_DATA
STATE_WAIT_DATA   → 收 N 字节  → STATE_WAIT_CRC
STATE_WAIT_CRC    → 收到 CRC   → STATE_WAIT_TAIL
STATE_WAIT_TAIL   → 收到 0x55  → CRC 校验 → 处理帧 → STATE_WAIT_HEADER
```

- 100ms 软件定时器 (one-shot): 每收到一个字节 `xTimerResetFromISR`，超时则 `Protocol_Reset()`
- 帧长度检查: `Len > MAX_FRAME_DATA` → 丢弃复位
- 地址过滤 (仅前端): `Addr != DEVICE_ADDR` → 丢弃复位
- CRC-8 校验: 多项式 0x07，覆盖 AA ~ CRC 前一字节

## Project File Structure

```
User/
├── main.c                    # 主程序：初始化→创建任务→启动调度器
├── stm32f4xx_it.c            # 中断服务 (RS485 RXNE ISR, Timer ISR)
├── stm32f4xx_it.h
├── stm32f4xx_conf.h          # 外设头文件配置
├── FreeRTOSConfig.h          # FreeRTOS 配置 (已启用 timers)
├── config.h                  # 新建：设备角色/地址/参数配置
├── uart/                     # 原有：调试串口 (开发阶段使用)
│   ├── bsp_debug_usart.c
│   └── bsp_debug_usart.h
├── rs485/                    # 新建：RS485 驱动
│   ├── bsp_rs485.h
│   └── bsp_rs485.c
├── protocol/                 # 新建：RS485 协议解析 (地址过滤+CRC)
│   ├── protocol.h
│   └── protocol.c
├── sensor/                   # 新建：传感器驱动
│   ├── bsp_dht.h
│   ├── bsp_dht.c             #   DHT11/DHT22 单总线驱动
│   ├── bsp_mpu6050.h
│   ├── bsp_mpu6050.c         #   MPU6050 I2C + DMP 驱动
│   ├── inv_mpu.h             #   InvenSense 官方库
│   ├── inv_mpu.c
│   ├── inv_mpu_dmp_motion_driver.h
│   └── inv_mpu_dmp_motion_driver.c
├── gui/                      # 新建：触摸屏 GUI
│   ├── gui.h
│   ├── gui.c
│   ├── touch.h
│   └── touch.c               #   GT9157/FT5x06 触摸驱动
├── data/                     # 新建：传感器数据存储 (环形缓冲)
│   ├── data_store.h
│   └── data_store.c
├── lcd/                      # 原有：LCD 驱动 (LTDC + SDRAM)
│   ├── bsp_lcd.c
│   └── bsp_lcd.h
├── sdram/                    # 原有：SDRAM 驱动
│   ├── bsp_sdram.c
│   └── bsp_sdram.h
├── font/                     # 原有：字库
│   ├── fonts.c
│   └── fonts.h
├── led/                      # 原有
├── key/                      # 原有
└── exti/                     # 原有
```

## Keil Project Setup (Manual Steps Required)

在 Keil MDK 中需要手动完成以下配置：

1. **添加新源文件到工程**:
   - `rs485/bsp_rs485.c`, `protocol/protocol.c`
   - `sensor/bsp_dht.c`, `sensor/bsp_mpu6050.c`
   - `sensor/inv_mpu.c`, `sensor/inv_mpu_dmp_motion_driver.c`
   - `gui/gui.c`, `gui/touch.c`
   - `data/data_store.c`
   - `timers.c` (FreeRTOS/src/)
2. **添加头文件路径**: `User\rs485`, `User\protocol`, `User\sensor`, `User\gui`, `User\data`
3. **确认宏定义**: `STM32F429_439xx` (确保 LTDC/FMC/DMA2D/I2C 头文件被包含)
4. **SDRAM 初始化**: `bsp_sdram.c` 需参与编译，否则 LCD 无法工作
5. **栈大小**: 建议 `Stack_Size` 至少 0x2000 (GUI 栈需求大)
6. **MicroLIB**: 如需 `printf` 浮点输出，启用 "Use MicroLIB"

## Encoding Warning (CRITICAL)

**原始 `.c` 和 `.h` 源文件是 GB2312 编码（中文注释）。**

### Edit 工具会破坏 GB2312 文件！

即使 `old_string` 只包含 ASCII 字符，**Edit 工具内部会将文件当作 UTF-8 读取**，发现无效 UTF-8 序列（即 GB2312 中文字节）后，会将每个中文字符替换为 U+FFFD (EF BF BD)。文件会被永久破坏。

**绝对不要对 GB2312 文件使用 Edit 或 Write 工具！**

### 安全的修改方式

对于 GB2312 文件的修改，使用 PowerShell 字节级操作：

```powershell
$bytes = [System.Collections.ArrayList]::new([System.IO.File]::ReadAllBytes($path))
$pattern = [System.Text.Encoding]::ASCII.GetBytes('old_string')
$replacement = [System.Text.Encoding]::ASCII.GetBytes('new_string')
# 定位并替换...
[System.IO.File]::WriteAllBytes($path, [byte[]]$bytes)
```

### 检测文件是否已被破坏

```powershell
# 检查 EF BF BD 序列数量。GB2312 中 1721 个是正常的。
# 关键是看文件大小是否膨胀（UTF-8 中文字符是 3 字节，GB2312 是 2 字节）
```

- CLAUDE.md 本身是 UTF-8，不影响

## Common Pitfalls

### 1. RS485 方向控制
发送前拉高 DE/RE，发送完成后必须拉低。在 `USART_IT_TC` (发送完成中断) 中拉低 DE/RE。否则最后一个字节丢失或总线锁死。

### 2. 半双工总线冲突
RS485 同一时刻只能有一个设备发送。管理端轮询方式天然避免冲突。前端**只在收到完整正确帧后**才能应答。禁止前端主动上报。

### 3. 100ms 帧超时
115200 波特率下，最长帧 (~30 bytes) 传输约 3ms。100ms 超时足够宽裕，注意超时定时器为 one-shot，每收到字节重置。

### 4. MPU6050 DMP
需上传 DMP 固件到 MPU6050。`inv_mpu.c`/`inv_mpu_dmp_motion_driver.c` 为 InvenSense 官方库（约 30KB 常驻 flash）。I2C 速率 400kHz，DMP 初始化约 200ms。

### 5. DHT 单总线时序
DHT11/DHT22 需要精确 μs 级延迟。FreeRTOS 中读取时需 `taskENTER_CRITICAL()` 防止时序被高优先级任务打断。

### 6. 触摸屏驱动
确认触摸芯片型号 (GT9157 或 FT5x06)。GT9157 地址 0x14/0x5D，FT5x06 地址 0x38。驱动不通用。

### 7. GB2312 编码
修改 `.c`/`.h` 文件时整文件重写会破坏中文注释。新建文件用 UTF-8 编码，或只做字节级修改。

### 8. 中断优先级
USART 中断优先级 6 在 FreeRTOS 安全范围 (5~15) 内，可安全调用 `xTaskNotifyFromISR`/`xTimerResetFromISR`。

### 9. LCD 依赖 SDRAM
LCD 帧缓冲在外部 SDRAM (`0xD0000000`)。`SDRAM_Init()` 必须在 `LCD_Init()` 之前调用。

### 10. 任务通知覆盖
`eSetValueWithOverwrite` 在高波特率下可能丢字节。RS485RxTask 优先级设为 6（最高应用优先级），确保 ISR 返回后立即消费字节。
