# 4-Axis Flight Controller - Project Context

## 项目概述

基于 STM32F103C8T6 (Cortex-M3, 72MHz) 的四轴无人机飞行控制器固件。
- RTOS: FreeRTOS (heap_4, 1000Hz tick)
- HAL: STM32Cube FW_F1 V1.8.7
- IDE: Keil MDK-ARM v5.32, 由 STM32CubeMX v6.15.0 生成初始代码
- 传感器: MPU6050 (I2C1), SBUS 接收机 (USART2 DMA), SI24R1 数传 (SPI1)
- 执行器: 4路 ESC PWM 输出 (TIM1-TIM4, 50Hz)

## 目录结构

```
Core/
├── Inc/                  # CubeMX 生成的 HAL 外设头文件 (不修改)
├── Src/                  # CubeMX 生成的 HAL 外设源文件 (不修改)
├── application/          # 应用层: FreeRTOS 任务、飞控、遥控接收
├── common_caculate/      # 算法库: PID、滤波器、四元数姿态解算
├── hardware/             # 硬件驱动: IMU、电机、SBUS、SI24R1
└── Freertos/             # FreeRTOS 内核源码 (手动移植)
```

## 数据流

```
SBUS接收机 → USART2(DMA环形) → IDLE中断 → sbus_task → app_receive_getdata()
                                                          ↓ rc_data
MPU6050 → I2C1 → flight_task → app_flight_get_euler_angles() → 姿态角
                        ↓
              app_flight_pid_process() → 级联PID → Motor_Setspeed() → ESC PWM
```

FreeRTOS 任务:
- `flight_task` (优先级4): 6ms周期, IMU读取 + 姿态解算 + 级联PID控制
- `sbus_task` (优先级3): 6ms周期, SBUS DMA数据解析
- `led_task` (优先级2): 6ms周期, 预留

---

## 代码风格规范

以下规范适用于 `Core/application/`、`Core/common_caculate/`、`Core/hardware/` 中所有手写代码。
CubeMX 自动生成的 `Core/Inc/`、`Core/Src/` 下文件保持工具生成风格不改动。

### 缩进与格式
- **4 空格缩进**，不使用 Tab
- 编码: UTF-8
- 行宽: 不超过 100 字符

### 大括号 (K&R 风格)
```c
// 函数定义 — 左括号与函数名不同行
void app_freertos_start(void) 
{
    // ...
}

// 控制流 — 左括号同行
if (condition) 
{
    // ...
} 
else 
{
    // ...
}

while (1) 
{
    // ...
}
```

### 命名约定

| 类型 | 风格 | 示例 |
|------|------|------|
| 应用层函数 | `snake_case` | `app_freertos_start()`, `app_receive_getdata()` |
| 模块接口函数 | `ModuleName_Action()` | `PID_Calc()`, `Motor_Init()`, `Filter_LowPass()` |
| 全局/局部变量 | `snake_case` | `rc_data`, `last_gyro_data`, `xLastWakeTime` |
| 宏常量 | `UPPER_SNAKE_CASE` | `MAX_ROLL_ANGLE`, `SBUS_FRAME_LEN` |
| 结构体类型 | `Name_Struct` | `PID_Struct`, `Motor_Struct`, `Euler_struct` |
| Typedef 别名 | `Name_t` | `RC_Data_t` |
| 文件私有函数/变量 | 加 `static` | `static void map_channel(...)` |

### Include 守卫
```c
#ifndef __FILE_H
#define __FILE_H


// ... 头文件内容 ...

#endif
```
- 使用 `#ifndef` (不用 `#pragma once`)
- 新文件的守卫宏命名: `__FILE_H` (双下划线, 不带尾下划线)

### Include 顺序
每个 `.c` 文件先包含自己的头文件，再包含其他依赖:
```c
#include "app_flight.h"      // 自己的头文件

#include "app_receive.h"     // 其他模块依赖
#include "imu.h"
```

### 注释

- **函数文档**: 使用 `/** @brief ... @param ... @retval ... */` (Doxygen 风格)
- **行内注释**: 使用 `//`
- **语言**: 应用层/硬件驱动层注释用中文; 算法库(common_caculate)注释用英文
- **分段标记**: 使用 `/* ===== 段落名 ===== */`

```c
/**
 * @brief 获取遥控器数据并映射为飞控目标值
 * @param data RC_Data_t 指针
 */
void app_receive_getdata(RC_Data_t *data) {
    /* ===== SBUS帧解析 ===== */
    // 检查帧头
    if (sbus_frame[0] != 0x0F) {
        return;
    }
    // ...
}
```

### 指针与类型
- 指针声明: `type *name` (星号两侧均有空格)
- 零初始化: `= {0}`
- 指定初始化器: `= {.kp = 0, .ki = 0, .kd = 0}`
- ISR 共享变量: 加 `volatile` 修饰

### 模块文件结构
```c
// module.c 的标准结构:
// 1. #include 头文件
// 2. #define 宏常量
// 3. static 全局变量
// 4. static 辅助函数
// 5. 公开函数实现
```

### 误差处理
- CubeMX 生成的致命错误: `while(1)` 死循环
- 手写代码中不添加冗余的错误检查, 信任内部 API 和硬件保证
- 仅在系统边界(外部输入、通信协议)做校验 (如 SBUS 帧头校验、加速度计量程校验)

# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.
