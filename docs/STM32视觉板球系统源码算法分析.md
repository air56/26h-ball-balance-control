# 视觉板球系统 STM32 源码算法分析

> 来源：`F:\1A TI library\26电赛\开源算法\1.视觉板球系统 STM32源码\1.视觉板球系统 STM32源码`
> 作者：平衡小车之家
> 芯片：STM32F103（标准库 V3.5）

## 0. 系统概览

这是一个**双舵机倾斜平台 + 摄像头视觉闭环**的板球系统：

- **视觉端（鲁班猫）**：识别小球，通过串口把小球像素坐标发给 STM32。
- **STM32**：解算坐标 → PD(+积分) 控制 → 输出两路 PWM 驱动两个舵机 → 倾斜平板让小球滚向目标点。
- **PS2 手柄**：用于设置目标点、校准板面水平、在线调 PID 参数。
- **OLED**：显示状态、坐标、PID 参数。

```
鲁班猫(视觉) ──USART1──> 坐标帧 0x7B X Y 0x7D
                              │
                     STM32F103 (中断驱动)
                              │
                  PD控制 → Set_Pwm() → TIM4 PWM 两路舵机
                              │
                     OLED / PS2 / Flash / 蓝牙APP
```

**核心文件**：

| 文件 | 职责 |
|------|------|
| `USER/MiniBalance.c` | 主程序：初始化 + 主循环（PS2采集 / 显示 / 计时） |
| `SYSTEM/usart/usart.c` | **USART1** 接收鲁班猫坐标帧，解析出 `X/Y` |
| `MiniBalance/CONTROL/control.c` | **核心控制**：TIM1 定时中断里的所有控制代码 |
| `MiniBalance/HARDWARE/USART2/usart2.c` | USART3 蓝牙 APP 在线调参 |
| `MiniBalance/HARDWARE/MOTOR/motor.c` | TIM4 四通道 PWM（舵机） |
| `MiniBalance/HARDWARE/STMFLASH/stmflash.c` | 板面水平 PWM 校准值的 Flash 存储 |
| `MiniBalance/HARDWARE/EXTI/exti.c` | EXTI3 中断：触摸开关控制启停 |
| `MiniBalance/show/show.c` | OLED 显示 + DataScope 上位机预留 |

---

## 1. 视觉数据接收算法（USART1）

文件：`SYSTEM/usart/usart.c` → `USART1_IRQHandler`

**协议**：鲁班猫每帧发 4 字节 `0x7B X Y 0x7D`（帧头 0x7B，帧尾 0x7D，中间两字节为坐标）。

```c
rxbuf[Count] = Usart_Receive;
if (Usart_Receive == 0X7B || Count > 0) Count++;   // 帧头或已同步后继续累积
else Count = 0;                                    // 未同步则丢弃并重头开始
if (Count == 5) {                                  // 收集满一帧
    Count = 0;
    if (rxbuf[4] == 0X7D) {                        // 校验帧尾
        Urxbuf[0] = rxbuf[2];                      // X 坐标
        Urxbuf[1] = rxbuf[3];                      // Y 坐标
        Usart_Flag = 0;                            // 通知控制层有新数据
    }
}
```

- **同步策略**：遇 `0x7B` 才开始计数；`Count` 从 1 数到 5 即收满 4 字节负载；收满后校验 `rxbuf[4]==0x7D`，不匹配则丢弃整帧。
- **数据交接**：中断写 `Urxbuf[0/1]`，TIM1 中断里 `memcpy` 到 `rxbuf[0/1]`（见下文），形成**双缓冲 + 标志位**交接。

**与单片机主循环的衔接**（`control.c` TIM1 中断）：

```c
if (Usart_Flag == 0) {
    memcpy(rxbuf, Urxbuf, 2*sizeof(u8));
    Usart_Flag = 1;          // 置位表示已消费
}
Position_Y = rxbuf[1];
Position_X = rxbuf[0];
```

**丢球保护**：视觉端在无法检测到小球时发送特殊值 `253/253`（0xFD），控制层判断：

```c
if (Position_Y != 253 || Position_X != 253) {        // 检测到球才更新目标
    Target_X = -balanceX(Position_X);
    Target_Y = -balanceY(Position_Y);
}
...
if (Position_Y == 253 && Position_X == 253 && Flag_Stop == 0)
    Set_Pwm(Last_Target_Y, Last_Target_X);           // 丢球保持上一帧输出
```

---

## 2. 核心控制算法（control.c）

所有控制代码运行在 **TIM1 定时更新中断**（`TIM1_Int_Init(169, 7199)`，72MHz 时钟下约 17ms 周期）内，**中断驱动、非主循环轮询**。

### 2.1 主控流程 `TIM1_UP_IRQHandler`

```
读取新坐标(Position_X/Y)
PWM_adjust()      → 检测 PS2 手势，切换"调平模式/控制模式"
├─ 调平模式(Flash_Send==1): Balance_Set() 手调PWM，Set_Pwm(0,0)，不控制
└─ 控制模式:
   ├─ PS2 方向键 13~16 → 设定轨迹 Flag_Move
   ├─ Flag_Move==0 → Get_RC()       由 PS2 摇杆设定目标点(零位)
   │  Flag_Move!=0 → Setting_Move() 按预设轨迹运动
   ├─ Adjust()          → PS2 在线调 PID 参数
   ├─ 有球: Target = -balance(Position)   ← 核心 PD 控制
   ├─ 目标限幅 ±Max_Target(100)
   └─ Flag_Stop==0 → Set_Pwm(Target_Y, Target_X)   ← 输出
```

### 2.2 双轴 PD 控制 `balanceX` / `balanceY`

对 X 轴和 Y 轴**各自独立**地做平衡控制，结构完全相同。以 X 为例：

```c
int balanceX(float Angle) {
    Bias = Angle - Zero_X;              // 偏差 = 当前坐标 - 目标点(零位)
    Differential = Bias - Last_Bias;    // 偏差变化率(微分项)

    // 每 30 次(约510ms) 才做一次积分
    if (++Flag_Target > 30) {
        Flag_Target = 0;
        if (Flag_Stop == 0 && 检测到球) Integration += Bias;
        else Integration = 0;
        if (Integration < -500) Integration = -500;   // 积分限幅
        if (Integration >  500) Integration =  500;
        Balance_Integration = Integration * Balance_Ki;  // Ki=0.001
    }

    balance = Balance_Kp * Bias/100          // 比例项
            + Balance_Kd * Differential/10   // 微分项
            + Balance_Integration;           // 积分项(低频)
    Last_Bias = Bias;
    return balance;
}
```

**特点**：

- **P 项**：`Kp(48) × 偏差 / 100`，把小球坐标偏差线性映射为控制量。
- **D 项**：`Kd(98) × 偏差变化量 / 10`，利用相邻两次偏差的**差值**（非单位时间速率）提供阻尼，抑制超调。
- **I 项**：**低频积分**——每 30 个周期（约 510ms）才累加一次偏差，`Ki=0.001`，积分限幅 ±500。低频+小系数是为了消除**静差**（小球停在目标点附近缓慢回落），同时避免高频噪声累积。丢球或急停时积分清零。
- **输出即 PWM 变化量**：返回值为有符号整数，直接叠加到舵机 PWM 占空比上（见 `Set_Pwm`）。

### 2.3 双轴解耦输出 `Set_Pwm`

```c
void Set_Pwm(int motor_x, int motor_y) {
    TIM4->CCR3 = PWM3 - motor_x;   // X 轴舵机
    TIM4->CCR4 = PWM4 + motor_y;   // Y 轴舵机
}
```

- `PWM3`/`PWM4`（默认 740）是**机械水平零位**的 PWM 值，`motor_x/motor_y` 是控制输出，二者叠加得到实际舵机角度。
- X 轴取负、Y 轴取正，是**机械方向标定**（镜像）。
- 舵机由 TIM4 PWM 驱动（`TIM4_PWM_Init(9999, 143)`），PWM 周期固定，CCR 决定脉宽 → 舵机角度。

### 2.4 目标点设定（零位移动）

目标点用 `Zero_X/Zero_Y` 表示（即 PD 控制里的"期望坐标"）。有两种设定途径：

**① PS2 摇杆手动设定 `Get_RC()`**

```c
zero_x = ZERO_X + (PS2_LX - 128) * 0.3;   // 摇杆偏移 × 灵敏度0.3
zero_y = ZERO_Y + (PS2_LY - 128) * 0.3;
if (Zero_X < zero_x) Zero_X += 3;          // 以 Step=3 步进逼近目标
if (Zero_X > zero_x) Zero_X -= 3;
```

摇杆静止(128)时目标不变；移动时目标以**固定步长 3** 缓变，避免跳变。

**② 预设轨迹 `Setting_Move()`**：按 `count` 分段时间步进修改 `Zero_X/Zero_Y`，实现：
- 轨迹 1：**小三角形**（三段直线回原点）
- 轨迹 2：**圆形**（`Zero_Y=ZERO_Y+40*cos((count-40)/25)`、`Zero_X=ZERO_X+40*sin(...)` 参数方程）
- 轨迹 3：**四叶草/绕圈**（多段斜线组合）
- 轨迹 4：**大矩形/回字形**（多段垂直折线）

每段都是纯增量步进（如 `Zero_Y+=0.5`），计数到设定值后切段并复位 `count`。

### 2.5 在线调参（PS2 + 蓝牙 APP 双通道）

**PS2 `Adjust()`**：摇杆右摇杆偏移超过阈值 100，且在方向键 5~8 模式下，在线增减 `Balance_Kp`/`Balance_Kd`（X 摇杆调 Kp，Y 摇杆调 Kd）。

**蓝牙 APP（USART3，`usart2.c`）**：

- 协议：`0x7B ... 0x7D` 包裹的调参帧；`Receive[1]` 为参数索引（`0x30`→Kp，`0x31`→Kd），`Receive[3..]` 为 ASCII 数字串，按位加权还原为 `float`。
- 请求 PID（`Receive[3]==0x50`）时置 `PID_Send=1`，主程序里 `APP_Show()` 通过 `printf` 把当前 Kp/Kd 回传给 APP 显示。

### 2.6 板面水平校准与 Flash 存储

- **`PWM_adjust()`**：检测 PS2 左手柄左拨 + 右手柄右拨 → 进入调平模式（`Flash_Send=1`）；反向手势退出。
- **`Balance_Set()`**：调平模式下用 PS2 摇杆微调 `PWM3/PWM4` 使板面水平。
- **`Flash_Write/Read()`**：调平值（×100 存为整数）写入内部 Flash 地址 `0x0800E000`；上电读取，若全 `0xFFFF`（空/未校准）则回退默认 740。

### 2.7 启停控制

- **触摸开关**（EXTI3 下降沿中断）：触摸闭合（`INT==0`）→ `Flag_Stop=0` 启动控制。
- **按键双击**（`click_N_Double(25)`）：双击翻转 `Flag_Stop`，启停控制。
- `Flag_Stop==1` 时输出 0（舵机回中），控制暂停。

---

## 3. 任务调度与 50ms 精准延时

`delay_flag`/`delay_50` 机制（主循环）：

```c
delay_flag = 1;
delay_50 = 0;
while (delay_flag);          // 等待
```

TIM1 中断里（每 17ms 执行一次）：

```c
if (delay_flag == 1) {
    if (++delay_50 == 3) delay_50 = 0, delay_flag = 0;   // 3×17ms≈50ms
}
```

即用**定时器中断计数**实现约 50ms 的主循环节拍（每 3 个 17ms 中断释放一次主循环），使 PS2 采集、OLED 显示按固定节奏运行，避免占用大量 CPU 轮询。

---

## 4. 显示与上位机

- **OLED**（`show.c`）：12 号字体分 6 行显示 Servo 状态、PS2 各轴值、Kp/Kd、目标坐标 `Zero_X/Y` 与实时坐标 `Position_X/Y`（含正负号处理）。
- **DataScope**（`show.c` 预留接口）：用 USART1 发送 `Position_X/Y` 两通道数据到上位机波形显示，因串口 1 需接鲁班猫，功能被注释。

---

## 5. 参数清单与可移植要点

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `Balance_Kp` | 48 | 比例系数（除以 100 使用） |
| `Balance_Kd` | 98 | 微分系数（除以 10 使用） |
| `Balance_Ki` | 0.001 | 积分系数（每 30 周期累加，限幅 ±500） |
| `Zero_X/Zero_Y` | 128 | 目标点（期望坐标），ZERO_X=109/ZERO_Y=122 为宏 |
| `Max_Target` | 100 | 目标控制量限幅 |
| `PWM3/PWM4` | 740 | 板面水平零位舵机 PWM |
| 视觉帧 | `0x7B X Y 0x7D` | USART1 @115200 |
| 丢球标记 | 253/253 | 坐标无效标志 |
| 控制周期 | ~17ms | TIM1 更新中断（169,7199） |
| 主循环节拍 | ~50ms | 3 个中断周期 |

**移植到 MSPM0G3507 的启示**：

1. 控制核心是**定时中断里的 PD(+低频积分)**，与我们已在 `ball_control.c` 实现的 PD 思路一致，但本项目还额外引入了**双轴解耦 + 目标零位 + 丢球保持 + 低频积分**。
2. 视觉协议不同：参考项目用 `0x7B X Y 0x7D`；我们的 maixcam 用 `AA XL XH BB`。
3. 参考项目的积分策略（低频采样 + 小系数 + 死区外才累加）对消除静差有价值，后续可考虑在 MSPM0 端补一个低频积分项。
4. 丢球时"保持上一帧输出"的策略，与当前 maixcam 实现的"失帧停机"不同——如果机械上小球会快速滑走，保持输出可能更稳，可作为实验对比项。
