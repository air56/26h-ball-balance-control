# 26 电赛 H 题：车载平衡滚球运动控制系统

基于 **TI MSPM0G3507** 的摆杆平衡滚球闭环控制系统。摄像头识别小球像素坐标 → PD(+积分) 位置环 → 42 闭环步进电机通过齿轮齿条驱动摆杆，使钢球稳定在指定位置（0cm / ±5cm）。

## 系统架构

```
                    ┌─────────────────────────────────────────────┐
                    │                MSPM0G3507                   │
                    │                                             │
 maixcam2 ──UART1──►│  camera_uart.c   帧解析 (AA XH XL BB)       │
 视觉识别            │      │                                      │
 (YOLO/球形目标)     │      ▼                                      │
                    │  ball_control.c  PD+积分位置环(20ms)         │
                    │      │                                      │
 按键 key1/key2 ────►│  task.c  题目状态机(0cm / ±5cm)             │
                    │      │                                      │
                    │      ▼                                      │
                    │  Emm_V5.c / motor_parser.c                  │
                    │  UART0 ──► ZDT_X42S 闭环步进电机             │
                    │                (42 电机 + 齿轮齿条 → 摆杆)    │
                    │                                             │
                    └─────────────────────────────────────────────┘
```

- **视觉端**：`maixcam2` 脚本识别钢球中心，每帧经 UART 发送 `0xAA [X低] [X高] 0xBB`，115200 波特。
- **控制端**：`ball_control.c` 实现带积分与预测的 PD 控制，输出限步数位置模式指令驱动电机。
- **电机驱动**：`Emm_V5.c` 封装 ZDT_X42S 串口协议（Emm 固件，位置模式限步数，防止小球俯冲）。

## 外设型号清单

| 外设 | 型号 | 说明 |
| --- | --- | --- |
| 主控 MCU | TI **MSPM0G3507**（LQFP-64，Cortex-M0+） | 运行 `empty.syscfg` 配置，SDK `mspm0_sdk@2.10.00.04`，CCS Theia 开发 |
| 视觉识别模块 | Sipeed **MaixCam 2**（`maixcam2`） | MicroPython 脚本，YOLO 模型 `/root/models/xgq1/model_313989.mud`，识别钢球并通过 UART1（PA8/PA9）发送中心像素坐标 |
| 执行电机 | **ZDT_X42S 42 闭环步进电机**（Emm 固件，V5 协议） | 串口控制，16 细分（3200 脉冲/圈），最高 3000 RPM，默认波特 115200，校验码 0x6B |
| 传动机构 | 齿轮齿条 + 摆杆 | 电机经齿轮齿条驱动摆杆倾斜，总长 25cm，行程约 -11cm ~ +12cm |
| 显示模块 | **0.96 寸 SSD1306 OLED**（128×64，SPI 接口） | 软件 SPI（PB8~PB11/PB14），显示目标/角度/转速与 S/T/H 状态 |
| 按键 | 2 个轻触按键 | PA23(key1)/PA21(key2)，内部上拉，下降沿中断接入任务状态机 |
| 传感器 | ADC12 编码器通道（PA16，SysConfig 已配置） | 预留编码器接口（当前主循环未使用） |
| 电源/串口 | TTL 串口模块 | 调试观测，115200 |

> 注：`empty.syscfg` 中另配置了 I2C1（PB2/PB3）与 ADC12 编码器通道，当前控制闭环未启用，供扩展。

## 目录结构

```
├── empty.c            # 主程序：闭环主循环 + 任务状态机 + OLED 显示
├── empty.syscfg       # SysConfig 外设/引脚配置（MSPM0G3507）
├── ball_control.c/.h  # PD(+积分) 位置环，方向区分限速/阻尼/预测
├── camera_uart.c/.h   # maixcam UART 帧解析（0xAA 头 / 0xBB 尾）
├── Emm_V5.c/.h        # ZDT_X42S 闭环步进电机串口协议封装
├── motor_parser.c/.h  # 电机应答帧解析（实时位置/转速）
├── task.c/.h          # 题目状态机（0cm 稳定 / ±5cm 往返）
├── key/key.c          # 按键 GPIO 中断 + 软件防抖
├── oled/oled.c        # SSD1306 OLED 软件 SPI 驱动 + 字库
├── uart/uart.c        # 串口收发封装
├── delay/delay.c      # 延时
├── maixcam2           # MaixCam 2 视觉识别 + 坐标发送脚本
└── docs/              # 算法分析 / 调参 / 调试记录文档
```

## 使用方法

1. **构建**：用 CCS Theia 打开工程（`empty.syscfg` + MSPM0G3507），导入 `mspm0_sdk@2.10.00.04`，编译烧录。
2. **视觉端**：将 `maixcam2` 脚本部署到 MaixCam 2（模型已训练识别钢球），上电后经 UART1 发送坐标。
3. **接线**：电机接 UART0（PA31/PA28），摄像头接 UART1（PA8/PA9），OLED/按键按引脚表连接。
4. **操作**：
   - `key1`：启停 0cm 闭环；
   - `key2`：启动/退出题目 1（±5cm）流程；
   - OLED 第 4 行显示 S/T/H 三态（运行/到达/保持）。

## 控制算法

- 位置环：`Kp`（RPM/px）+ 方向区分的速度阻尼 `Kd`（`right_kd`/`left_kd`）+ 方向区分的最大转速（`right_max_rpm`/`left_max_rpm`）。
- 积分：`ki_scaled` + 积分限幅，消除稳态偏差。
- 预测：`predict_px` 将当前偏差外推 N 帧，实现提前 0.5s 级调节。
- 执行端：位置模式限步数（`MOTOR_PULSES_STEP`，40 脉冲/100ms），防止电机急转导致小球俯冲。

全部可调参数集中在 `empty.c` 的 `ball_config`，注释标明各参数作用与调参经验。

## 参考资料

- 控制算法参考：平衡小车之家「视觉板球系统」STM32 源码（见 `docs/STM32视觉板球系统源码算法分析.md`）
- 电机协议：`ZDT_X42S_串口控制总结.md`
- 调参/调试经验：`电机控制速查.md`、`调试问题记录.md`

## License

[MIT](LICENSE) © 2026 Rin
