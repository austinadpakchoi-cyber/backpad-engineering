# 架构与硬件接口

## 版本边界

本分支以 2026-11-01 的 v0.20.16 pressure-continuity R2 为基础，修改了意图窗口中清理过期样本的方式。历史 R2 的完整源码 ID 为 `1ef290e5f908b07023b844b98ee0786ad732f1902652f1a56c84a9517289de71`。[SOURCE.json](../SOURCE.json) 记录历史导出，回放工具依据上下文补丁重建 R2，并核对原始摘要。

目标为现有BackPad V2触控板（IQS9151）与nRF52840主控；不是IQS550参考版，也不是后续IQS9150 R3板或11月V06外壳已装配验收版本。13Rx×12Tx、4.5mm间距的首末电极中心距离为54.0×49.5mm，供坐标比例换算；这是感应几何，不是整机外形尺寸。

## 接口级说明

以[设备树](../firmware/app/boards/backpad/backpad.dts)及[驱动定义](../firmware/sensor9151/include/bp_iqs9151.h)为代码依据：

| 信号/设置 | 代码中的配置 | 边界 |
|---|---|---|
| MCU | nRF52840 QIAA；目标`backpad/nrf52840` | 非通用开发板镜像。 |
| SDA / SCL | P0.04 / P0.05；I2C fast mode 400kHz | 400kHz为总线配置，不是触控/端到端响应率。 |
| RDY | P0.03，开漏低有效；触控板上拉 | 事务在有效RDY窗口内进行；窗口外读到0xEE不能作为触点。 |
| IQS9151地址 | 7-bit `0x56`；16-bit寄存器地址低字节先发 | 以实际芯片/板版核对，不泛化到其他IQS芯片。 |
| LED / 按键 | P0.26低有效 / P0.02上拉低有效 | 用于本板，不提供未经核对的FPC物理针脚顺序。 |
| 应用Flash | `[0, 0xF8000)` | 后32KiB为storage；构建不等于刷写授权。 |
| NVS | `[0xF8000, 0x100000)`；配对存储 | 不公开设备NVS/配对密钥或Flash转储。 |
| HID报告 | buttons + signed 8-bit dx/dy/wheel，共4字节 | [编码器](../firmware/app/vendor/v5/src/bp_hid.c)限制按钮和相对增量；不是USB触控协议实现。 |
| BLE参数 | 配置偏好15ms连接间隔 | 实际协商值及手机响应另测，不能宣传15ms端到端延迟。 |

供电、电池、FPC方向、PCB连线及机械公差没有随源码开放。请勿仅凭该接口表对未知板接电或刷写。硬件设计文件权利核实与实物版本确认后再单独发布。

## 实际数据流

1. **采样与传感通信。** `bp_iqs9151_zephyr.c`处理RDY与I2C事务；`bp_iqs9151.c`解码72字节传感寄存器块并拒绝无效帧。`bp_sensor_service.c`保留采集时刻、会话/序号，调用`bp_touch_convert`。
2. **触点策略。** [bp_touch_policy.c](../firmware/app/src/bp_touch_policy.c)的`bp_touch_convert`将芯片坐标与触点strength/area转换为核心样本，执行准入、接触身份、滤波和意图判定。文中的“压力”是电容触点信号的代理，没有标定为力或压强。
3. **意图与输出分离。** `intent_observe`观察有效轨迹，输出可冻结而意图仍积累；`pressure_safe`表明当前严格拟合，`was_active`区分正在移动与仅历史确认过移动。恢复锚定最后输出位置，不一次释放冻结期间位移。
4. **手势核心。** [bp_pointer.c](../firmware/app/vendor/v5/src/bp_pointer.c)的`bp_process`、`bp_tick`、`bp_flush`处理tap/hold/drag、双指与滚动、时限与队列。R2重点修改触点策略，不借本次整理重写手势核心。
5. **BLE输出。** [main.c](../firmware/app/src/main.c)把核心输出交给`bp_channel_send`，`submit`经`bp_hid_encode`和`bt_hids_inp_rep_send_userdata`提交，完成事件回到通道。背压/连接状态与指针状态不是同一个变量。
6. **离线观测。** [replay.c](../tools/replay.c)把保存的36字节帧内传感内容重建成输入，调用同一个`bp_touch_convert → bp_process → bp_tick → bp_flush`；sink记录HID报告，不连接BLE。它从干净状态开始，因此不假定采样开始前设备全部内部状态已还原。

## 构建适配与核心保持

CMake 接收外置设置文件和生成的 build ID。设备端与回放端共用 C 手势算法。本分支比较当前实现、历史 v15、R1、R2；GPU 仅在开发机上生成用于重放的输入。详见[技术方案](TECHNICAL-SOLUTION.md)。
