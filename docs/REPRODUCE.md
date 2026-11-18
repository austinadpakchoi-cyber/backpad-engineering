# 构建、样本与复现

## 先跑无需硬件的对照

Windows 复现需要 Python 3.10+ 和 x64 MSVC Build Tools（C++ 桌面开发组件）。脚本使用 C11 和 `/O2`，可从 Visual Studio 安装器定位 MSVC，也可在 x64 Native Tools 终端运行。GitHub 源码 ZIP 解压后即可运行，Python 端只使用标准库。`CC/cc/gcc/clang` 路径也已保留，当前公开验证使用 Windows x64。

在仓库根目录：

```sh
python tools/reproduce.py --out .build/replay
```

输出目录必须新建或为空，以免覆盖旧结果。全部编译文件、临时历史源码和编译日志只写到`--out`；成功后可读`.build/replay/result.json`。程序不调用下载器、BLE、SDK服务或生产接口。退出码非0表示构建、输入完整性、历史重建或判定失败；请读输出目录中的compiler.log，不把文件生成当通过。

工具实际执行以下步骤：

1. 校验样本原始字节摘要。用 [R2 源码归档差异](../samples/variants/r2-export-overrides.zip) 与公开源码重建历史 R2，只归档后续修改过的 5 个文件。按 LF 归一后逐文件校验 [SOURCE.json](../SOURCE.json) 记录的历史摘要。
2. 对归档的 R2 策略应用[历史差异](../samples/variants/manifest.json)还原 v15 和 R1。三个历史回放使用归档的指针代码和头文件，当前回放使用现行固件代码。
3. 独立编译当前实现与三个历史版本，给 C 回放器输入同一段采集与配置。当前版在压力下降窗口必须恢复移动且不出现按键；全样本的按键和滚轮报告与 R2 一致。完整 HID 报告的比较结果单独记录。
4. 各版本均运行 18 组停止/抬手测试。逐组检查左键按下与最终释放，按 v15 基线追踪 R1、R2 和当前版的尾移。当前版须满足与 R2 相同的停止/抬手判定阈值。
5. 实际运行结果写入 `result.json`。当前 1 mm PLA 固件的原型验证另见[连续性验证摘要](../validation/pla-continuity-20261118.json)。

需要单独改变接触强度时，使用[可选的 CUDA 或 CPU 灵敏度扫描](TECHNICAL-SOLUTION.md#复现与资料)。

## 样本一：真实压力下降失败片段

路径：[samples/pressure-drop](../samples/pressure-drop/metadata.json)。这是维护者在现有IQS9151双板上操作所得的1600帧采样，采样固件为v0.20.15-intent-gain-r1；不是pressure-continuity R1。输入来自历史6751失败记录。它是独立的定点失败采样，**不在历史17组既有回归列表里**。

| 文件 | 内容 |
|---|---|
| capture.bin | 128字节头 + 1600×48字节记录，共76928字节。 |
| touch.bin | 20字节触点准入参数。 |
| filter.bin | 24字节滤波参数。 |
| gesture.bin | 96字节指针/手势参数。无配对密钥、设备地址或登录信息。 |
| metadata.json | 时间窗口、版本、来源类别、摘要和判定条件。 |

脱敏仅改变：未使用头字段清零，保留回放使用的分辨率/电极映射；相对时间起点改为1000ms、序号从1开始、会话ID改为1。每帧36字节传感内容保留，时间间隔、序号间隔不变。未发布浏览器用户操作文本、屏幕内容、设备身份、机器路径或完整调试记录。

原始记录格式：头部offset20为u32帧数；offset64开始为回放读取的几何配置；每帧依次为u32时间、u32序号、u32会话、36字节传感内容。所有整数按历史小端格式读取。参数文件按本工程C结构ABI解释；本轮在Windows x64编译并由`sizeof(bp_config)==96`约束，未证明任意架构ABI通用。

历史导出时已将原始回放器和输入与公开回放器和脱敏输入逐条对照。时间原点平移后，v15 的 411 条、R2 的 419 条 HID 报告各与历史版本逐项相等。当前实现用同一输入独立回放，完整报告数见 `current_hid_event_comparison`。

判定：metadata中的motion窗口v15为0、R2为16；button窗口v15为1、R2为0；最终buttons为0。101ms是该窗口起点至首次回放报告的差值，不是屏幕响应时间。HID报告次数不是位移像素数。

## 样本二：18项合成停止/尾迹反例

[release_tail.c](../tools/release_tail.c)复用历史测试，不伪装成真实传感器采样。生成：先移动、停住，再让压力代理下降并施加质心尾移；组合为2种帧间隔（5/10ms）×3种尾移（500/1000/1500µm）×3种持续时间（60/120/180ms）。使用[samples/release-tail/gesture.bin](../samples/release-tail/gesture.bin)固定96字节参数。

统计**相对同条件 v15 新增尾移大于 1 HID count** 的用例数量。历史 pressure R1 为 6、R2 为 0，当前版的通过条件为 0。所有用例都必须维持松开状态。

## 完整固件构建

完整固件编译需要 nRF Connect SDK 和取得使用授权的 IQS9151 设置 C 文件。设置由厂商工具生成，需自行准备。上述主机回放只读取仓库内的样本与参数。

已使用版本：

| 依赖 | 本轮版本 / 固定提交 |
|---|---|
| nRF Connect SDK | 3.4.0-lts；nrf `99553055607b2e9885fbc80ccd11fa9da81c2df0` |
| Zephyr | 4.4.0；`bf801e4e3d19e1ffa76164346480cb7734dd2800` |
| Nordic工具链目录标识 | `dcbdc366a1`；GNU Arm 14.3.0，Zephyr SDK 1.0.1 |
| 构建工具 | CMake4.2.1、west1.5.0、SDK Python3.12.4、Ninja |
| 目标 | `backpad/nrf52840`，`--no-sysbuild` |

设置环境变量指向自己已安装/获授权的内容，再在PowerShell运行：

```powershell
./tools/build_firmware.ps1 -SdkRoot $env:NCS_ROOT -ToolchainRoot $env:NCS_TOOLCHAIN_ROOT -SettingsFile $env:BACKPAD_SETTINGS_FILE -BuildDir "$PWD/.build/firmware"
```

脚本沿用原工程的工具链环境，实际调用：

```text
python -m west build --no-sysbuild -b backpad/nrf52840 -s <repo>/firmware/app -d <BuildDir>/firmware -- -DUSER_CACHE_DIR=<BuildDir>/cache -DBACKPAD_SENSOR_SETTINGS=<authorized-settings.c> -DBACKPAD_GENERATED_INCLUDE=<BuildDir>/generated
```

设置文件须与R2传感配置一致：13Rx×12Tx、对应电极映射、分辨率及原调参；不能拿任意同名文件冒充已复现版本。公开包不自动下载、生成或伪造厂商设置。`tools/source_id.py`把实际公开固件文件与外置设置一起绑定为新build ID，因此本轮镜像不冒用历史已刷写固件身份。

产物位于 `<BuildDir>/firmware/zephyr/`，包括 `zephyr.elf`、`zephyr.hex` 和 `zephyr.bin`。历史导出构建记录见[本机构建记录](../validation/local-20261116.json)，现行 1 mm PLA 版本的构建及刷写核对见[连续性验证摘要](../validation/pla-continuity-20261118.json)。

## 验证范围

- **公开主机回放**覆盖一个真实压力下降样本和 18 项合成释放尾迹，在历史版和当前版上分别运行。
- **历史 R2 记录**另含 17 组真实输入回归和本机硬件检查。公开包保留选定样本及已有指标。
- **1 mm PLA 原型记录**包含团队内部的分动作实采、固件回放和板端读回摘要。用户设备上的短划体验还在继续验证。

每次执行使用新输出目录。需要精简产物时，可保留 `result.json` 并清理本次命令创建的编译目录。
