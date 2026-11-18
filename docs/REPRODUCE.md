# 构建、样本与复现

## 先跑无需硬件的对照

依赖：Python 3.10+；Windows x64 MSVC Build Tools（C++桌面开发组件）。本轮使用Python 3.12环境与MSVC 19.44.35225，C11、`/O2`；无pip依赖。脚本可从Visual Studio安装器定位MSVC，也可在x64 Native Tools终端运行。代码保留`CC/cc/gcc/clang`路径，但Linux/macOS本轮未执行，不标为已验证平台。

在仓库根目录：

```sh
python tools/reproduce.py --out .build/replay
```

输出目录必须新建或为空，以免覆盖旧结果。全部编译文件、临时历史源码和编译日志只写到`--out`；成功后可读`.build/replay/result.json`。程序不调用下载器、BLE、SDK服务或生产接口。退出码非0表示构建、输入完整性、历史重建或判定失败；请读输出目录中的compiler.log，不把文件生成当通过。

工具实际执行以下步骤：

1. 校验[SOURCE.json](../SOURCE.json)内按LF归一的导出源码摘要和样本原始字节摘要，避免Git换行转换造成假失败。
2. 从当前策略应用 [R2 恢复补丁](../samples/variants/r2-bp_touch_policy.c.patch)，校验历史 R2 摘要，然后用[历史差异](../samples/variants/manifest.json)还原 v15/R1。只重建所需的策略文件。
3. 独立编译当前实现与三个历史版本，给 C 回放器输入同一段采集与配置，逐条比较当前实现与 R2 的带时间戳 HID 报告。
4. 各版本均运行 18 组停止/抬手测试，检查左键按下沿和最终键态，比较当前实现与 R2 的各项指标，并按 v15 基线追踪 R1/R2 的尾移。
5. 写出实际结果JSON；代码不以预存报告代替执行。

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

本轮已将原始回放器+原始输入，与公开回放器+脱敏输入做完整HID事件比较：时间原点平移后，v15的411个、R2的419个事件逐项相等。此项同时覆盖入口适配和归一化没有改变该样本输出。它不能证明所有未来输入都等价。

判定：metadata中的motion窗口v15为0、R2为16；button窗口v15为1、R2为0；最终buttons为0。101ms是该窗口起点至首次回放报告的差值，不是屏幕响应时间。HID报告次数不是位移像素数。

## 样本二：18项合成停止/尾迹反例

[release_tail.c](../tools/release_tail.c)复用历史测试，不伪装成真实传感器采样。生成：先移动、停住，再让压力代理下降并施加质心尾移；组合为2种帧间隔（5/10ms）×3种尾移（500/1000/1500µm）×3种持续时间（60/120/180ms）。使用[samples/release-tail/gesture.bin](../samples/release-tail/gesture.bin)固定96字节参数。

判定为**相对同条件v15新增尾移大于1 HID count**的用例数量：历史pressure R1为6，R2为0；同时最终按钮必须释放。不是要求绝对尾移为0，也不是证明18项代表所有真实抬手。

## 完整固件构建

本轮实际编译完成；条件是复用本机已安装SDK，并从候选之外提供已授权的IQS9151设置C文件。该文件由Azoteq设置头生成，重分发权利未确定，所以公开包故意不包含；无需它也能完成上述全部host回放。

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

产物：`<BuildDir>/firmware/zephyr/zephyr.elf`、`.hex`、`.bin`。本轮FLASH408612B、RAM228224B；结果与新build ID/镜像摘要见[实际执行记录](../validation/local-20261116.json)。只做编译，没有刷写、读设备或新运行验收；镜像不随公开包分发。

## 历史与本轮分别计

- **本轮实际执行：** 上述真实样本三版本、18合成反例、原始与公开HID事件等价、条件完整编译。
- **历史已有记录：** R2的17组真实输入回归；408612字节刷写读回一致；BLE运行检查。该17组未全部收录，不宣称本公开命令重跑了它们。
- **实现/定义：** 完整状态机还有大量路径，不因源码存在就宣称覆盖；参数与平台ABI限制如上。
- **尚待验证：** R2刷后的真实快划/微调/抬手/双指体验，以及本轮新build ID产物实机运行。

本候选未加入CI工作流和通过徽章。干净远端runner尚未实跑；未来可先接host回放，不将硬件体验转换成CI绿灯。重复执行请选择新输出目录；确认保留result.json后，可自行删除仅由本次命令生成的输出目录，不删除原始采样。
