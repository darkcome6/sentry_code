# CAN 口"打不开"排查记录与结论

> 日期：2026-09-01
> 平台：Jetson Orin NX Developer Kit（内核 5.15.185-tegra）
> 结论：**2pin CAN 口 CANH/CANL 极性接反**，对调后链路正常

---

## 一、问题现象

- `./run_sentry.sh real` 启动时报：`RTNETLINK answers: Device or resource busy`、`Cannot find device "can1"`
- `setting.sh` 无论成败都打印 `[OK]`，误导判断
- 接口显示 `ERROR-PASSIVE (berr-counter tx 128 rx 0)` 或一发帧立即 **BUS-OFF（tx 248）**
- `candump can0` 永远收不到电机反馈帧（电机侧用 USB 转 CAN 分析器却能正常收到）

## 二、硬件 / 软件环境

| 项 | 值 |
|----|----|
| 板卡 | NVIDIA Jetson Orin NX Developer Kit（Engineering Reference Developer Kit） |
| 内核 | 5.15.185-tegra |
| CAN 控制器 | mttcan（`nvidia,tegra194-mttcan`），支持 CAN FD（当前用 Classic CAN 1M） |
| 设备树节点 | `mttcan@c310000`（=can0，okay）、`mttcan@c320000`（=can1，**disabled**，本板无 can1） |
| 当前 boot DTB | `/boot/kernel_no_dma_uart.dtb`（extlinux LABEL no-dma-uart） |
| CAN 收发器 | 板载（P3768 类载板），2pin 差分输出 |

## 三、根因与解决方案（最终结论）

```
根因：NX 的 2pin CAN 口是标准差分 CAN（有板载收发器），
      但接线时 CANH / CANL 接反了。
```

- **现象特征**：极性反 → 发帧必然 BUS-OFF（tx 248）、收不到任何数据
- **解决**：把 2pin 口两根线对调（CANH↔CANH、CANL↔CANL），对调后立即恢复
- 验证：`candump can0` 持续刷出电机反馈帧（如 `can0 20A [8] 0F 01 00 00 FF ...`）
- 官方口径：NVIDIA 工程师多次强调 **CAN_H 必须接 CAN_H、CAN_L 必须接 CAN_L**

> ⚠️ 以后接线请先确认极性；若 `candump` 突然又收不到且一发帧就 BUS-OFF，优先怀疑极性。

## 四、排查过程（关键命令 + 结论）

### 1. 看接口与控制器状态
```bash
ip -details link show can0
# 关键看：state UP / can state / bitrate / berr-counter
```
- `ERROR-PASSIVE (tx 128 rx 0)`：控制器想发但总线无对端 ACK，错误计数累积

### 2. 监听总线 / 测试发送
```bash
candump can0 -n 10 -T 5000     # 监听 5 秒，收 10 帧
cansend can0 123#DEADBEEF      # 测试发送
```
- 发送后 `ip -details link show can0` 变 `BUS-OFF (tx 248)` → 物理层无对端 ACK

### 3. 收发统计定位
```bash
cat /sys/class/net/can0/statistics/tx_packets
cat /sys/class/net/can0/statistics/rx_packets
cat /sys/class/net/can0/statistics/tx_errors
```
- 发送帧后 `tx_errors` 不涨但 `rx_packets` 涨 → 往往处于 **LOOPBACK 模式**（见坑 1）

### 4. 收发器使能脚检查（P3768 类载板）
```bash
cat /sys/kernel/debug/gpio | grep -E "PAA.0[45]|gpio-32[01]"
```
| 脚 | 功能 | 正常状态 |
|----|------|---------|
| PAA.04 (gpio-320) | CAN0_STB | LOW（normal mode） |
| PAA.05 (gpio-321) | CAN0_EN | HIGH（enabled） |
- 本板：STB 本为 LOW、EN 被 regulator 占用但 out hi → **使能脚正常，非根因**

### 5. 内部回环测试（验证控制器本身）
```bash
ip link set can0 down
ip link set can0 up type can bitrate 1000000 loopback on
cansend can0 123#DEADBEEF
candump can0 -n 1              # 应收到自己回环的帧
# 测完务必关闭回环：
ip link set can0 down
ip link set can0 type can loopback off
ip link set can0 up type can bitrate 1000000 restart-ms 100
```

### 6. 万用表快速判定 2pin 信号类型（未接任何设备、can0 up）
| 测量 | 差分 CAN（有收发器） | TTL（无收发器） |
|------|------|------|
| 两脚对 GND 电压 | 都 ≈ 2.5V | ≈ 3.3V 或 0V |
| 两脚之间电阻 | ≈ 60Ω | 高阻/开路 |
- 本板测出为差分 CAN → 问题锁定为接线极性

## 五、本次顺手修复的脚本 bug

| 文件 | 问题 | 修复 |
|------|------|------|
| `run_sentry.sh` | 第一行 shebang 缺 `#`（`!/bin/bash`），每次运行报 `行 1: !/bin/bash: 没有那个文件或目录` | 改为 `#!/bin/bash` |
| `setting.sh` | 无论成败都打印 `[OK]`；末尾混入一段 `pkill` 清理代码；can1 不存在还假装成功 | 重写：检测接口存在性 / UP / 错误状态，失败明确 `[FAIL]`，不存在的接口 `[SKIP]`，支持 `restart-ms 100` |

## 六、自查命令速查表

```bash
ip -details link show can0                       # 接口/控制器状态
candump can0                                     # 持续监听总线
cansend can0 123#DEADBEEF                        # 发送测试
cat /sys/class/net/can0/statistics/*_errors      # 错误统计
sudo dmesg | grep -i -E "mttcan|can"             # 内核日志
./setting.sh can0                                # 复位/重开 can0
```

## 七、注意事项 / 坑

1. **mttcan 的 loopback 坑**：`down`/`up` 不会清除 `loopback` 模式，`ip -details` 里带 `<LOOPBACK>` 时收不到总线帧。测试完回环必须显式 `ip link set can0 type can loopback off`。
2. **本板无 can1**：设备树 `mttcan@c320000` 为 `disabled`，系统只注册 can0。需要 can1 需改 DTB 并重启（本次单电机验证用不到）。
3. **波特率**：当前 can0 = 1Mbps（Classic CAN）。大疆/DM 电机默认 1M，匹配；不要对电机开 CAN FD。
4. **BUS-OFF 是极性错误的典型信号**：一发送就 BUS-OFF（tx 248）≈ 总线无对端 ACK ≈ 优先查极性和接线。
5. sudo 密码：`spr`（可用 `echo 'spr' | sudo -S` 缓存）。

## 八、相关参考

- NVIDIA 论坛：CAN Communication Failed on Jetson Orin NX Developer Kit — BUS-OFF Despite Correct Configuration（topic 371556，现象与本例完全一致）
