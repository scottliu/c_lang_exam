# c_lang_exam

## ping 程序说明

目录 [ping/ping.c](ping/ping.c) 实现了一个基于 ICMP 的 ping 工具，能够向目标主机或 IP 地址发送 Echo 请求并获取响应结果。

### 功能特点
- 支持向指定主机或 IP 地址发起 ping
- 支持设置发送次数、间隔时间和超时时间
- 可在 Windows 和 Linux/macOS 平台上编译运行
- 通过 ICMP Echo 请求和回复实现基本的网络连通性测试

### 命令行参数
- `-c <次数>`：设置发送次数
- `-i <间隔毫秒>`：设置发送间隔
- `-t <超时毫秒>`：设置等待超时时间
- `<主机名或 IP>`：目标地址

### 编译方法

#### Linux / macOS
```bash
cd ping
gcc -o ping ping.c
./ping 127.0.0.1
```

#### Windows
```bash
gcc -o ping.exe ping.c -lws2_32 -liphlpapi
ping.exe 127.0.0.1
```

> 说明：Linux/macOS 下需要使用 root 或 sudo 权限运行，因为程序使用原始 ICMP Socket。

## mvcs_door 目录说明

[目录 mvcs_door/](mvcs_door/) 是基于三菱车辆控制系统（MVCS）与门操作员系统通信协议文档
（WDR502215E h 第 4.6 / 4.7 章节）实现的 C 语言协议栈。

### 协议要点
- 物理层：RS485 三线半双工，NRZi 编码，38.4 kbps ±0.1%，轮询周期 20 ms（80 bytes/20 ms）
- 数据链路层：HDLC 兼容帧
  `Opening PAD(4B) + Flag(7EH) + Address1/Upper + Address2/Lower + Control + Information + CRC(2B) + Flag(7EH)`
- 地址：设备类型 `13H`(CDR, MVCS→门) / `83H`(CD, 门→MVCS) / `FFH`(全选)；门号 `01H~04H`
- 信息字段：CDR 20 字节（发车测试请求、BCD 时间、开门命令位图等）
  CD 36 字节（发车测试判定/测量值、门状态、内部信息、故障信息、程序版本/校验和等）
- CRC：CCITT-1 多项式 `x16+x12+x5+1`（0x1021，初值 0xFFFF，HDLC 约定）
- 半双工发送使能：opening PAD 前 ≥1 bit 使能，closing flag 后 ≤6 bit 内释放

### 文件
- `mvcs_door.h`：协议常量、CDR/CD 字段结构体、API 声明
- `mvcs_door.c`：CRC-CCITT、CDR/CD 编解码、HDLC 帧构建（含 bit-stuffing）、逐字节接收状态机
- `mvcs_door_demo.c`：自测/演示（CRC 向量校验、收发往返、20 ms 轮询模拟、错误注入）
- `mvcs_door_monitor.c`：串口线路监听工具（Windows / Linux 跨平台）

### 编译方法
```bash
cd mvcs_door
clang -Wall -Wextra -O2 -o mvcs_door_demo mvcs_door_demo.c mvcs_door.c
./mvcs_door_demo

clang -Wall -Wextra -O2 -o mvcs_door_monitor mvcs_door_monitor.c mvcs_door.c
./mvcs_door_monitor COM3            # Windows: 监听串口 COM3
./mvcs_door_monitor /dev/ttyUSB0    # Linux: 监听 RS485 总线
./mvcs_door_monitor --file cap.bin  # 离线回放抓包字节流
./mvcs_door_monitor --selftest      # 内置帧回放自测
```

### 监听工具用法
```
mvcs_door_monitor <串口设备> [-b 波特率] [-v] [-o 日志] [--no-log]
mvcs_door_monitor --file <字节流文件> [-v]        # 离线分析
mvcs_door_monitor --selftest                     # 自测
```
监听程序读取总线字节流，复用接收状态机解析 HDLC 帧，按设备类型区分
CDR（MVCS→门）/ CD（门→MVCS），解码并打印发车测试、测量值（按 0.1 s/bit、
0.05 A/bit、5 mm/bit 换算）、门状态、内部信息、故障信息与程序版本/校验和。
`-v` 额外打印每帧原始字节；串口数据位/校验/停止位文档未明确，默认 8N1。

**日志**：每次运行自动把监听结果写入日志文件，控制台与日志同步输出。
默认文件名 `mvcs_monitor_YYYYMMDD_HHMMSS.log`（每次运行一个新文件，追加写入，
不覆盖已有内容），可用 `-o <路径>` 指定文件，`--no-log` 关闭日志。

> 说明：`mvcs_door.c` 中的 CRC 初值/字节序为 HDLC 常用约定，文档未明确给出；
> 若与既有系统对接，需按对方实现调整 `MVCS_CRC_INIT` 与 CRC 字节序。
