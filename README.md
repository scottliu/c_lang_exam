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
