
# 云台控制器通信协议V1.0

## 1. 协议概述

本文档描述了上位机（运行Linux的主机）与STM32微控制器之间的云台控制器通信协议。该协议基于串口通信（UART），暂用于获取云台的日志数据和设置云台的角速度。

## 2. 通信参数

- 波特率：921600
- 数据位：8位
- 停止位：1位
- 奇偶校验：无
- 通信方式：全双工异步串行通信

## 3. 数据帧格式

### 3.1 发送帧（上位机 → STM32）

发送帧包含帧头、长度、类型、命令和数据字段。

#### 3.1.1 帧头结构
```cpp
#pragma pack(push, 1)
struct GimbalTxFrameHeader
{
    uint16_t header;      // 帧头固定为0x5A89
    uint16_t length;      // 数据总长度（包括头部和数据部分）
    uint8_t type;         // 帧类型: 1-读请求, 2-写请求
    uint8_t cmd;          // 操作类型: 1-获取日志数据, 2-设置云台角速度
};
#pragma pack(pop)
```

#### 3.1.2 发送数据包格式
```
[帧头(6字节)] + [数据部分]
```

### 3.2 接收帧（STM32 → 上位机）

接收帧包含帧头和数据字段。

#### 3.2.1 帧头结构
```cpp
#pragma pack(push, 1)
struct GimbalRxFrameHeader
{
    uint16_t header; // 帧头固定为0x5A89
    uint16_t length; // 数据总长度（包括头部和数据部分）
};
#pragma pack(pop)
```

## 4. 操作命令定义

### 4.1 帧类型（FrameType）
```cpp
enum class FrameType : uint8_t {
    READ_REQUEST = 1,   // 读请求
    WRITE_REQUEST = 2   // 写请求
};
```

### 4.2 操作命令（CmdType）
```cpp
enum class CmdType : uint8_t {
    GET_LOG_DATA = 1,           // 获取日志数据
    SET_GIMBAL_ANGLE_VELOCITY = 2,  // 设置云台角速度
    GET_GIMBAL_ANGLE = 3          // 获取云台角度
};
```

## 5. 具体功能实现

### 5.1 获取云台日志数据（GET_LOG_DATA）

- **帧类型**: READ_REQUEST (1)
- **命令**: GET_LOG_DATA (1)
- **数据长度**: 6字节
- **请求格式**: 
  ```
  [0x89][0x5A][0x00][0x06][0x01][0x01]
  ```
- **GimbalLogData结构体数据结构**:
  ```cpp
  struct GimbalLogData {
      uint64_t timestamp;        // 时间戳（微秒）
      float pitch;               // 俯仰电机的角度
      float pitch_omega;         // 俯仰电机的角速度
      float pitch_omega_fbk;     // 俯仰电机的角速度反馈
      float yaw;                 // 偏航电机的角度
      float yaw_omega;           // 偏航电机的角速度
      float yaw_omega_fbk;       // 偏航电机的角速度反馈
  };
  ```
- **STM32响应数据格式**: GimbalRxFrameHeader + GimbalLogData结构体数据
  ```
  [帧头(4字节)] + [GimbalLogData结构体数据(32字节)]
  ```

### 5.2 获取云台角度 (GET_GIMBAL_ANGLE)
#### 

- **帧类型**: READ_REQUEST (1)
- **命令**: GET_GIMBAL_ANGLE (3)
- **数据长度**: 6字节
- **请求格式**: 
  ```
  [0x89][0x5A][0x00][0x06][0x01][0x03]
  ```

- **GimbalAngle结构体数据结构**:
  ```cpp
  struct GimbalAngle {
      float pitch;   // 俯仰角度
      float yaw;     // 偏航角度
      float roll;    // 滚转角度
  };

- **STM32响应数据格式**: GimbalRxFrameHeader + GimbalAngle结构体数据
  ```
  [帧头(4字节)] + [GimbalAngle结构体数据(12字节)]
  ```

### 5.3 设置云台角速度（SET_GIMBAL_ANGLE_VELOCITY）

- **帧类型**: WRITE_REQUEST (2)
- **命令**: SET_GIMBAL_ANGLE_VELOCITY (2)
- **数据长度**: 6 + 12字节（6字节帧头 + 12字节GimbalAngleVelocity结构体数据）
- **STM32需要接收的数据格式**:
  ```cpp
  struct GimbalAngleVelocity {
      float pitch;   // 俯仰角速度
      float yaw;     // 偏航角速度
      float roll;    // 滚转角速度
  };
  ```

- **请求格式**:
  ```
  [帧头(6字节)] + [GimbalAngleVelocity结构体数据(12字节)]
  ```
  总共16字节数据。

## 6. 数据类型说明

- `uint16_t`: 无符号16位整数，小端序（Little Endian）
- `uint64_t`: 无符号64位整数，小端序（Little Endian）
- `float`: IEEE 754标准32位浮点数，小端序（Little Endian）

## 7. 注意事项

1. 所有数值类型均采用小端序（Little Endian）格式传输。
2. 帧头固定为0x5A89（在内存中的存储顺序为0x89, 0x5A）。
4. STM32对于写操作只需要接收数据，不需要返回数据。
5. STM32对于读操作，STM32需要返回相应的数据。
6. 云台角速度发送频率为50ms，即每50ms发送一次数据。
7. 云台日志数据发送频率为2ms，即每2ms发送一次数据。
