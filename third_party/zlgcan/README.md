# ZLG CAN Driver

周立功 (ZLG) CAN FD 设备驱动库

## 文件说明

| 路径 | 说明 |
|------|------|
| `x64/zlgcan.dll` | 64位 DLL (用于64位 Python/C++) |
| `x86/zlgcan.dll` | 32位 DLL (用于32位 Python/C++) |
| `include/*.h` | C/C++ 头文件 |

## 支持的设备

- USBCANFD-100U-mini (设备类型: 43)
- USBCANFD-100U
- USBCANFD-200U
- USBCANFD-800U
- USBCANFD-400U
- 其他 ZLG CAN FD 设备

## 使用方法

### Python

```python
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType

driver = ZlgCanDriver()  # 自动检测 DLL 路径
driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
```

### C++

```cpp
// 在 CMakeLists.txt 中添加
link_directories(${PROJECT_SOURCE_DIR}/third_party/zlgcan/x64)
target_link_libraries(your_target zlgcan)
```

## 驱动来源

- 版本: 2023.07.28
- 来源: ZLG 官方驱动包
- 原始路径: `aa1c7-main/zlgcanDev(2023.07.28)/`

## 注意事项

1. 使用64位 Python 时必须使用 `x64/zlgcan.dll`
2. 使用32位 Python 时必须使用 `x86/zlgcan.dll`
3. 确保 `kerneldlls` 目录与 DLL 在同一目录下
