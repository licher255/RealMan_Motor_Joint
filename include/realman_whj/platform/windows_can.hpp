/**
 * @file windows_can.hpp
 * @brief RealMan WHJ Joint Motor Driver - Windows CAN Interface Implementation
 * @author RealMan Driver Team
 * @version 1.0.0
 * @date 2026-03-03
 * 
 * Supports USB-CAN adapters through Windows CAN API or third-party libraries.
 * Currently supports: ZLG CAN (周立功), Kvaser, PEAK PCAN, and generic USB-CAN.
 */

#pragma once

#ifdef _WIN32

#include "realman_whj/platform/can_interface.hpp"
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

namespace realman_whj {

/**
 * @brief Windows CAN interface implementation
 * 
 * This implementation uses Windows CAN API or vendor-specific SDKs.
 * It supports multiple USB-CAN adapter brands commonly used in China.
 */
class WindowsCANInterface : public ICANInterface {
public:
    WindowsCANInterface();
    ~WindowsCANInterface() override;

    // Delete copy/move
    WindowsCANInterface(const WindowsCANInterface&) = delete;
    WindowsCANInterface& operator=(const WindowsCANInterface&) = delete;
    WindowsCANInterface(WindowsCANInterface&&) = delete;
    WindowsCANInterface& operator=(WindowsCANInterface&&) = delete;

    // ICANInterface implementation
    bool init(const std::string& interface_name, const DriverConfig& config) override;
    void close() override;
    bool send(const CANFDFrame& frame) override;
    void registerCallback(CANRxCallback callback) override;
    bool isOpen() const override;
    std::string getInterfaceName() const override;
    std::string getLastError() const override;
    void setReceiveTimeout(uint32_t timeout_ms) override;
    bool receive(CANFDFrame& frame) override;

private:
    // Implementation details (PIMPL idiom)
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};

/**
 * @brief CAN adapter types supported on Windows
 */
enum class CANAdapterType {
    AUTO,           ///< Auto-detect
    ZLG,            ///< ZLG CAN (周立功) - USBCAN-II, USBCAN-E-U, etc.
    KVASER,         ///< Kvaser CAN
    PEAK_PCAN,      ///< PEAK PCAN
    VECTOR,         ///< Vector CAN
    SOCKETCAN_GW,   ///< SocketCAN gateway over TCP/UDP
    SIMULATION      ///< Simulation mode for testing
};

/**
 * @brief Windows-specific configuration
 */
struct WindowsCANConfig {
    CANAdapterType adapter_type{CANAdapterType::AUTO};
    int device_index{0};        ///< Device index for multi-channel adapters
    int channel_index{0};       ///< Channel index (0=CAN1, 1=CAN2)
    bool use_fd{true};          ///< Enable CAN FD mode
    bool use_brs{true};         ///< Enable Bit Rate Switching
    
    // ZLG-specific
    uint32_t zlg_device_type{4};  // USBCAN-2A = 4
    
    // For SocketCAN gateway
    std::string gateway_ip{"192.168.1.10"};
    uint16_t gateway_port{20001};
};

/**
 * @brief Create Windows CAN interface with specific adapter type
 */
std::unique_ptr<ICANInterface> createWindowsCANInterface(
    const WindowsCANConfig& win_config);

/**
 * @brief List available CAN adapters on Windows
 */
std::vector<std::pair<std::string, CANAdapterType>> listWindowsCANAdapters();

} // namespace realman_whj

#endif // _WIN32
