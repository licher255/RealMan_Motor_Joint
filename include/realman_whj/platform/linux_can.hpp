/**
 * @file linux_can.hpp
 * @brief RealMan WHJ Joint Motor Driver - Linux SocketCAN Implementation
 * @author RealMan Driver Team
 * @version 1.0.0
 * @date 2026-03-03
 */

#pragma once

#ifdef __linux__

#include "realman_whj/platform/can_interface.hpp"
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

namespace realman_whj {

/**
 * @brief Linux SocketCAN interface implementation
 * 
 * Uses Linux SocketCAN API (socket.h, can.h) for native CAN support.
 * Supports CAN FD frames with bit rate switching.
 */
class LinuxCANInterface : public ICANInterface {
public:
    LinuxCANInterface();
    ~LinuxCANInterface() override;

    // Delete copy/move
    LinuxCANInterface(const LinuxCANInterface&) = delete;
    LinuxCANInterface& operator=(const LinuxCANInterface&) = delete;
    LinuxCANInterface(LinuxCANInterface&&) = delete;
    LinuxCANInterface& operator=(LinuxCANInterface&&) = delete;

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
 * @brief Linux-specific SocketCAN configuration
 */
struct LinuxCANConfig {
    // Frame reception mode
    enum class RxMode {
        CALLBACK,       ///< Use callback for async reception
        POLLING,        ///< Manual polling
        THREADED        ///< Internal thread with callback
    };
    
    RxMode rx_mode{RxMode::THREADED};
    int rx_thread_priority{0};      ///< Real-time priority for RX thread (0=normal)
    
    // Socket options
    bool enable_loopback{false};    ///< Enable local loopback
    int receive_buffer_size{1024};  ///< Socket receive buffer size
    
    // For virtual CAN (vcan) testing
    bool is_virtual{false};
};

/**
 * @brief Configure and bring up a SocketCAN interface
 * 
 * This function uses iproute2 (ip link) to configure the interface.
 * Requires root privileges.
 * 
 * @param interface_name Interface name (e.g., "can0")
 * @param config Driver configuration with bitrates
 * @return true if configuration successful
 */
bool configureSocketCAN(const std::string& interface_name, 
                        const DriverConfig& config);

/**
 * @brief Bring up a SocketCAN interface
 */
bool bringUpInterface(const std::string& interface_name);

/**
 * @brief Bring down a SocketCAN interface
 */
bool bringDownInterface(const std::string& interface_name);

/**
 * @brief List available SocketCAN interfaces
 */
std::vector<std::string> listSocketCANInterfaces();

/**
 * @brief Create Linux CAN interface with specific configuration
 */
std::unique_ptr<ICANInterface> createLinuxCANInterface(
    const LinuxCANConfig& linux_config);

/**
 * @brief Check if SocketCAN supports CAN FD
 */
bool socketCANSupportsFD();

} // namespace realman_whj

#endif // __linux__
