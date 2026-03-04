/**
 * @file can_interface.hpp
 * @brief RealMan WHJ Joint Motor Driver - Platform Abstract CAN Interface
 * @author RealMan Driver Team
 * @version 1.0.0
 * @date 2026-03-03
 */

#pragma once

#include "realman_whj/core/types.hpp"
#include <functional>
#include <string>
#include <vector>
#include <memory>

namespace realman_whj {

/**
 * @brief CAN message reception callback type
 */
using CANRxCallback = std::function<void(const CANFDFrame& frame)>;

/**
 * @brief Platform abstract CAN interface
 * 
 * This interface abstracts the underlying CAN hardware to provide
 * a unified API for Linux (SocketCAN) and Windows (USB-CAN adapters).
 */
class ICANInterface {
public:
    virtual ~ICANInterface() = default;

    /**
     * @brief Initialize the CAN interface
     * 
     * @param interface_name Interface identifier (e.g., "can0" on Linux, "0" on Windows)
     * @param config Driver configuration including bitrates
     * @return true if initialization successful
     */
    virtual bool init(const std::string& interface_name, 
                      const DriverConfig& config) = 0;
    
    /**
     * @brief Close the CAN interface
     */
    virtual void close() = 0;

    /**
     * @brief Send a CAN FD frame
     * 
     * @param frame Frame to send
     * @return true if successfully queued/sent
     */
    virtual bool send(const CANFDFrame& frame) = 0;

    /**
     * @brief Register a callback for received frames
     * 
     * @param callback Function to call when a frame is received
     */
    virtual void registerCallback(CANRxCallback callback) = 0;

    /**
     * @brief Check if interface is open and ready
     */
    virtual bool isOpen() const = 0;
    
    /**
     * @brief Get interface name
     */
    virtual std::string getInterfaceName() const = 0;
    
    /**
     * @brief Get last error message
     */
    virtual std::string getLastError() const = 0;

    /**
     * @brief Set receive timeout (for blocking receive)
     * @param timeout_ms Timeout in milliseconds
     */
    virtual void setReceiveTimeout(uint32_t timeout_ms) = 0;
    
    /**
     * @brief Receive a single frame (blocking with timeout)
     * 
     * @param frame Output frame
     * @return true if frame received, false on timeout/error
     */
    virtual bool receive(CANFDFrame& frame) = 0;
};

/**
 * @brief Factory function to create platform-specific CAN interface
 * 
 * @return std::unique_ptr<ICANInterface> Platform-specific implementation
 */
std::unique_ptr<ICANInterface> createCANInterface();

/**
 * @brief List available CAN interfaces on the system
 * 
 * @return std::vector<std::string> List of available interface names
 */
std::vector<std::string> listAvailableInterfaces();

/**
 * @brief Check if running on Linux
 */
bool isLinux();

/**
 * @brief Check if running on Windows
 */
bool isWindows();

} // namespace realman_whj
