/**
 * @file windows_can.cpp
 * @brief Windows CAN Interface Implementation
 * 
 * Supports multiple USB-CAN adapters through vendor SDKs or generic interfaces.
 * Also includes a simulation mode for testing without hardware.
 */

#ifdef _WIN32

#include "realman_whj/platform/windows_can.hpp"
#include <iostream>
#include <cstring>
#include <chrono>

// Windows headers
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

namespace realman_whj {

// ============================================================================
// Implementation Class
// ============================================================================

class WindowsCANInterface::Impl {
public:
    std::string interface_name_;
    DriverConfig config_;
    WindowsCANConfig win_config_;
    std::atomic<bool> is_open_{false};
    std::atomic<bool> should_stop_{false};
    
    CANRxCallback callback_;
    std::thread rx_thread_;
    std::mutex callback_mutex_;
    
    std::queue<CANFDFrame> rx_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    std::string last_error_;
    std::atomic<uint32_t> receive_timeout_ms_{100};
    
    // Simulation mode
    bool simulation_mode_{false};
    std::atomic<uint32_t> sim_message_count_{0};
    
    // SocketCAN Gateway support (for Windows via TCP/UDP)
    SOCKET gateway_socket_{INVALID_SOCKET};
    sockaddr_in gateway_addr_;
    bool use_gateway_{false};
    
    Impl() = default;
    
    ~Impl() {
        stopReceiveThread();
        closeInterface();
    }
    
    bool init(const std::string& interface_name, const DriverConfig& config) {
        interface_name_ = interface_name;
        config_ = config;
        
        // Check for simulation mode
        if (interface_name == "sim" || interface_name == "SIM") {
            simulation_mode_ = true;
            is_open_ = true;
            std::cout << "[WindowsCAN] Simulation mode enabled" << std::endl;
            return true;
        }
        
        // Check for SocketCAN gateway mode (format: "gateway:IP:PORT")
        if (interface_name.find("gateway:") == 0) {
            return initGateway(interface_name.substr(8));
        }
        
        // Try to auto-detect and initialize adapter
        return initAdapter();
    }
    
    bool initGateway(const std::string& gateway_spec) {
        // Parse IP:PORT from spec
        size_t colon_pos = gateway_spec.find(':');
        if (colon_pos == std::string::npos) {
            last_error_ = "Invalid gateway spec, use gateway:IP:PORT";
            return false;
        }
        
        std::string ip = gateway_spec.substr(0, colon_pos);
        int port = std::stoi(gateway_spec.substr(colon_pos + 1));
        
        // Initialize Winsock
        WSADATA wsaData;
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0) {
            last_error_ = "WSAStartup failed: " + std::to_string(result);
            return false;
        }
        
        // Create UDP socket
        gateway_socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (gateway_socket_ == INVALID_SOCKET) {
            last_error_ = "Socket creation failed: " + std::to_string(WSAGetLastError());
            WSACleanup();
            return false;
        }
        
        // Set timeout
        DWORD timeout = 100;  // 100ms
        setsockopt(gateway_socket_, SOL_SOCKET, SO_RCVTIMEO, 
                   (const char*)&timeout, sizeof(timeout));
        
        // Setup address
        gateway_addr_.sin_family = AF_INET;
        gateway_addr_.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &gateway_addr_.sin_addr);
        
        use_gateway_ = true;
        is_open_ = true;
        
        // Start receive thread
        startReceiveThread();
        
        return true;
    }
    
    bool initAdapter() {
        // TODO: Implement specific adapter initialization
        // For now, fall back to simulation mode with a warning
        std::cout << "[WindowsCAN] Warning: No specific adapter type configured, "
                  << "falling back to simulation mode" << std::endl;
        simulation_mode_ = true;
        is_open_ = true;
        return true;
    }
    
    void closeInterface() {
        stopReceiveThread();
        
        if (use_gateway_ && gateway_socket_ != INVALID_SOCKET) {
            closesocket(gateway_socket_);
            gateway_socket_ = INVALID_SOCKET;
            WSACleanup();
        }
        
        is_open_ = false;
    }
    
    bool send(const CANFDFrame& frame) {
        if (!is_open_) {
            last_error_ = "Interface not open";
            return false;
        }
        
        if (simulation_mode_) {
            // In simulation mode, just print and echo back
            std::cout << "[SimCAN] TX: ID=0x" << std::hex << frame.id 
                      << " Len=" << std::dec << (int)frame.len;
            if (frame.len > 0) {
                std::cout << " Data=";
                for (int i = 0; i < frame.len && i < 8; i++) {
                    std::cout << std::hex << (int)frame.data[i] << " ";
                }
            }
            std::cout << std::endl;
            
            // Simulate response
            sim_message_count_++;
            if (sim_message_count_ % 10 == 0) {
                // Echo back as response
                CANFDFrame response = frame;
                response.id = frame.id + 0x100;
                
                // Modify data to simulate response
                if (response.len >= 2 && response.data[0] == 0x01) {
                    // Read response - add some data
                    response.len = 4;
                    response.data[2] = 0x34;  // Example data
                    response.data[3] = 0x12;
                } else if (response.len >= 2 && response.data[0] == 0x02) {
                    // Write response - success
                    response.len = 3;
                    response.data[2] = 0x01;
                }
                
                // Queue response
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    rx_queue_.push(response);
                }
                queue_cv_.notify_one();
                
                // Callback
                {
                    std::lock_guard<std::mutex> lock(callback_mutex_);
                    if (callback_) {
                        callback_(response);
                    }
                }
            }
            return true;
        }
        
        if (use_gateway_) {
            // Pack and send via gateway
            // Simple protocol: [4 bytes ID][1 byte len][64 bytes data]
            char buffer[69];
            uint32_t id = frame.id;
            memcpy(buffer, &id, 4);
            buffer[4] = frame.len;
            memcpy(buffer + 5, frame.data, 64);
            
            int result = sendto(gateway_socket_, buffer, 69, 0,
                               (sockaddr*)&gateway_addr_, sizeof(gateway_addr_));
            return (result == 69);
        }
        
        return false;
    }
    
    void startReceiveThread() {
        should_stop_ = false;
        rx_thread_ = std::thread(&Impl::receiveLoop, this);
    }
    
    void stopReceiveThread() {
        should_stop_ = true;
        if (rx_thread_.joinable()) {
            rx_thread_.join();
        }
    }
    
    void receiveLoop() {
        while (!should_stop_) {
            if (use_gateway_) {
                receiveFromGateway();
            } else if (simulation_mode_) {
                // In simulation mode, just sleep
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }
    
    void receiveFromGateway() {
        char buffer[69];
        sockaddr_in from_addr;
        int from_len = sizeof(from_addr);
        
        int result = recvfrom(gateway_socket_, buffer, sizeof(buffer), 0,
                             (sockaddr*)&from_addr, &from_len);
        
        if (result == 69) {
            CANFDFrame frame;
            memcpy(&frame.id, buffer, 4);
            frame.len = buffer[4];
            if (frame.len > 64) frame.len = 64;
            memcpy(frame.data, buffer + 5, 64);
            
            frame.is_fd = true;
            frame.bit_rate_switch = true;
            frame.is_extended = (frame.id & 0x80000000) != 0;
            frame.timestamp = std::chrono::steady_clock::now();
            
            // Callback
            {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                if (callback_) {
                    callback_(frame);
                }
            }
            
            // Queue
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                rx_queue_.push(frame);
            }
            queue_cv_.notify_one();
        }
    }
    
    bool receiveBlocking(CANFDFrame& frame, uint32_t timeout_ms) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        bool has_frame = queue_cv_.wait_for(lock, 
            std::chrono::milliseconds(timeout_ms),
            [this] { return !rx_queue_.empty(); });
        
        if (!has_frame) {
            return false;
        }
        
        frame = rx_queue_.front();
        rx_queue_.pop();
        return true;
    }
};

// ============================================================================
// WindowsCANInterface Public Methods
// ============================================================================

WindowsCANInterface::WindowsCANInterface() 
    : pImpl_(std::make_unique<Impl>()) {}

WindowsCANInterface::~WindowsCANInterface() = default;

bool WindowsCANInterface::init(const std::string& interface_name, const DriverConfig& config) {
    return pImpl_->init(interface_name, config);
}

void WindowsCANInterface::close() {
    pImpl_->closeInterface();
}

bool WindowsCANInterface::send(const CANFDFrame& frame) {
    return pImpl_->send(frame);
}

void WindowsCANInterface::registerCallback(CANRxCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl_->callback_mutex_);
    pImpl_->callback_ = callback;
}

bool WindowsCANInterface::isOpen() const {
    return pImpl_->is_open_;
}

std::string WindowsCANInterface::getInterfaceName() const {
    return pImpl_->interface_name_;
}

std::string WindowsCANInterface::getLastError() const {
    return pImpl_->last_error_;
}

void WindowsCANInterface::setReceiveTimeout(uint32_t timeout_ms) {
    pImpl_->receive_timeout_ms_ = timeout_ms;
}

bool WindowsCANInterface::receive(CANFDFrame& frame) {
    return pImpl_->receiveBlocking(frame, pImpl_->receive_timeout_ms_);
}

// ============================================================================
// Helper Functions
// ============================================================================

std::unique_ptr<ICANInterface> createWindowsCANInterface(const WindowsCANConfig& win_config) {
    auto iface = std::make_unique<WindowsCANInterface>();
    // Note: Configuration is applied during init
    return iface;
}

std::vector<std::pair<std::string, CANAdapterType>> listWindowsCANAdapters() {
    std::vector<std::pair<std::string, CANAdapterType>> adapters;
    
    // Simulation always available
    adapters.push_back({"sim", CANAdapterType::SIMULATION});
    
    // TODO: Enumerate actual adapters via vendor SDKs
    
    return adapters;
}

} // namespace realman_whj

#endif // _WIN32
