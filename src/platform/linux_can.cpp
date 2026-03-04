/**
 * @file linux_can.cpp
 * @brief Linux SocketCAN Implementation
 */

#ifdef __linux__

#include "realman_whj/platform/linux_can.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <poll.h>

// CAN FD support check
#ifndef CAN_RAW_FD_FRAMES
    #define CAN_RAW_FD_FRAMES 5
#endif

namespace realman_whj {

// ============================================================================
// Implementation Class
// ============================================================================

class LinuxCANInterface::Impl {
public:
    int sock_{-1};
    std::string interface_name_;
    DriverConfig config_;
    LinuxCANConfig linux_config_;
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
    
    Impl() = default;
    
    ~Impl() {
        stopReceiveThread();
        closeSocket();
    }
    
    bool init(const std::string& interface_name, const DriverConfig& config) {
        interface_name_ = interface_name;
        config_ = config;
        
        // Create socket
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0) {
            last_error_ = "Failed to create CAN socket: " + std::string(strerror(errno));
            return false;
        }
        
        // Enable CAN FD if requested
        if (config_.can_fd_bitrate > 0) {
            int enable_fd = 1;
            if (setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 
                          &enable_fd, sizeof(enable_fd)) < 0) {
                last_error_ = "Failed to enable CAN FD: " + std::string(strerror(errno));
                closeSocket();
                return false;
            }
        }
        
        // Get interface index
        struct ifreq ifr;
        strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
            last_error_ = "Failed to get interface index: " + std::string(strerror(errno));
            closeSocket();
            return false;
        }
        
        // Bind socket
        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        
        if (bind(sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            last_error_ = "Failed to bind socket: " + std::string(strerror(errno));
            closeSocket();
            return false;
        }
        
        // Set non-blocking mode for polling
        int flags = fcntl(sock_, F_GETFL, 0);
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
        
        is_open_ = true;
        
        // Start receive thread if using threaded mode
        if (linux_config_.rx_mode == LinuxCANConfig::RxMode::THREADED) {
            startReceiveThread();
        }
        
        return true;
    }
    
    void closeSocket() {
        stopReceiveThread();
        
        if (sock_ >= 0) {
            ::close(sock_);
            sock_ = -1;
        }
        is_open_ = false;
    }
    
    bool send(const CANFDFrame& frame) {
        if (!is_open_ || sock_ < 0) {
            last_error_ = "Socket not open";
            return false;
        }
        
        if (frame.is_fd) {
            // Use CAN FD frame
            struct canfd_frame fd_frame;
            memset(&fd_frame, 0, sizeof(fd_frame));
            
            fd_frame.can_id = frame.id;
            if (frame.is_extended) {
                fd_frame.can_id |= CAN_EFF_FLAG;
            }
            
            fd_frame.len = frame.len;
            memcpy(fd_frame.data, frame.data, frame.len);
            
            if (frame.bit_rate_switch) {
                fd_frame.flags |= CANFD_BRS;
            }
            
            ssize_t nbytes = ::write(sock_, &fd_frame, sizeof(fd_frame));
            if (nbytes < 0) {
                last_error_ = "Failed to send CAN FD frame: " + std::string(strerror(errno));
                return false;
            }
        } else {
            // Use classic CAN frame
            struct can_frame can_frame;
            memset(&can_frame, 0, sizeof(can_frame));
            
            can_frame.can_id = frame.id;
            if (frame.is_extended) {
                can_frame.can_id |= CAN_EFF_FLAG;
            }
            
            can_frame.can_dlc = frame.len > 8 ? 8 : frame.len;
            memcpy(can_frame.data, frame.data, can_frame.can_dlc);
            
            ssize_t nbytes = ::write(sock_, &can_frame, sizeof(can_frame));
            if (nbytes < 0) {
                last_error_ = "Failed to send CAN frame: " + std::string(strerror(errno));
                return false;
            }
        }
        
        return true;
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
        struct pollfd fds[1];
        fds[0].fd = sock_;
        fds[0].events = POLLIN;
        
        while (!should_stop_) {
            int ret = poll(fds, 1, 10);  // 10ms timeout
            
            if (ret < 0) {
                if (errno != EINTR) {
                    last_error_ = "Poll error: " + std::string(strerror(errno));
                }
                continue;
            }
            
            if (ret == 0) {
                continue;  // Timeout
            }
            
            if (fds[0].revents & POLLIN) {
                receiveAndProcess();
            }
        }
    }
    
    void receiveAndProcess() {
        struct canfd_frame fd_frame;
        struct can_frame* can_frame = (struct can_frame*)&fd_frame;
        
        ssize_t nbytes = ::read(sock_, &fd_frame, sizeof(fd_frame));
        
        if (nbytes < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                last_error_ = "Read error: " + std::string(strerror(errno));
            }
            return;
        }
        
        // Determine if it's CAN FD
        bool is_fd = (nbytes == CANFD_MTU);
        
        CANFDFrame frame;
        
        if (is_fd) {
            frame.id = fd_frame.can_id & CAN_EFF_MASK;
            frame.is_extended = (fd_frame.can_id & CAN_EFF_FLAG) != 0;
            frame.is_fd = true;
            frame.bit_rate_switch = (fd_frame.flags & CANFD_BRS) != 0;
            frame.len = fd_frame.len;
            memcpy(frame.data, fd_frame.data, fd_frame.len);
        } else {
            frame.id = can_frame->can_id & CAN_EFF_MASK;
            frame.is_extended = (can_frame->can_id & CAN_EFF_FLAG) != 0;
            frame.is_fd = false;
            frame.bit_rate_switch = false;
            frame.len = can_frame->can_dlc;
            memcpy(frame.data, can_frame->data, can_frame->can_dlc);
        }
        
        frame.timestamp = std::chrono::steady_clock::now();
        
        // Call registered callback
        {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            if (callback_) {
                callback_(frame);
            }
        }
        
        // Add to queue for blocking receive
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            rx_queue_.push(frame);
        }
        queue_cv_.notify_one();
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
    
    std::string getLastError() const {
        return last_error_;
    }
};

// ============================================================================
// LinuxCANInterface Public Methods
// ============================================================================

LinuxCANInterface::LinuxCANInterface() 
    : pImpl_(std::make_unique<Impl>()) {}

LinuxCANInterface::~LinuxCANInterface() = default;

bool LinuxCANInterface::init(const std::string& interface_name, const DriverConfig& config) {
    return pImpl_->init(interface_name, config);
}

void LinuxCANInterface::close() {
    pImpl_->closeSocket();
}

bool LinuxCANInterface::send(const CANFDFrame& frame) {
    return pImpl_->send(frame);
}

void LinuxCANInterface::registerCallback(CANRxCallback callback) {
    std::lock_guard<std::mutex> lock(pImpl_->callback_mutex_);
    pImpl_->callback_ = callback;
}

bool LinuxCANInterface::isOpen() const {
    return pImpl_->is_open_;
}

std::string LinuxCANInterface::getInterfaceName() const {
    return pImpl_->interface_name_;
}

std::string LinuxCANInterface::getLastError() const {
    return pImpl_->getLastError();
}

void LinuxCANInterface::setReceiveTimeout(uint32_t timeout_ms) {
    pImpl_->receive_timeout_ms_ = timeout_ms;
}

bool LinuxCANInterface::receive(CANFDFrame& frame) {
    return pImpl_->receiveBlocking(frame, pImpl_->receive_timeout_ms_);
}

// ============================================================================
// Helper Functions
// ============================================================================

bool configureSocketCAN(const std::string& interface_name, const DriverConfig& config) {
    // Build ip link command
    std::string cmd = "ip link set " + interface_name + " up type can bitrate " 
                     + std::to_string(config.can_bitrate);
    
    if (config.can_fd_bitrate > 0) {
        cmd += " dbitrate " + std::to_string(config.can_fd_bitrate) + " fd on";
    }
    
    int ret = system(cmd.c_str());
    return (ret == 0);
}

bool bringUpInterface(const std::string& interface_name) {
    std::string cmd = "ip link set " + interface_name + " up";
    int ret = system(cmd.c_str());
    return (ret == 0);
}

bool bringDownInterface(const std::string& interface_name) {
    std::string cmd = "ip link set " + interface_name + " down";
    int ret = system(cmd.c_str());
    return (ret == 0);
}

std::vector<std::string> listSocketCANInterfaces() {
    std::vector<std::string> interfaces;
    
    FILE* fp = popen("ip -o link show | grep -E 'can|vcan' | awk -F': ' '{print $2}'", "r");
    if (fp) {
        char buffer[128];
        while (fgets(buffer, sizeof(buffer), fp) != nullptr) {
            // Remove newline
            size_t len = strlen(buffer);
            if (len > 0 && buffer[len-1] == '\n') {
                buffer[len-1] = '\0';
            }
            interfaces.push_back(buffer);
        }
        pclose(fp);
    }
    
    return interfaces;
}

std::unique_ptr<ICANInterface> createLinuxCANInterface(const LinuxCANConfig& linux_config) {
    auto iface = std::make_unique<LinuxCANInterface>();
    // Note: Configuration is applied during init
    return iface;
}

bool socketCANSupportsFD() {
#ifdef CANFD_MTU
    return true;
#else
    return false;
#endif
}

} // namespace realman_whj

#endif // __linux__
