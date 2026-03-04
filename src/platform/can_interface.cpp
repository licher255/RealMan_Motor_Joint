/**
 * @file can_interface.cpp
 * @brief Platform Abstract CAN Interface - Factory Implementation
 */

#include "realman_whj/platform/can_interface.hpp"

#ifdef _WIN32
    #include "realman_whj/platform/windows_can.hpp"
#else
    #include "realman_whj/platform/linux_can.hpp"
#endif

namespace realman_whj {

std::unique_ptr<ICANInterface> createCANInterface() {
#ifdef _WIN32
    return std::make_unique<WindowsCANInterface>();
#else
    return std::make_unique<LinuxCANInterface>();
#endif
}

std::vector<std::string> listAvailableInterfaces() {
    std::vector<std::string> interfaces;
    
#ifdef _WIN32
    auto adapters = listWindowsCANAdapters();
    for (const auto& [name, type] : adapters) {
        interfaces.push_back(name);
    }
#else
    interfaces = listSocketCANInterfaces();
#endif
    
    return interfaces;
}

bool isLinux() {
#ifdef __linux__
    return true;
#else
    return false;
#endif
}

bool isWindows() {
#ifdef _WIN32
    return true;
#else
    return false;
#endif
}

} // namespace realman_whj
