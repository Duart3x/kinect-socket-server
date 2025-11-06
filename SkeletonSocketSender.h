// Licensed under the MIT License.

#pragma once

#include <k4abt.h>
#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

class SkeletonSocketSender
{
public:
    SkeletonSocketSender(const std::string& host = "127.0.0.1", int port = 8888);
    ~SkeletonSocketSender();

    // Initialize the socket connection
    bool Initialize();

    // Send skeleton data as JSON
    bool SendSkeletonData(const k4abt_body_t& body, uint64_t timestamp);

    // Close the connection
    void Close();

 // Check if connected
    bool IsConnected() const;

private:
    std::string CreateJsonFromSkeleton(const k4abt_body_t& body, uint64_t timestamp);
    const char* GetJointName(int jointId) const;

    std::string m_host;
    int m_port;
    SOCKET m_socket;
    bool m_initialized;
    bool m_connected;
};
