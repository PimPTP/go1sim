#pragma once
#include <arpa/inet.h>
#include <unistd.h>

class VisionUDP {
public:
    VisionUDP(int port = 9000)
    {
        sock = socket(AF_INET, SOCK_DGRAM, 0);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;

        bind(sock, (sockaddr*)&addr, sizeof(addr));
    }

    bool recv(float& x, float& y, float& z)
    {
        float buf[3];

        int n = recvfrom(sock,
                         buf,
                         sizeof(buf),
                         MSG_DONTWAIT,
                         nullptr,
                         nullptr);

        if(n != sizeof(buf))
            return false;

        x = buf[0];
        y = buf[1];
        z = buf[2];

        return true;
    }

private:
    int sock;
};