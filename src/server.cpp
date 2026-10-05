#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>

int main()
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in receiver{};
    receiver.sin_family = AF_INET;
    receiver.sin_port = htons(5000);

    inet_pton(AF_INET, "192.168.0.195", &receiver.sin_addr);

    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};

    ssize_t n = sendto(
        sock,
        data,
        sizeof(data),
        0,
        (sockaddr*)&receiver,
        sizeof(receiver));

    if (n < 0)
        perror("sendto");
    else
        std::cout << "Sent " << n << " bytes\n";

    close(sock);
}