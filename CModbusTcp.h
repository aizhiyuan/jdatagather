#ifndef _CMODBUSSOCKET_H_
#define _CMODBUSSOCKET_H_
#include "include/TcpSocket.h"
#include "include/SocketHandler.h"
#include "include/ListenSocket.h"
#include "DataGather.h"

// socket
class CModbusTcp : public TcpSocket
{
public:
    CModbusTcp(ISocketHandler &h);

    // rcv data
    void OnRawData(const char *pcBuf, size_t size);

private:
};

#endif
