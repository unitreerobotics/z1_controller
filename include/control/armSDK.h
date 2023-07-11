#ifndef _SDK_H
#define _SDK_H

#include "message/udp.h"
#include "control/cmdPanel.h"

class ARMSDK : public CmdPanel{
public:
    ARMSDK(std::vector<KeyAction*> events, 
        EmptyAction emptyAction, const char* sdkIP, uint sdkPort, uint ownPort, double dt = 0.002);
    ~ARMSDK();
    SendCmd getSendCmd();
    int getState(size_t channelID = 0);
    void setRecvState(RecvState& recvState);
    void start();
private:
    void _sendRecv();
    void _read(){};
    UDPPort *_udp;
    SendCmd _sendCmd, _sendCmdTemp;
    RecvState _recvState;
    size_t _recvLength;
};

#endif