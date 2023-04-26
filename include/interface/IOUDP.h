#ifndef IOUDP_H
#define IOUDP_H

#include "interface/IOInterface.h"

class IOUDP : public IOInterface{
public:
    IOUDP(const char* IP, uint port, size_t timeOutUs = 100000, bool showInfo = true);
    ~IOUDP();

    bool sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
    bool calibration();
    bool isDisconnect(){ return _ioPort->isDisConnect;}
private:
    IOPort *_ioPort;

    UDPSendCmd _cmd;
    UDPRecvState _state;
    UDPRecvStateOld _stateOld;
    size_t _motorNum;
    size_t _jointNum;
    uint8_t _singleState;
    uint8_t _selfCheck[10];
};

#endif  // IOUDP_H