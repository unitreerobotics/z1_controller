#ifndef IOUDP_H
#define IOUDP_H

#include "interface/IOInterface.h"
// #include "/usr/include/x86_64-linux-gnu/bits/floatn-common.h"

class IOUDP : public IOInterface{
public:
    IOUDP(CmdPanel *cmdPanel, const char* IP, bool hasGripper);
    ~IOUDP();
    bool sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
    // void setGripper(int position, int force);
    // void getGripper(int &position, int &force);

    bool calibration();
private:
    IOPort *_ioPort;

    UDPSendCmd _cmd;
    UDPRecvState _state;

    size_t _motorNum;
    size_t _jointNum;

    uint8_t _singleState;
    uint8_t _selfCheck[10];
};

#endif  // IOUDP_H