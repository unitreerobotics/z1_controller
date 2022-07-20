#ifndef IOUDP_H
#define IOUDP_H

#include "interface/IOInterface.h"
#include "/usr/include/x86_64-linux-gnu/bits/floatn-common.h"

// #pragma pack(1)
// struct JointCmd{
//     _Float32 T;
//     _Float32 W;
//     _Float32 Pos;
//     _Float32 K_P;
//     _Float32 K_W;
// };

// struct JointState{
//     _Float32 T;
//     _Float32 W;
//     _Float32 Acc;
//     _Float32 Pos;
// };

// union SendCmd{
//     uint8_t checkCmd;
//     JointCmd jointCmd[7];
// };

// union RecvState{
//     uint8_t singleState;
//     uint8_t selfCheck[10];
//     JointState jointState[7];
//     uint8_t errorCheck[16];
// };
// #pragma pack()

class IOUDP : public IOInterface{
public:
    IOUDP(CmdPanel *cmdPanel);
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
};

#endif  // IOUDP_H