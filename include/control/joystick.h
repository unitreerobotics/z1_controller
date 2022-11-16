#ifndef _UNITREE_ARM_JOYSTICK_H_
#define _UNITREE_ARM_JOYSTICK_H_

#include <vector>
#include "control/cmdPanel.h"
#include "message/joystick_common.h"
#include "message/aliengo_common.h"
#include "message/b1_common.h"
#include <math.h>
#include "message/udp.h"

class UnitreeJoystick : public CmdPanel{
public:
    UnitreeJoystick( size_t dogType, std::vector<KeyAction*> events, 
        EmptyAction emptyAction, size_t channelNum = 1,
        double dt = 0.002);
    ~UnitreeJoystick();
private:
    void _read();
    void _extractCmd();
    size_t _dogType;
    xRockerBtnDataStruct _keyData;
    UDPPort *_udp;
    UNITREE_LEGGED_SDK_ALIENGO::HighCmd _udpCmdAliengo;
    UNITREE_LEGGED_SDK_ALIENGO::HighState _udpStateAliengo;
    UNITREE_LEGGED_SDK_B1::HighCmd _udpCmdB1;
    UNITREE_LEGGED_SDK_B1::HighState _udpStateB1;
};

#endif  // _UNITREE_ARM_JOYSTICK_H_
