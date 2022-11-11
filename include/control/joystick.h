#ifndef _UNITREE_ARM_JOYSTICK_H_
#define _UNITREE_ARM_JOYSTICK_H_

#include <vector>
#include "message/udp.h"
#include "control/cmdPanel.h"
#include "message/joystick_common.h"
#include "message/aliengo_common.h"
#include "message/b1_common.h"
#include <math.h>

using namespace UNITREE_LEGGED_SDK_ALIENGO;
// using namespace UNITREE_LEGGED_SDK_B1;

class UnitreeJoystick : public CmdPanel{
public:
    UnitreeJoystick(std::vector<KeyAction*> events, 
        EmptyAction emptyAction, size_t channelNum = 1,
        double dt = 0.002)
        :  CmdPanel(events, emptyAction, channelNum, dt){
        _udp = new UDPPort("dog", "192.168.123.220", 8082, 8081, HIGH_STATE_LENGTH, BlockYN::NO, 500000);

        _udpCmd = {0};
        _udpState = {0};
        _readThread = new LoopFunc("JoyStickRead", 0.0, boost::bind(&UnitreeJoystick::_read, this));
        _runThread = new LoopFunc("CmdPanelRun", _dt, boost::bind(&UnitreeJoystick::_run, this));
        _readThread->start();
        _runThread->start();
    };
    ~UnitreeJoystick(){
        delete _udp;
        delete _runThread;
        delete _readThread;
        
    };
private:
    void _read(){
        _udp->send((uint8_t*)&_udpCmd, HIGH_CMD_LENGTH);
        _udp->recv((uint8_t*)&_udpState);

        #ifdef CTRL_BY_ALIENGO_JOYSTICK
        memcpy(&_keyData, _udpState.wirelessRemote, 40);
        #else
        memcpy(&_keyData, &_udpState.wirelessRemote[0], 40);
        #endif

        _extractCmd();
        _updateState();
    };
    void _extractCmd(){
        float joyThre = 0.5;    // 手柄数值范围 +-1

        if(((int)_keyData.btn.components.L1 == 1) &&
        ((int)_keyData.btn.components.L2 == 1)){
            _keyCmd.c = "l12";
        }else if((int)_keyData.btn.components.R2  == 1){
            _keyCmd.c = "r2";
            if((int)_keyData.btn.components.X  == 1){
                _keyCmd.c = "r2x";
            }
        }else if((int)_keyData.btn.components.R1  == 1){
            _keyCmd.c = "r1";
        }else if(fabs(_keyData.lx) > joyThre){
            _keyData.lx > joyThre?_keyCmd.c = "left_left":_keyCmd.c = "left_right";
        }else if(fabs(_keyData.ly) > joyThre){
            _keyData.ly > joyThre?_keyCmd.c = "left_up":_keyCmd.c = "left_down";
        }else if((int)_keyData.btn.components.up  == 1){
            _keyCmd.c = "up";
        }else if((int)_keyData.btn.components.down  == 1){
            _keyCmd.c = "down";
        }else if(fabs(_keyData.rx) > joyThre){
            _keyData.rx > joyThre?_keyCmd.c = "right_left":_keyCmd.c = "right_right";
        }else if(fabs(_keyData.ry) > joyThre){
            _keyData.ry > joyThre?_keyCmd.c = "right_up":_keyCmd.c = "right_down";
        }else if((int)_keyData.btn.components.Y  == 1){
            _keyCmd.c = "Y";
        }else if((int)_keyData.btn.components.A  == 1){
            _keyCmd.c = "A";
        }else if((int)_keyData.btn.components.X  == 1){
            _keyCmd.c = "X";
        }else if((int)_keyData.btn.components.B  == 1){
            _keyCmd.c = "B";
        }else if((int)_keyData.btn.components.right  == 1){
            _keyCmd.c = "left";
        }else if((int)_keyData.btn.components.left  == 1){
            _keyCmd.c = "right";
        }else if((int)_keyData.btn.components.up  == 1){
            _keyCmd.c = "up";
        }else if((int)_keyData.btn.components.down  == 1){
            _keyCmd.c = "down";
        }else if((int)_keyData.btn.components.select  == 1){
            _keyCmd.c = "select";
        }else{
            _releaseKeyboard();
            return;
        }

        _pressKeyboard();
    };

    xRockerBtnDataStruct _keyData;
    UDPPort *_udp;
    HighCmd _udpCmd;
    HighState _udpState;
};

#endif  // _UNITREE_ARM_JOYSTICK_H_
