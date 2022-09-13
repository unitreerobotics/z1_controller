#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "unitree_arm_sdk/cmdPanel.h"
#include "unitree_arm_sdk/keyboard.h"
#include <string>
#include "common/math/robotics.h"

class IOInterface{
public:
    IOInterface(CmdPanel *cmdPanel):_cmdPanel(cmdPanel){}

    virtual ~IOInterface(){delete _cmdPanel;};
    virtual bool sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;

    // void zeroCmdPanel(){_cmdPanel->setZero();}
    std::string getString(std::string slogan){return _cmdPanel->getString(slogan);}
    std::vector<double> stringToArray(std::string slogan){return _cmdPanel->stringToArray(slogan);}
    std::vector<std::vector<double> > stringToMatrix(std::string slogan){return _cmdPanel->stringToMatrix(slogan);}
    virtual bool calibration(){return false;};

    SendCmd getSendCmd(){return _cmdPanel->getSendCmd();};
    void getRecvState(RecvState recvState){ _cmdPanel->getRecvState(recvState);};

    bool isDisConnect = false;// udp disconnection
    bool _isDisConnect[7];// joint(motor) disconnection
protected:
    uint16_t _isDisConnectCnt[7];
    CmdPanel *_cmdPanel;
};

#endif  //IOINTERFACE_H