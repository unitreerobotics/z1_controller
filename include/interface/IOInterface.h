#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include <string>
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "control/keyboard.h"
#include "common/math/robotics.h"

class IOInterface{
public:
    IOInterface(){}
    ~IOInterface(){
        delete lowCmd;
        delete lowState;
    };
    virtual bool sendRecv(const LowlevelCmd *cmd, LowlevelState *state) = 0;
    virtual bool calibration(){return false;};
    bool checkGripper(){return hasGripper;};
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    virtual bool isDisconnect(){ return false;};
    bool hasErrorState;
protected:
    bool hasGripper;
};

#endif  //IOINTERFACE_H