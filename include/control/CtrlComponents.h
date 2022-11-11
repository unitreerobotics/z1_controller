#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include <string>
#include <iostream>
#include "common/utilities/loop.h"
#include "message/arm_common.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/utilities/CSVTool.h"
#include "model/ArmModel.h"
#include "interface/IOUDP.h"
#include "interface/IOROS.h"
#include "control/armSDK.h"

using namespace std;

struct CtrlComponents{
public:
    CtrlComponents();
    ~CtrlComponents();

    std::string armConfigPath;
    CmdPanel *cmdPanel;
    IOInterface *ioInter;
    ArmModel *armModel;
    CSVTool *stateCSV;

    SendCmd sendCmd; // cmd that receive from SDK 
    RecvState recvState;// state that send to SDK
    
    //config
    double dt;
    bool *running;
    Control ctrl;
    bool hasGripper;
    bool isCollisionOpen;
    double collisionTLimit;
    bool isPlot;

    void geneObj();
    void writeData();
private:
    std::string ctrl_IP;
    uint ctrl_port;
    double _loadWeight;
};

#endif  // CTRLCOMPONENTS_H