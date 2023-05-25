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
#include "control/armSDK.h"
#include "model/unitree_gripper.h"

using namespace std;

struct CtrlComponents{
public:
    CtrlComponents(int argc, char**argv);
    ~CtrlComponents();

    std::string armConfigPath;
    CmdPanel *cmdPanel;
    IOInterface *ioInter;
    Z1Model *armModel;
    CSVTool *stateCSV;
    std::shared_ptr<Unitree_Gripper> gripper;
    double gripper_max_tau = 10;

    SendCmd sendCmd; // cmd that receive from SDK 
    RecvState recvState;// state that send to SDK
    
    //config
    double dt;
    bool *running;
    Control ctrl = Control::SDK;
    bool hasGripper;
    bool isCollisionOpen;
    double collisionTLimit;
    bool isPlot;
    int trajChoose = 1;
    size_t armType = 36;
    std::string ctrl_IP;
    uint ctrl_port;
    
    void geneObj();
    void writeData();
private:
    void configProcess(int argc, char** argv);

    double _loadWeight;
};

#endif  // CTRLCOMPONENTS_H