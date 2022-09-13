#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/IOROS.h"
#include "model/ArmDynKineModel.h"
#include "common/utilities/CSVTool.h"
#include <string>
#include <iostream>
#include "common/math/robotics.h"

struct CtrlComponents{
public:
    CtrlComponents();
    ~CtrlComponents();

    int dof;
    std::string armConfigPath;
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;
    ArmDynKineModel *armModel;
    CSVTool *stateCSV;
    
    //config
    Control ctrl;
    bool _hasGripper;
    std::string ctrl_IP;

    double dt;
    bool *running;

    void sendRecv();
    void geneObj();

    bool isRecord;
    void writeData();
private:
    Vec3 _endPosLocal;
    double _endEffectorMass;
    Vec3 _endEffectorCom;
    Mat3 _endEffectorInertia;
};

#endif  // CTRLCOMPONENTS_H