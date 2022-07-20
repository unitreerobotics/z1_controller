#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H


#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/IOROS.h"
#include "model/ArmDynKineModel.h"
#include "model/ArmReal.h"
#include "common/utilities/CSVTool.h"
#include <string>
#include <iostream>

#ifdef COMPILE_DEBUG
#include "common/utilities/PyPlot.h"
#endif  // COMPILE_DEBUG

struct CtrlComponents{
public:
    CtrlComponents(){}
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
        delete armModel;
        delete stateCSV;



#ifdef COMPILE_DEBUG
        delete plot;
#endif  // COMPILE_DEBUG
    }

    int dof;
    std::string armConfigPath;
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;
    ArmDynKineModel *armModel;
    CSVTool *stateCSV;

#ifdef COMPILE_DEBUG
    PyPlot *plot;
#endif  // COMPILE_DEBUG

    double dt;
    bool *running;

    void sendRecv(){
        ioInter->sendRecv(lowCmd, lowState);
    }

    void geneObj(){
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState(dt);

#ifdef NO_GRIPPER    
    // armModel = new Z1DynKineModel();
    armModel = new Z1DynKineModel(Vec3(0.0, 0.0, 0.0),
        0.0, Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0).asDiagonal());
#endif
#ifdef AG95
    armModel = new Z1DynKineModel(Vec3(0.21, 0, 0),
        1.107, Vec3(0.092, 0, 0), Vec3(0.005, 0.005, 0.005).asDiagonal());
#endif
#ifdef MASS3KG
    armModel = new Z1DynKineModel(Vec3(0.05, 0, 0),
        3.1, Vec3(0.05, 0, 0), Vec3(0.005, 0.005, 0.005).asDiagonal());
#endif
#ifdef UNITREE_GRIPPER
    // 安装面高出6mm
    // armModel = new Z1DynKineModel(Vec3(0.141, 0, -0.005),
    //     0.712, Vec3(0.0488, 0.0, 0.002), Vec3(0.000688, 0.000993, 0.000812).asDiagonal());
    armModel = new Z1DynKineModel(Vec3(0.0, 0.0, 0.0),
        0.80225, Vec3(0.0037, 0.0014, -0.0003), Vec3(0.00057593, 0.00099960, 0.00106337).asDiagonal());
#endif
    }

private:
};

#endif  // CTRLCOMPONENTS_H