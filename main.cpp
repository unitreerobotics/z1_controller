#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <iomanip>
#include "control/CtrlComponents.h"
#include "model/ArmReal.h"
#include "model/ArmDynKineModel.h"
#include "interface/IOUDP.h"

#include "FSM/State_BackToStart.h"
#include "FSM/State_Cartesian.h"
#include "FSM/State_MoveJ.h"
#include "FSM/State_MoveL.h"
#include "FSM/State_MoveC.h"
#include "FSM/State_Dance.h"
#include "FSM/State_JointSpace.h"
#include "FSM/State_Passive.h"
#include "FSM/State_SaveState.h"
#include "FSM/State_Teach.h"
#include "FSM/State_TeachRepeat.h"
#include "FSM/State_ToState.h"
#include "FSM/State_Trajectory.h"
#include "FSM/State_Calibration.h"
#include "FSM/State_LowCmd.h"
#include "FSM/FiniteStateMachine.h"

#include "unitree_arm_sdk/unitree_arm_sdk.h"

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig){
	std::cout << "[STATE] stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler(){
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv){
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);

    EmptyAction emptyAction((int)ArmFSMStateName::INVALID);
    std::vector<KeyAction*> events;
    CmdPanel *cmdPanel;
#ifdef CTRL_BY_SDK
    events.push_back(new StateAction("`", (int)ArmFSMStateName::BACKTOSTART));
    events.push_back(new StateAction("1", (int)ArmFSMStateName::PASSIVE));
    events.push_back(new StateAction("2", (int)ArmFSMStateName::JOINTCTRL));
    events.push_back(new StateAction("3", (int)ArmFSMStateName::CARTESIAN));
    events.push_back(new StateAction("4", (int)ArmFSMStateName::MOVEJ));
    events.push_back(new StateAction("5", (int)ArmFSMStateName::MOVEL));
    events.push_back(new StateAction("6", (int)ArmFSMStateName::MOVEC));
    events.push_back(new StateAction("7", (int)ArmFSMStateName::TEACH));
    events.push_back(new StateAction("8", (int)ArmFSMStateName::TEACHREPEAT));
    events.push_back(new StateAction("9", (int)ArmFSMStateName::SAVESTATE));
    events.push_back(new StateAction("0", (int)ArmFSMStateName::TOSTATE));
    events.push_back(new StateAction("-", (int)ArmFSMStateName::TRAJECTORY));
    events.push_back(new StateAction("=", (int)ArmFSMStateName::CALIBRATION));
    events.push_back(new StateAction("]", (int)ArmFSMStateName::NEXT));
    events.push_back(new StateAction("/", (int)ArmFSMStateName::LOWCMD));

    events.push_back(new ValueAction("q", "a", 1.0));
    events.push_back(new ValueAction("w", "s", 1.0));
    events.push_back(new ValueAction("e", "d", 1.0));
    events.push_back(new ValueAction("r", "f", 1.0));
    events.push_back(new ValueAction("t", "g", 1.0));
    events.push_back(new ValueAction("y", "h", 1.0));
    events.push_back(new ValueAction("down", "up", 1.0));
    
    cmdPanel = new UnitreeKeyboardUDPRecv(events, emptyAction);
#endif
#ifdef CTRL_BY_KEYBOARD
    events.push_back(new StateAction("`", (int)ArmFSMStateName::BACKTOSTART));
    events.push_back(new StateAction("1", (int)ArmFSMStateName::PASSIVE));
    events.push_back(new StateAction("2", (int)ArmFSMStateName::JOINTCTRL));
    events.push_back(new StateAction("3", (int)ArmFSMStateName::CARTESIAN));
    events.push_back(new StateAction("4", (int)ArmFSMStateName::MOVEJ));
    events.push_back(new StateAction("5", (int)ArmFSMStateName::MOVEL));
    events.push_back(new StateAction("6", (int)ArmFSMStateName::MOVEC));
    events.push_back(new StateAction("7", (int)ArmFSMStateName::TEACH));
    events.push_back(new StateAction("8", (int)ArmFSMStateName::TEACHREPEAT));
    events.push_back(new StateAction("9", (int)ArmFSMStateName::SAVESTATE));
    events.push_back(new StateAction("0", (int)ArmFSMStateName::TOSTATE));
    events.push_back(new StateAction("-", (int)ArmFSMStateName::TRAJECTORY));
    events.push_back(new StateAction("=", (int)ArmFSMStateName::CALIBRATION));
    events.push_back(new StateAction("]", (int)ArmFSMStateName::NEXT));

    events.push_back(new ValueAction("q", "a", 1.0));
    events.push_back(new ValueAction("w", "s", 1.0));
    events.push_back(new ValueAction("e", "d", 1.0));
    events.push_back(new ValueAction("r", "f", 1.0));
    events.push_back(new ValueAction("t", "g", 1.0));
    events.push_back(new ValueAction("y", "h", 1.0));
    events.push_back(new ValueAction("down", "up", 1.0));

    cmdPanel = new Keyboard(events, emptyAction);
#endif
#ifdef CTRL_BY_JOYSTICK
    events.push_back(new StateAction("r12_passive", (int)ArmFSMStateName::PASSIVE));
    events.push_back(new StateAction("r2_backToState", (int)ArmFSMStateName::BACKTOSTART));
    events.push_back(new StateAction("r1_cartesian", (int)ArmFSMStateName::CARTESIAN));
    events.push_back(new StateAction("select", (int)ArmFSMStateName::NEXT));

    events.push_back(new ValueAction("ly_forTranY", "ly_invTranY", 1.0));
    events.push_back(new ValueAction("lx_forTranX", "lx_invTranX", 1.0));
    events.push_back(new ValueAction("up_forTranZ", "down_invTranZ", 1.0));
    events.push_back(new ValueAction("ry_forRotY", "ry_invRotY", 1.0));
    events.push_back(new ValueAction("rx_forRotX", "rx_invRotX", 1.0));
    events.push_back(new ValueAction("Y_forRotZ", "A_invRotZ", 1.0));
    events.push_back(new ValueAction("right_gClose", "left_gOpen",  1.0));
    // events.push_back(new ValueAction("X_velDown", "B_velUp", 1.0));

    cmdPanel = new UnitreeJoystick(events, emptyAction);
#endif

#ifdef RUN_ROS
    ros::init(argc, argv, "z1_controller");
#endif  // RUN_ROS

    CtrlComponents *ctrlComp = new CtrlComponents();
    ctrlComp->dof = 6;
    ctrlComp->armConfigPath =  "../config/";
    ctrlComp->stateCSV = new CSVTool("../config/savedArmStates.csv");
    ctrlComp->running = &running;

#ifdef UDP
    ctrlComp->ioInter = new IOUDP(cmdPanel);
    ctrlComp->dt = 0.004;
#endif
#ifdef ROS
    ctrlComp->ioInter = new IOROS(cmdPanel);
    ctrlComp->dt = 0.004;
#endif

#ifdef COMPILE_DEBUG
std::cout << "add plot" << std::endl;

    ctrlComp->plot = new PyPlot();
    /* for general debugging */
    // ctrlComp->plot->addPlot("qCmd", 6);
    // ctrlComp->plot->addPlot("qdCmd", 6);
    // ctrlComp->plot->addPlot("tauCmd", 6);

    // ctrlComp->plot->addPlot("qReal", 6);
    // ctrlComp->plot->addPlot("qdReal", 6);
    // ctrlComp->plot->addPlot("realTau", 6);

    // ctrlComp->plot->addPlot("errorTau", 6);

    // ctrlComp->plot->addPlot("error", 1);
    // ctrlComp->plot->addPlot("costTime", 1);

    /* for MoveJ, MoveL, MoveC*/
    // ctrlComp->plot->addPlot("moveLPostureDes", 6);
    // ctrlComp->plot->addPlot("moveLTwistDes", 6);
    // ctrlComp->plot->addPlot("moveLPostureAct", 6);
    // ctrlComp->plot->addPlot("moveLTwistAct", 6);
    // ctrlComp->plot->addPlot("moveLPostureError", 6);
    // ctrlComp->plot->addPlot("moveLTwistError", 6);

    // /* for cartesian */
    // ctrlComp->plot->addPlot("cartesianPostureDes", 6);
    // ctrlComp->plot->addPlot("cartesianTwistDes", 6);
    // ctrlComp->plot->addPlot("cartesianPostureAct", 6);
    // ctrlComp->plot->addPlot("cartesianTwistAct", 6);
    // ctrlComp->plot->addPlot("cartesianPostureError", 6);
    // ctrlComp->plot->addPlot("cartesianTwistError", 6);

    /* for trajectory */
    // plot->addPlot("s", 1);
    // plot->addPlot("qPath", 6);
    // plot->addPlot("qdPath", 6);
    // plot->addPlot("qdPathDiff", 6);
    // ctrlComp->plot->addPlot("qState", 6);
    // ctrlComp->plot->addPlot("qError", 6);

    /* for gripper */
    // ctrlComp->plot->addPlot("gripper q error", 1);
    // ctrlComp->plot->addPlot("gripper qd error", 1);
    // ctrlComp->plot->addPlot("gripper tau", 1);

#endif  // COMPILE_DEBUG
    ctrlComp->geneObj();

    std::vector<BaseState*> states;
    states.push_back(new State_Passive(ctrlComp));      // First state in states is the begining state for FSM
    states.push_back(new State_BackToStart(ctrlComp));
    states.push_back(new State_JointSpace(ctrlComp));
    states.push_back(new State_Cartesian(ctrlComp));
    states.push_back(new State_MoveJ(ctrlComp));
    states.push_back(new State_MoveL(ctrlComp));
    states.push_back(new State_MoveC(ctrlComp));
    states.push_back(new State_LowCmd(ctrlComp));
    states.push_back(new State_SaveState(ctrlComp));
    states.push_back(new State_Teach(ctrlComp));
    states.push_back(new State_TeachRepeat(ctrlComp));
    states.push_back(new State_ToState(ctrlComp));
    states.push_back(new State_Trajectory(ctrlComp));
    states.push_back(new State_Calibration(ctrlComp));

    std::vector<TrajectoryManager*> _trajMag;
    std::vector<JointSpaceTraj*> _jointTraj;

    /* start to forward */
    _trajMag.push_back(new TrajectoryManager(ctrlComp));
    _jointTraj.push_back(new JointSpaceTraj(ctrlComp));
    _jointTraj.at(_jointTraj.size()-1)->setJointTraj("startFlat", "forward", 0.5);
    _trajMag.at(_trajMag.size()-1)->addTrajectory(_jointTraj.at(_jointTraj.size()-1));

    /* forward to start */
    _trajMag.push_back(new TrajectoryManager(ctrlComp));
    _jointTraj.push_back(new JointSpaceTraj(ctrlComp));
    _jointTraj.at(_jointTraj.size()-1)->setJointTraj("forward", "startFlat", 0.5);
    _trajMag.at(_trajMag.size()-1)->addTrajectory(_jointTraj.at(_jointTraj.size()-1));

    states.push_back(new State_Dance(ctrlComp, _trajMag.at(0), ArmFSMStateName::DANCE00, ArmFSMStateName::DANCE01, "dance 00"));
    states.push_back(new State_Dance(ctrlComp, _trajMag.at(1), ArmFSMStateName::DANCE01, ArmFSMStateName::TRAJECTORY, "dance 01"));
// #endif

    FiniteStateMachine fsm(states, cmdPanel, 0, ctrlComp->dt);

    signal(SIGINT, ShutDown);

    // std::vector<double> values;
    while(running){
        // values = cmdPanel->getValues();
        // std::cout << "value: " << values.at(0) << std::endl;
        usleep(100000);
    }

#ifdef COMPILE_DEBUG
std::cout << "show plot" << std::endl;
    ctrlComp->plot->showPlotAll();
#endif  // COMPILE_DEBUG
    delete ctrlComp;

    return 0;
}
