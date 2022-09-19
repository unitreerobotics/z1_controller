#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>
#include <iomanip>
#include "control/CtrlComponents.h"
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
#include "FSM/State_SetTraj.h"

#include "unitree_arm_sdk/unitree_arm_sdk.h"

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig){
	std::cout << "[STATE] stop the controller" << std::endl;
    running = false;
}

//set real-time program
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
    CtrlComponents *ctrlComp = new CtrlComponents();

    if(ctrlComp->ctrl == Control::_SDK)
    {
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
        events.push_back(new StateAction("l", (int)ArmFSMStateName::SETTRAJ));

        events.push_back(new ValueAction("q", "a", 1.0));
        events.push_back(new ValueAction("w", "s", 1.0));
        events.push_back(new ValueAction("e", "d", 1.0));
        events.push_back(new ValueAction("r", "f", 1.0));
        events.push_back(new ValueAction("t", "g", 1.0));
        events.push_back(new ValueAction("y", "h", 1.0));
        events.push_back(new ValueAction("down", "up", 1.0));
        
        cmdPanel = new UnitreeKeyboardUDPRecv(events, emptyAction);
    }else if(ctrlComp->ctrl == Control::_KEYBOARD){
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
    }else if(ctrlComp->ctrl == Control::_JOYSTICK){
        events.push_back(new StateAction("r2x", (int)ArmFSMStateName::TRAJECTORY));
        events.push_back(new StateAction("l12", (int)ArmFSMStateName::PASSIVE));
        events.push_back(new StateAction("r2", (int)ArmFSMStateName::JOINTCTRL));
        events.push_back(new StateAction("r1", (int)ArmFSMStateName::CARTESIAN));
        events.push_back(new StateAction("select", (int)ArmFSMStateName::BACKTOSTART));

        events.push_back(new ValueAction("left_up", "left_down", 1.0));//Tran_Y
        events.push_back(new ValueAction("left_left", "left_right", -1.0));//Tran_X, inverse
        events.push_back(new ValueAction("up", "down", 1.0));//Tran_Z
        events.push_back(new ValueAction("right_up", "right_down", 1.0));//Rot_Y
        events.push_back(new ValueAction("right_left", "right_right", 1.0));//Rot_x
        events.push_back(new ValueAction("Y", "A", 1.0));//Rot_Z
        events.push_back(new ValueAction("right", "left",  1.0));//girpper, close-open

        cmdPanel = new UnitreeJoystick(events, emptyAction);
    }

#ifdef RUN_ROS
    ros::init(argc, argv, "z1_controller");
#endif  // RUN_ROS

    ctrlComp->dof = 6;
    ctrlComp->armConfigPath =  "../config/";
    ctrlComp->stateCSV = new CSVTool("../config/savedArmStates.csv");
    ctrlComp->running = &running;
#ifdef UDP
    ctrlComp->ioInter = new IOUDP(cmdPanel, ctrlComp->ctrl_IP.c_str(), ctrlComp->_hasGripper);
#elif defined ROS
    ctrlComp->ioInter = new IOROS(cmdPanel, ctrlComp->_hasGripper);
#endif
    ctrlComp->dt = 0.004;
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
    states.push_back(new State_SetTraj(ctrlComp));

    FiniteStateMachine fsm(states, cmdPanel, 0, ctrlComp->dt);

    signal(SIGINT, ShutDown);

    while(running){
        usleep(100000);
    }

#ifdef COMPILE_DEBUG
    ctrlComp->writeData();
#endif

    delete ctrlComp;

    return 0;
}
