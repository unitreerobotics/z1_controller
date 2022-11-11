#include <csignal>
#include <sched.h>
#include "FSM/FiniteStateMachine.h"
#include "FSM/State_Passive.h"
#include "FSM/State_BackToStart.h"
#include "FSM/State_Calibration.h"
#include "FSM/State_Cartesian.h"
#include "FSM/State_JointSpace.h"
#include "FSM/State_MoveJ.h"
#include "FSM/State_MoveL.h"
#include "FSM/State_MoveC.h"
#include "FSM/State_ToState.h"
#include "FSM/State_SaveState.h"
#include "FSM/State_Teach.h"
#include "FSM/State_TeachRepeat.h"
#include "FSM/State_Trajectory.h"
#include "FSM/State_LowCmd.h"
#include "FSM/State_SetTraj.h"

const std::string Z1_CTRL_VERSION = "2022.11.11";
bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig){
    running = false;
	std::cout << "[STATE] stop the controller" << std::endl;
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
    if(argc == 2) {
        if((strcmp(argv[1], "-v") == 0) || (strcmp(argv[1], "--version") == 0)){
            std::cout << "Version z1_controller: "<< Z1_CTRL_VERSION<<std::endl;
            return 0;
        }
    }
    
    /* set real-time process */
    setProcessScheduler();
    /* set the print format */
    std::cout << std::fixed << std::setprecision(3);


    EmptyAction emptyAction((int)ArmFSMStateName::INVALID);
    std::vector<KeyAction*> events;
    CtrlComponents *ctrlComp = new CtrlComponents();

    // control method
    if(argc == 2) {
        if(argv[1][0] == 'k'){
            ctrlComp->ctrl = Control::KEYBOARD;
        }else if(argv[1][0] == 's'){
            ctrlComp->ctrl = Control::SDK;
        }else if(argv[1][0] == 'j'){
            ctrlComp->ctrl = Control::JOYSTICK;
        }
    }
    
#ifdef COMPILE_WITH_SIMULATION
    ros::init(argc, argv, "z1_controller");
#endif

    // ctrlComp->isPlot = true;
    ctrlComp->dt = 1.0/250.;
    ctrlComp->armConfigPath =  "../config/";
    ctrlComp->stateCSV = new CSVTool("../config/savedArmStates.csv");
    ctrlComp->geneObj();
    if(ctrlComp->ctrl == Control::SDK){
        ctrlComp->cmdPanel = new ARMSDK(events, emptyAction, "127.0.0.1", 8072, 0.002);
    }else if(ctrlComp->ctrl == Control::KEYBOARD){
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

        events.push_back(new ValueAction("q", "a", 0.5));
        events.push_back(new ValueAction("w", "s", 0.5));
        events.push_back(new ValueAction("e", "d", 0.5));
        events.push_back(new ValueAction("r", "f", 0.5));
        events.push_back(new ValueAction("t", "g", 0.5));
        events.push_back(new ValueAction("y", "h", 0.5));
        events.push_back(new ValueAction("down", "up", 1.));

        ctrlComp->cmdPanel = new Keyboard(events, emptyAction);
    }else if(ctrlComp->ctrl == Control::JOYSTICK){
        events.push_back(new StateAction("r2x",     (int)ArmFSMStateName::TRAJECTORY));
        events.push_back(new StateAction("l12",     (int)ArmFSMStateName::PASSIVE));
        events.push_back(new StateAction("r2",      (int)ArmFSMStateName::JOINTCTRL));
        events.push_back(new StateAction("r1",      (int)ArmFSMStateName::CARTESIAN));
        events.push_back(new StateAction("select",  (int)ArmFSMStateName::BACKTOSTART));

        events.push_back(new ValueAction("left_up",     "left_down",    0.5));//Tran_Y
        events.push_back(new ValueAction("left_right",   "left_left",   0.5));//Tran_X, inverse
        events.push_back(new ValueAction("up",          "down",         -0.5));//Tran_Z, inverse
        events.push_back(new ValueAction("right_up",    "right_down",   0.5));//Rot_Y
        events.push_back(new ValueAction("right_left",  "right_right",  0.5));//Rot_x
        events.push_back(new ValueAction("Y",           "A",            0.5));//Rot_Z
        events.push_back(new ValueAction("right",       "left",         1.0));//girpper, close-open

        ctrlComp->cmdPanel = new UnitreeJoystick(events, emptyAction);
    }
    std::vector<FSMState*> states;
    states.push_back(new State_Passive(ctrlComp));
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

    FiniteStateMachine *fsm;
    fsm = new FiniteStateMachine(states, ctrlComp);

    ctrlComp->running = &running;
    signal(SIGINT, ShutDown);
    while(running){
        usleep(100000);
    }

    delete fsm;
    delete ctrlComp;
    return 0;
}