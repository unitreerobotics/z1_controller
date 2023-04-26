#ifndef STATE_CALIBRATION_H
#define STATE_CALIBRATION_H

#include "FSM/FSMState.h"

class State_Calibration : public FSMState{
public:
    State_Calibration(CtrlComponents *ctrlComp);
    ~State_Calibration(){}
    void enter();
    void run(){};
    void exit(){};
    int checkChange(int cmd);
private:

};

#endif  // STATE_CALIBRATION_H