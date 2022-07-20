#ifndef SAVESTATE_H
#define SAVESTATE_H

#include "FSMState.h"
#include "common/utilities/CSVTool.h"

class State_SaveState : public FSMState{
public:
    State_SaveState(CtrlComponents *ctrlComp);
    ~State_SaveState();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    CSVTool *_csv;
    Vec6 _q;
    HomoMat _endHomo;
};

#endif  // SAVESTATE_H