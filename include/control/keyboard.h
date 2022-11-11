#ifndef _UNITREE_ARM_KEYBOARD_H_
#define _UNITREE_ARM_KEYBOARD_H_

#include <string>
#include <vector>
#include <deque>

#include "message/udp.h"
#include "common/enumClass.h"
#include "control/cmdPanel.h"

class Keyboard : public CmdPanel{
public:
    Keyboard(std::vector<KeyAction*> events, 
        EmptyAction emptyAction, size_t channelNum = 1, double dt = 0.002);
    ~Keyboard();
    std::string getString(std::string slogan);
    std::vector<double> stringToArray(std::string slogan);
    std::vector<std::vector<double> > stringToMatrix(std::string slogan);
private:
    void _read();
    void _pauseKey();
    void _startKey();
    void _extractCmd();

    fd_set _set;
    char _c = '\0';

    termios _oldSettings;
    termios _newSettings;
    timeval _tv;
};

#endif // _UNITREE_ARM_KEYBOARD_H_