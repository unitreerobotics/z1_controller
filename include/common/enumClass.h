#ifndef ENUMCLASS_H
#define ENUMCLASS_H

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class ArmFSMStateName{
    INVALID,
    PASSIVE,
    JOINTCTRL,
    CARTESIAN,
    MOVEJ,
    MOVEL,
    MOVEC,
    TRAJECTORY,
    TOSTATE,
    SAVESTATE,
    TEACH,
    TEACHREPEAT,
    CALIBRATION,
    SETTRAJ,//no longer used
    BACKTOSTART,
    NEXT,
    LOWCMD
};

enum class JointMotorType{
    SINGLE_MOTOR,
    DOUBLE_MOTOR
};

enum class Control{
    KEYBOARD,
    SDK
};

#endif  // ENUMCLASS_H