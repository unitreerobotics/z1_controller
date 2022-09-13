#ifndef ENUMCLASS_H
#define ENUMCLASS_H

enum class FrameType{
    BODY,
    HIP,
    ENDEFFECTOR,
    GLOBAL
};

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
    SETTRAJ,
    DANCE00,
    DANCE01,
    DANCE02,
    DANCE03,
    DANCE04,
    DANCE05,
    DANCE06,
    DANCE07,
    DANCE08,
    DANCE09,
    BACKTOSTART,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
    NEXT,
    LOWCMD
};

enum class JointMotorType{
    SINGLE_MOTOR,
    DOUBLE_MOTOR
};

enum class MotorMountType{
    STATOR_FIRST,
    OUTPUT_FIRST
};

enum class HeadType{
    TIGER,
    DOG
};

enum class Control{
    _KEYBOARD,
    _SDK,
    _JOYSTICK,
};

#endif  // ENUMCLASS_H