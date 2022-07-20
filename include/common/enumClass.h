#ifndef ENUMCLASS_H
#define ENUMCLASS_H

// enum class UserCommand{
//     // EXIT,
//     NONE,
//     START,
//     L2_A,       //used by robot
//     L2_B,       //used by robot
//     L2_X,       //used by robot
//     R2_A,       //Arm: joint space control
//     R2_X,       //Arm: Cartesian
//     R2_Y,       //
//     R2_temp,
//     ARM_PASSIVE,       //Arm: passive
//     ARM_CHECKJOINT_0,
//     ARM_TRAJECTORY_0,
//     ARM_SAVESTATE_0,
//     ARM_TEACH_0,
//     ARM_TEACHREPEAT_0,
//     ARM_NEXT,
//     ARM_BACKTOSTART,

//     HEAD_JOINTCTRL,
//     HEAD_TOSTART,
//     HEAD_SAVE,
//     HEAD_TURN,
//     HEAD_CALI,
//     HEAD_NEXT,
//     HEAD_DANCE01,
//     HEAD_DANCE02,
//     HEAD_DANCE03,
//     HEAD_DANCE04,
//     HEAD_DANCE05,
//     HEAD_DANCE06,
//     HEAD_DANCE07
// };

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

#endif  // ENUMCLASS_H