/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    GAZEBO,
    REALROBOT,
};

enum class RobotType{
    A1,
    Go1
};

enum class UserCommand{
    // EXIT,
    NONE,
    START,      // trotting
    L2_A,       // fixedStand
    L2_B,       // passive
    L2_X,       // freeStand
#ifdef COMPILE_WITH_MOVE_BASE
    L2_Y,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    L1_X,
    L1_A,       // 9 pawrequest
    L1_B,       // 8 trottingNav
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,
    PAWREQUEST,
    TROTTINGNAV,
#ifdef COMPILE_WITH_MOVE_BASE
    MOVE_BASE,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    BALANCETEST,
    SWINGTEST,
    STEPTEST
};

#endif  // ENUMCLASS_H