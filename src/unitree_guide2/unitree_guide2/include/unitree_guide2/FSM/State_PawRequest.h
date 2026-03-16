#ifndef STATE_PAWREQUEST_H
#define STATE_PAWREQUEST_H

#include "FSM/FSMState.h"
#include "VisionUDP.h"

struct VisionTarget{ 
    Vec3 pos_cam; 
    bool valid; 
};

class State_PawRequest : public FSMState {
public:
    State_PawRequest(CtrlComponents *ctrlComp);
    ~State_PawRequest() {}

    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    float _percent;
    int _phase;
    float _duration;
    float _holdTime;

    float _startPos[12];
    float _targetPos[12];

    bool  _hasTarget;
    float _visionTarget[3];

    VisionUDP _visionUDP;
};

#endif
