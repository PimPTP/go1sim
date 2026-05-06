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
    int   _phase;
    float _duration;
    float _holdTime;
    float _waitTime;

    float _startPos[12];
    float _targetPos[12];

    VisionUDP _visionUDP;
    bool _target_init;
    bool _hasTarget;

    bool getVisionTarget(Vec3& p_leg_out);

    Vec3 _p_target_filt;

    bool  _use_path;   
    bool  _path_init;
    float _path_t;

    Vec3   _last_target;
    double _target_thresh;

    Vec3 _p_start;
    Vec3 _p_mid;
    Vec3 _p_target;
};

#endif