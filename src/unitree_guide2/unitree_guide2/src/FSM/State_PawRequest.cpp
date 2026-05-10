#include <iostream>
#include <cmath>
#include "FSM/State_PawRequest.h"
#include "FSM/Process.h"

bool State_PawRequest::getVisionTarget(Vec3& p_leg_out)
{
    float x, y, z;

    if(!_visionUDP.recv(x, y, z))
        return false;

    std::cout << "[UDP] "
              << x << " "
              << y << " "
              << z << std::endl;

    Vec3 p_cam{x, y, z};
    Vec3 p_base = camToBase(p_cam);
    Vec3 p_leg  = baseToLegFR(p_base);

    std::cout << "[TARGET CHECK]\n";
    std::cout << "Cam  : "
              << p_cam.x() << " "
              << p_cam.y() << " "
              << p_cam.z() << std::endl;

    std::cout << "Base : "
              << p_base.x() << " "
              << p_base.y() << " "
              << p_base.z() << std::endl;

    std::cout << "Leg  : "
              << p_leg.x() << " "
              << p_leg.y() << " "
              << p_leg.z() << std::endl;

    if(!inWorkspace(p_leg.x(), p_leg.y(), p_leg.z())){
        std::cout << "Out of workspace\n";
        return false;
    }

    p_leg_out = p_leg;
    return true;
}

State_PawRequest::State_PawRequest(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::PAWREQUEST, "paw request"),
      _visionUDP(9000)
{
    _percent   = 0.0f;
    _phase     = 0;
    _holdTime  = 0.0f;
    _waitTime  = 0.0f;

    _hasTarget   = false;
    _target_init = false;

    _use_path = true;
    _path_init = false;
    _path_t = 0.0f;

    _target_thresh = 0.02;
}

void State_PawRequest::enter()
{
    std::cout << "[FSM] Enter PAWREQUEST" << std::endl;

    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
        _lowCmd->setZeroDq(i);
        _lowCmd->setZeroTau(i);
    }

    for(int i=0; i<12; i++){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _startPos[i] = _lowState->motorState[i].q;
        _targetPos[i] = _startPos[i];
    }

    // sit pose
    _targetPos[0] = 0.0;  _targetPos[1] = 0.6;  _targetPos[2] = -1.2;
    _targetPos[3] = 0.0;  _targetPos[4] = 0.6;  _targetPos[5] = -1.2;
    _targetPos[6] = 0.0;  _targetPos[7] = 1.4;  _targetPos[8] = -2.4;
    _targetPos[9] = 0.0;  _targetPos[10] = 1.4; _targetPos[11] = -2.4;

    _percent = 0.0f;
    _phase   = 0;

    _ctrlComp->setAllStance();
    *_ctrlComp->contact = VecInt4(1,1,1,1);

    _target_init = false;
    _path_init = false;
}

void State_PawRequest::run()
{
    switch(_phase){
    // SIT 
    case 0:
    {
        _percent += _ctrlComp->dt / 2.0f;
        if(_percent > 1.0f) _percent = 1.0f;

        for(int j = 0; j < 12; j++){
            _lowCmd->motorCmd[j].q =
                (1.0f - _percent) * _startPos[j] +
                _percent * _targetPos[j];
        }

        bool ok = true;
        for(int j = 0; j < 12; j++){
            if(std::fabs(_lowState->motorState[j].q - _targetPos[j]) > 0.08){
                ok = false;
                break;
            }
        }

        if(_percent >= 0.999f && ok){
            std::cout << "[PAWREQUEST] Sit." << std::endl;
            _percent = 0.0f;
            _phase = 1;
        }
        break;
    }

    // WAIT TARGET 
    case 1:
    {
        Vec3 p_leg;

        if(getVisionTarget(p_leg)){
            _p_target_filt = p_leg;
            _last_target = p_leg;

            _target_init = true;
            _path_init = false;

            _phase = 2;
            std::cout << "[PAWREQUEST] Raise." << std::endl;
        }
        break;
    }

    // MOVE
    case 2:
    {
        *_ctrlComp->contact = VecInt4(1,0,1,1);

        if(!_target_init)
            break;

        double qh = _lowState->motorState[0].q;
        double qt = _lowState->motorState[1].q;
        double qc = _lowState->motorState[2].q;

        Vec3 p_now = solveFK_FR(qh, qt, qc);
        Vec3 p_des;

        // PATH 
        if(_use_path){

            if(!_path_init){
                _p_start  = p_now;
                _p_target = _p_target_filt;

                _p_mid = (_p_start + _p_target) * 0.5;
                _p_mid.z() += 0.1;

                _path_t = 0.0f;
                _path_init = true;

                std::cout << "[PATH] Start\n";
            }

            _path_t += _ctrlComp->dt / 2.0f;
            if(_path_t > 1.0f)
                _path_t = 1.0f;

            double t_s = smooth(_path_t);
            p_des = path(_p_start, _p_mid, _p_target, t_s);

            if(_path_t >= 1.0f){
                std::cout << "[PATH] Done\n";
                _holdTime = 0.0f;
                _phase = 3;
                break;
            }
        }

        // TRACK
        else{
            Vec3 p_leg;

            if(getVisionTarget(p_leg)){
                _p_target_filt = p_leg;
            }

            double gain = 0.2;
            p_des = p_now + gain * (_p_target_filt - p_now);
            p_des.z() += 0.03;

            double dist = (_p_target_filt - p_now).norm();

            if(dist < 0.04){
                std::cout << "[PAWREQUEST] Reach.\n";
                _holdTime = 0.0f;
                _phase = 3;
            }
        }

        double qh2, qt2, qc2;
        if(!solveIK_FR(p_des.x(), p_des.y(), p_des.z(), qh2, qt2, qc2))
            break;

        if(!checkJointLimit(qh2, qt2, qc2))
            break;

        _lowCmd->motorCmd[0].q = qh2;
        _lowCmd->motorCmd[1].q = qt2;
        _lowCmd->motorCmd[2].q = qc2;

        break;
    }

    // HOLD 
    case 3:
    {
        _holdTime += _ctrlComp->dt;

        if(_holdTime > 5.0f){
            std::cout << "[PAWREQUEST] Lower.\n";

            for(int i = 0; i < 12; i++)
                _startPos[i] = _lowCmd->motorCmd[i].q;

            _targetPos[0] = 0.0;
            _targetPos[1] = 0.6;
            _targetPos[2] = -1.2;

            _percent = 0.0f;
            _phase = 4;
        }
        break;
    }

    // LOWER 
    case 4:
    {
        _percent += _ctrlComp->dt / 2.0f;
        if(_percent > 1.0f)
            _percent = 1.0f;

        for(int j = 0; j <= 2; j++){
            _lowCmd->motorCmd[j].q =
                (1.0f - _percent) * _startPos[j] +
                _percent * _targetPos[j];
        }

        bool ok = true;
        for(int j = 0; j <= 2; j++){
            if(std::fabs(_lowState->motorState[j].q - _targetPos[j]) > 0.08){
                ok = false;
                break;
            }
        }

        if(_percent >= 0.999f && ok){
            std::cout << "[PAWREQUEST] Down.\n";
            _waitTime = 0.0f;
            _phase = 5;
        }
        break;
    }

    // WAIT 
    case 5:
    {
        _waitTime += _ctrlComp->dt;

        if(_waitTime > 10.0f){
            std::cout << "[PAWREQUEST] Ready.\n";
            _target_init = false;
            _phase = 1;
        }
        break;
    }

    default:
        _phase = 0;
        break;
    }
}

void State_PawRequest::exit(){
    std::cout << "[FSM] Exit PAWREQUEST" << std::endl;
    _percent = 0;
    _phase   = 0;
}

FSMStateName State_PawRequest::checkChange(){

    if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }

    return FSMStateName::PAWREQUEST;
}