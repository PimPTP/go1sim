#include <iostream>
#include <cmath>
#include "FSM/State_ShakeHand.h"
#include "FSM/Process.h"

State_ShakeHand::State_ShakeHand(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::SHAKEHAND, "shake hand"),
    _visionUDP(9000)
{
    _percent   = 0.0f;
    _phase = 0;
    _duration  = 2.0f;
    _holdTime = 0.0f;
    _hasTarget = false;
}

void State_ShakeHand::enter(){

    std::cout << "[FSM] Enter SHAKEHAND" << std::endl;

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

    _targetPos[0] = 0.0;   // FL hip
    _targetPos[1] = 0.6;   // FL thigh
    _targetPos[2] = -1.2;  // FL calf

    _targetPos[3] = 0.0;   // FR 
    _targetPos[4] = 0.6;
    _targetPos[5] = -1.2;

    _targetPos[6]  = 0.0;   // RL 
    _targetPos[7]  = 1.4;   
    _targetPos[8]  = -2.4; 

    _targetPos[9]  = 0.0;   // RR 
    _targetPos[10] = 1.4;
    _targetPos[11] = -2.4;

    _percent = 0.0f;
    _phase = 0;

    _ctrlComp->setAllStance();
    *_ctrlComp->contact = VecInt4(1,1,1,1);
}

void State_ShakeHand::run(){

    switch(_phase){
    case 0:
    {
        _percent += _ctrlComp->dt / _duration;
        if(_percent > 1.0f) _percent = 1.0f;

        for(int j=0; j<12; j++){
            _lowCmd->motorCmd[j].q =
                (1.0f - _percent)*_startPos[j] + _percent*_targetPos[j];
        }

        bool isSitReached = true;
        for(int j=0; j<12; j++){
            if(std::fabs(_lowState->motorState[j].q - _targetPos[j]) > 0.08){
                isSitReached = false;
                break;
            }
        }

        if(_percent >= 0.999f && isSitReached){
            std::cout << "[SHAKEHAND] Sit." << std::endl;
            _percent = 0.0f;
            _phase   = 1;
        }
        break;
    }

    case 1:
    {
        float x,y,z;

        if(_visionUDP.recv(x,y,z)){
            _visionTarget[0] = x;
            _visionTarget[1] = y;
            _visionTarget[2] = z;
            _hasTarget = true;

            std::cout << "[UDP] "
                      << x << " "
                      << y << " "
                      << z << std::endl;
        }

        if(!_hasTarget)
            break;
        
        Vec3 p_cam  {_visionTarget[0], _visionTarget[1], _visionTarget[2]};
        Vec3 p_base = camToBase(p_cam);
        Vec3 p_leg  = baseToLegFR(p_base);

        if(!inWorkspace(p_leg.x(), p_leg.y(), p_leg.z())){
            std::cout << "[SHAKEHAND] Out of workspace";
            break;
        }

        double qh, qt, qc;
        if(!solveIK_FR(p_leg.x(), p_leg.y(), p_leg.z(),
           qh, qt, qc)) break;
        if(!checkJointLimit(qh, qt, qc)) break;

        for(int i=0;i<12;i++)
            _startPos[i] = _lowCmd->motorCmd[i].q;

        _targetPos[3] = qh;
        _targetPos[4] = qt;
        _targetPos[5] = qc;

        _percent = 0.0f;
        _phase   = 2;
        break;
    }

    case 2:
    {
        *_ctrlComp->contact = VecInt4(1, 0, 1, 1);
        _ctrlComp->setAllStance();

        _percent += _ctrlComp->dt / 1.5f;
        if(_percent > 1.0f) _percent = 1.0f;

        for(int j=0; j<12; j++){
            _lowCmd->motorCmd[j].q = _startPos[j];
        }

        for(int j=3; j<=5; j++){
            _lowCmd->motorCmd[j].q =
                (1.0f - _percent)*_startPos[j] + _percent*_targetPos[j];
        }

        bool isLegReached = true;
        for(int j=3; j<=5; j++){
            if(std::fabs(_lowState->motorState[j].q - _targetPos[j]) > 0.08){
                isLegReached = false;
                break;
            }
        }
        
        if(_percent >= 0.999f && isLegReached){
            std::cout << "[SHAKEHAND] Raise." << std::endl;
            _percent = 0.0f;
            _phase = 3;
        }
        break;
    }

    case 3:
    {
        _holdTime += _ctrlComp->dt;
        if(_holdTime < 5.0f)
            break;

        std::cout << "[SHAKEHAND] Lower." << std::endl;
        for(int i=0;i<12;i++)
            _startPos[i] = _lowCmd->motorCmd[i].q;

        _targetPos[3] = 0.0;
        _targetPos[4] = 0.6;
        _targetPos[5] = -1.2;

        _percent = 0.0f;
        _phase = 4;
        break;
    }

    case 4:
    {
        _percent += _ctrlComp->dt / 1.5f;
        if(_percent > 1.0f) _percent = 1.0f;

        for(int j=3; j<=5; j++){
            _lowCmd->motorCmd[j].q =
                (1.0f - _percent)*_startPos[j] + _percent*_targetPos[j];
        }

        bool isLegReached = true;
        for(int j=3; j<=5; j++){
            if(std::fabs(_lowState->motorState[j].q - _targetPos[j]) > 0.08){
                isLegReached = false;
                break;
            }
        }
        break;
    }

    default:
        _phase = 0;
        break;
    }
}

void State_ShakeHand::exit(){
    std::cout << "[FSM] Exit SHAKEHAND" << std::endl;
    _percent = 0;
    _phase   = 0;
}

FSMStateName State_ShakeHand::checkChange(){

    if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }

    return FSMStateName::SHAKEHAND;
}
