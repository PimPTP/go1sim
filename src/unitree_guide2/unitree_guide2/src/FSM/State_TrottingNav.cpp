#include "FSM/State_TrottingNav.h"

State_TrottingNav::State_TrottingNav(CtrlComponents *ctrlComp)
: FSMState(ctrlComp, FSMStateName::TROTTINGNAV, "trotting nav"),
  _est(ctrlComp->estimator),
  _phase(ctrlComp->phase),
  _contact(ctrlComp->contact),
  _robModel(ctrlComp->robotModel),
  _balCtrl(ctrlComp->balCtrl)
{
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;

    _Kpp = Vec3(70,70,70).asDiagonal();
    _Kdp = Vec3(10,10,10).asDiagonal();
    _kpw = 780;
    _Kdw = Vec3(70,70,70).asDiagonal();
    _KpSwing = Vec3(400,400,400).asDiagonal();
    _KdSwing = Vec3(10,10,10).asDiagonal();

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

    _node = rclcpp::Node::make_shared("trotting_nav_node");

    _sub_cmd = _node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&State_TrottingNav::cmdCallback, this, std::placeholders::_1)
    );

    _executor.add_node(_node); 
}

State_TrottingNav::~State_TrottingNav(){
    delete _gait;
}

void State_TrottingNav::enter(){
    _pcd = _est->getPosition();
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0);

    _vCmdBody.setZero();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    _wCmdGlobal.setZero();

    _gait->restart();
}

void State_TrottingNav::exit(){
    _ctrlComp->setAllSwing();
}

FSMStateName State_TrottingNav::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::START){
        return FSMStateName::TROTTING;
    }
    return FSMStateName::TROTTINGNAV;
}

void State_TrottingNav::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    _cmd_vel = *msg;
}

void State_TrottingNav::getNavCmd(){
    _vCmdBody(0) = _cmd_vel.linear.x;
    _vCmdBody(1) = _cmd_vel.linear.y;
    _vCmdBody(2) = 0;
    _dYawCmd     = _cmd_vel.angular.z;
}

void State_TrottingNav::run(){

    _executor.spin_some();

    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();

    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    getNavCmd();

    if( fabs(_vCmdBody(0)) > 0.03 ||
        fabs(_vCmdBody(1)) > 0.03 ||
        fabs(_dYawCmd) > 0.20 )
    {
        _ctrlComp->setStartWave();
    }
    else
    {
        _ctrlComp->setAllStance();
    }

    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _pcd(0) += _vCmdGlobal(0) * _ctrlComp->dt;
    _pcd(1) += _vCmdGlobal(1) * _ctrlComp->dt;

    _yawCmd += _dYawCmd * _ctrlComp->dt;
    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat)
           + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for(int i=0;i<4;i++){
        if((*_contact)(i)==0){
            _forceFeetGlobal.col(i) =
                _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) +
                _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;

    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);

    _lowCmd->setTau(_tau);
}