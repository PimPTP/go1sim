#ifndef TROTTINGNAV_H
#define TROTTINGNAV_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class State_TrottingNav : public FSMState{
public:
    State_TrottingNav(CtrlComponents *ctrlComp);
    ~State_TrottingNav();

    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();

private:
    void getNavCmd();
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // ROS2
    rclcpp::Node::SharedPtr _node;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _sub_cmd;
    rclcpp::executors::SingleThreadedExecutor _executor;  

    geometry_msgs::msg::Twist _cmd_vel;

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    Vec3  _posBody, _velBody;
    double _yaw, _dYaw;
    Vec34 _posFeetGlobal, _velFeetGlobal;
    Vec34 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec12 _q;

    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    double _yawCmd, _dYawCmd;
    Vec3 _wCmdGlobal;
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34 _posFeet2BGoal, _velFeet2BGoal;
    RotMat _Rd;
    Vec3 _ddPcd, _dWbd;
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _qGoal, _qdGoal;
    Vec12 _tau;

    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim;
    Vec4 *_phase;
    VecInt4 *_contact;
};

#endif