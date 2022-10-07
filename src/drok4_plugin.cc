/*
 * RoK-3 Gazebo Simulation Code 
 * 
 * Robotics & Control Lab.
 * 
 * Master : BKCho
 * First developer : Yunho Han
 * Second developer : Minho Park
 * 
 * ======
 * Update date : 2022.03.16 by Yunho Han
 * ======
 */
//* Header file for C++
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <iostream>
#include <boost/bind.hpp>

//축 방향 (인수) 정의
//x축은 0, y축은 1, z축은 2이다.
#define X_ 0
#define Y_ 1
#define Z_ 2

//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <drok4_arm_pkgs/Gripper.h>
#include <functional>
#include <ignition/math/Vector3.hh>

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h> // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h> // urdf model read using RBDL
#include <Eigen/Dense> // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
//#include <Eigen/Geometry>

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

//Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

//RBDL//
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std;

typedef struct _AngleAxis {
    Vector3d n;
    double th;
} AngleAxis_;
AngleAxis_ a_axis;

typedef struct _Joystick {
    double x;
    double y;
    double z;
} Joystick_;
Joystick_ joy;

typedef struct _Orientation {
    double roll;
    double pitch;
    double yaw;
} Orientation_;
Orientation_ ori;

//Global variables declaration//
//Time variables, 1 - cos 제어를 위해 선언
double T = 4;
double t = 1.0;
double dt_ms = 1.0;

//Phase step, 동작 자동 제어를 위해 선언
int phase = 0;

//IK 연산 시간을 확인하기 위한 구조체 변수 선언
static struct timespec ik_start_time;
static struct timespec ik_end_time;
static struct timespec start_time;
static struct timespec end_time;

Vector3d start_posi;
Vector3d goal_posi;
Vector3d present_posi;
//Vector3d term_posi;
Vector3d command_posi;
//Vector3d prev_posi;

MatrixXd start_rot(3, 3);
MatrixXd goal_rot(3, 3);
MatrixXd present_rot(3, 3);
MatrixXd command_rot(3, 3);

//FK
VectorXd q_init(6);
VectorXd q_goal(6);
VectorXd q_present(6);
VectorXd q_command(6);
//VectorXd q_prev(6);

//IK
VectorXd q0(6);             //Initial guess for IK
MatrixXd C_err(3, 3);

int subTime = 0, updateTime = 0;

namespace gazebo
{

    class drok4_plugin : public ModelPlugin
    {
        //*** Variables for RoK-3 Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        int prev_press[5] = { 0 };
        int press[5] = { 0 };
        bool pressed[5] = { false };
        int joint_id = 0;
        //double P_coeff = 1, D_coeff = 1;

        int ctrl_mode = 0;
        bool goal_cal_once = true;
        //bool arrived = true;

        //* Model & Link & Joint Typedefs
        physics::ModelPtr model;

        physics::JointPtr Joint_1;
        physics::JointPtr Joint_2;
        physics::JointPtr Joint_3;
        physics::JointPtr Joint_4;
        physics::JointPtr Joint_5;
        physics::JointPtr Joint_6;

        //* ROS typedefs
        ros::NodeHandle nh;
        
        ros::Publisher J1_pub;
        ros::Publisher J2_pub;
        ros::Publisher J3_pub;
        ros::Publisher J4_pub;
        ros::Publisher J5_pub;
        ros::Publisher J6_pub;

        ros::Subscriber joySub;

        drok4_arm_pkgs::Gripper Joint_1_msg;
        drok4_arm_pkgs::Gripper Joint_2_msg;
        drok4_arm_pkgs::Gripper Joint_3_msg;
        drok4_arm_pkgs::Gripper Joint_4_msg;
        drok4_arm_pkgs::Gripper Joint_5_msg;
        drok4_arm_pkgs::Gripper Joint_6_msg;

        //std_msgs::Float64 J1_msg;
        //std_msgs::Float64 J2_msg;
        //std_msgs::Float64 J3_msg;
        //std_msgs::Float64 J4_msg;
        //std_msgs::Float64 J5_msg;
        //std_msgs::Float64 J6_msg;

        //* Index setting for each joint
        
        enum
        {
            J1 = 0, J2, J3, J4, J5, J6
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot

        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;

        } ROBO_JOINT;
        ROBO_JOINT* joint;

    public:
        //*** Functions for RoK-3 Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
        void UpdateAlgorithm(); // Algorithm update while simulation

        void jointController(); // Joint Controller for each joint

        void GetJoints(); // Get each joint data from [physics::ModelPtr _model]
        void GetjointData(); // Get encoder data of each joint

        void initializeJoint(); // Initialize joint variables for joint control
        void SetJointPIDgain(); // Set each joint PID gain for joint control

        void PdControl(double P_coeff, double D_coeff);
        //bool Arrived(double dt);

        void JoyCallback(const sensor_msgs::Joy::ConstPtr &msg);

    };
    GZ_REGISTER_MODEL_PLUGIN(drok4_plugin);
}

//------------------------------------------------------------//
// Joystick callback function
//------------------------------------------------------------//
void gazebo::drok4_plugin::JoyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    press[0] = msg->buttons[0];
    press[1] = msg->buttons[1];
    press[2] = msg->buttons[2];
    press[3] = msg->buttons[3];
    press[4] = -(int)msg->axes[6];    //left : 1.0, right : -1.0

    for (int i = 0; i < 5; i++)
    {
        if (abs(prev_press[i]) == 1 && prev_press[i] * press[i] == 0) {
            pressed[i] = true;
        }
        if (time < T) pressed[i] = false;
    }

    if (pressed[1] == true)
    {
        //O, Home positioning button
        pressed[3] = false;
        ctrl_mode = 0;      //Home positioning
        time = 0.0;
        goal_cal_once = true;
        pressed[1] = false;
    }
    else if (pressed[0] == true)
    {
        //X, Joystick position control button
        pressed[3] = false;
        ctrl_mode = 1;      //E-e position ctrl
        pressed[0] = false;
    }
    else if (pressed[2] == true)
    {
        //Triangle, Joystick rotation control button
        pressed[3] = false;
        ctrl_mode = 2;      //E-e orientation ctrl
        pressed[2] = false;
    }
    else if (pressed[3] == true)
    {
        //Square, Manual joint control button (Joystick)
        ctrl_mode = 3;      //Manual ctrl
        goal_cal_once = true;
        if (pressed[4] == true && press[4] > 0) {
            if (joint_id == 5) joint_id = 0;
            else joint_id++;
            //joint_id = (joint_id == 5) ? (0) : (joint_id++);
            pressed[4] = false;
        }
        else if (pressed[4] == true && press[4] < 0) {
            if (joint_id == 0) joint_id = 5;
            else joint_id--;
            //joint_id = (joint_id == 0) ? (6) : (joint_id--);
            pressed[4] = false;
        }
    }

    if (ctrl_mode == 1)
    {
        joy.x = msg->axes[1] * 0.0005;
        joy.y = msg->axes[0] * -0.0005;
        joy.z = msg->axes[4] * 0.0005;
    }
    else if (ctrl_mode == 2)
    {
        joy.x = msg->axes[1] * 0.05 * D2R;
        joy.y = msg->axes[0] * -0.05 * D2R;
        joy.z = msg->axes[4] * 0.05 * D2R;
    }
    else if (ctrl_mode == 3)
    {
        joy.z = msg->axes[4] * 0.01 * D2R;
    }

    memcpy(prev_press, press, sizeof (prev_press));
    //cout << "\npressed = " << pressed[0] << '\t' << pressed[1] << '\t' << pressed[2]
    //     << '\t' << pressed[3] << '\t' << pressed[4] << endl;
    //cout << "\njoint_id = " << joint_id << endl;
}

//------------------------------------------------------------//
// Transformation functions between orientation expressions
//------------------------------------------------------------//
AngleAxis_ RotMatToAngleAxis(MatrixXd C)
{
    AngleAxis_ a_axis;
    Vector3d n;
    double th;

    th = acos((C(0, 0) + C(1, 1) + C(2, 2) - 1) / 2);

    if (fabs(th) < 0.001)
    {
        n << 0, 0, 0;
    }
    else
    {
        n << (C(2, 1) - C(1, 2)),\
                (C(0, 2) - C(2, 0)),\
                (C(1, 0) - C(0, 1));
        n = (1 / (2 * sin(th))) * n;
    }

    a_axis.n = n;
    a_axis.th = th;

    return a_axis;
}

MatrixXd AngleAxisToRotMat(AngleAxis_ a_axis)
{
    MatrixXd C(3, 3);
    Vector3d n = a_axis.n;
    double th = a_axis.th;

    if (fabs(th) < 0.001) {
        C << MatrixXd::Identity(3, 3);
    }

    double nx = n(0), ny = n(1), nz = n(2);
    double s = sin(th), c = cos(th);

    C << nx*nx*(1-c)+c, nx*ny*(1-c)-nx*s, nx*nz*(1-c)+ny*s,\
            nx*ny*(1-c)+nz*s, ny*ny*(1-c)+c, ny*nz*(1-c)-nz*s,\
            nz*nx*(1-c)-ny*s, ny*nz*(1-c)+nz*s, nz*nz*(1-c)+c;

    return C;
}

VectorXd RotMatToEulerZyx(MatrixXd C)
{
    //회전 행렬을 Euler ZYX로 변환
    VectorXd euler_zyx = VectorXd::Zero(3);

    euler_zyx(0) = atan2(C(1, 0), C(0, 0));
    euler_zyx(1) = atan2(-C(2, 0), sqrt(pow(C(2, 1), 2) + pow(C(2, 2), 2)));
    euler_zyx(2) = atan2(C(2, 1), C(2, 2));

    return euler_zyx;
}

MatrixXd EulerZyxToRotMat(double z_rot, double y_rot, double x_rot)
{
    MatrixXd Z_rot(3, 3), Y_rot(3, 3), X_rot(3, 3);

    double sz = sin(z_rot), cz = cos(z_rot);
    double sy = sin(y_rot), cy = cos(y_rot);
    double sx = sin(x_rot), cx = cos(x_rot);

    Z_rot << cz, -sz, 0,\
            sz, cz, 0,\
            0, 0, 1;
    Y_rot << cy, 0, sy,\
            0, 1, 0,\
            -sy, 0, cy;
    X_rot << 1, 0, 0,\
            0, cx, -sx,\
            0, sx, cx;

    return Z_rot * Y_rot * X_rot;
}

MatrixXd RpyToRotMat(double roll, double pitch, double yaw)
{
    using namespace Eigen;
    cout << "\nroll : " << roll*R2D << "\npitch : " << pitch*R2D << "\nyaw : " << yaw*R2D << endl;
    MatrixXd C(3, 3);

    AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Quaterniond q = rollAngle * pitchAngle * yawAngle;
    //Quaterniond q = yawAngle * pitchAngle * rollAngle;
    C = q.toRotationMatrix();

    return C;
}

VectorXd RotMatToRotVec(MatrixXd C)
{
    //Input: a rotation matrix C
    //Output: the rotational vector which describes the rotation C
    VectorXd phi(3), n(3);
    double th;

    th = acos((C(0, 0) + C(1, 1) + C(2, 2) - 1) / 2);
    if(fabs(th) < 0.001)
    {
         n << 0, 0, 0;
    }
    else
    {
        n << (C(2, 1) - C(1, 2)),\
                (C(0, 2) - C(2, 0)),\
                (C(1, 0) - C(0, 1));
        n = (1 / (2 * sin(th))) * n;
    }

    phi = th * n;

    return phi;
}

//------------------------------------------------------------//
// Practice 2. Forward Kinematics (fuction declaration)
// 1, 2, 5, 6번 관절은 음의 축방향으로 회전한다.
//------------------------------------------------------------//
MatrixXd GetTransformI0(void)
{
    //Global frame to base link frame
    //htm = homogeneous transformation matrix
    
    MatrixXd htmI0 = MatrixXd::Identity(4, 4);
    
    return htmI0;
}

MatrixXd JointToTransform01(VectorXd q)
{
    //Base link frame to joint 1 frame
    //q : Generalized coordinates, q = [q1; q2; q3; q4; q5; q6]
    //q(0) : Angle of joint 1

    double q1 = q(0);
    
    double sq = sin(q1);
    double cq = cos(q1);

    MatrixXd htm01(4, 4);
    htm01 << cq, -sq, 0, 0,\
            sq, cq, 0, 0,\
            0, 0, 1, 0,\
            0, 0, 0, 1;
    
    return htm01;
}

MatrixXd JointToTransform12(VectorXd q)
{
    //q(1) : Angle of joint 2
    
    double q2 = q(1);
    
    double sq = sin(q2);
    double cq = cos(q2);

    MatrixXd htm12(4, 4);
    htm12 << cq, 0, sq, 0,\
            0, 1, 0, 0,\
            -sq, 0, cq, 0.161,\
            0, 0, 0, 1;
    
    return htm12;
}

MatrixXd JointToTransform23(VectorXd q)
{
    //q(2) : Angle of joint 3
    
    double q3 = q(2);
    
    double sq = sin(q3);
    double cq = cos(q3);

    MatrixXd htm23(4, 4);
    htm23 << cq, 0, sq, -0.605,\
            0, 1, 0, 0,\
            -sq, 0, cq, 0.002,\
            0, 0, 0, 1;

    return htm23;
}

MatrixXd JointToTransform34(VectorXd q)
{
    //q(3) : Angle of joint 4
    
    double q4 = q(3);
    
    double sq = sin(q4);
    double cq = cos(q4);

    MatrixXd htm34(4, 4);
    htm34 << 1, 0, 0, 0.507,\
            0, cq, -sq, 0,\
            0, sq, cq, 0.095,\
            0, 0, 0, 1;
    
    return htm34;
}

MatrixXd JointToTransform45(VectorXd q)
{
    //q(4) : angle of joint 5
    
    double q5 = q(4);
    
    double sq = sin(q5);
    double cq = cos(q5);
    
    MatrixXd htm45(4, 4);
    htm45 << cq, 0, sq, 0.096,\
            0, 1, 0, 0,\
            -sq, 0, cq, 0,\
            0, 0, 0, 1;
    
    return htm45;
}

MatrixXd JointToTransform56(VectorXd q)
{
    //q(5) : Angle of joint 6
    
    double q6 = q(5);
    
    double sq = sin(q6);
    double cq = cos(q6);
    
    MatrixXd htm56(4, 4);
    htm56 << 1, 0, 0, 0.109,\
            0, cq, -sq, 0,\
            0, sq, cq, -0.002,\
            0, 0, 0, 1;
    
    return htm56;
}

MatrixXd GetTransform6E(void)
{
    //두 좌표계 사이 회전 관계는 없고 x 방향 거리 관계만 존재한다.
    
    MatrixXd htm6E(4, 4);
    htm6E << 1, 0, 0, 0.15,\
            0, 1, 0, 0,\
            0, 0, 1, 0,\
            0, 0, 0, 1;
    
    return htm6E;
}

VectorXd JointToPosition(VectorXd q)
{
    MatrixXd TI0(4, 4), T01(4, 4), T12(4, 4), T23(4, 4), T34(4, 4), T45(4, 4),\
            T56(4, 4), T6E(4, 4), TIE(4, 4);
    TI0 = GetTransformI0();
    T01 = JointToTransform01(q);
    T12 = JointToTransform12(q);
    T23 = JointToTransform23(q);
    T34 = JointToTransform34(q);
    T45 = JointToTransform45(q);
    T56 = JointToTransform56(q);
    T6E = GetTransform6E();
    
    TIE = TI0 * T01 * T12 * T23 * T34 * T45 * T56 * T6E;
    
    Vector3d position;
    position = TIE.block(0, 3, 3, 1);
    
    //Block ocommentperation extracts a rectangular part of a matrix or array.
    //Block of size (p, q), starting at (i, j)
    //Version constructing a dynamic-size block expression = matrix.block(i, j, p, q)
    //Version constructing a fixed-size block expression = matrix.block<p, q>(i, j)
    
    return position;
}

MatrixXd JointToRotMat(VectorXd q)
{
    MatrixXd TI0(4, 4), T01(4, 4), T12(4, 4), T23(4, 4), T34(4, 4), T45(4, 4),\
            T56(4, 4), T6E(4, 4), TIE(4, 4);
    TI0 = GetTransformI0();
    T01 = JointToTransform01(q);
    T12 = JointToTransform12(q);
    T23 = JointToTransform23(q);
    T34 = JointToTransform34(q);
    T45 = JointToTransform45(q);
    T56 = JointToTransform56(q);
    T6E = GetTransform6E();
    
    TIE = TI0 * T01 * T12 * T23 * T34 * T45 * T56 * T6E;
    
    MatrixXd rot_m(3, 3);
    rot_m = TIE.block(0, 0, 3, 3);
    
    return rot_m;
}

//------------------------------------------------------------//
// Practice 3. Geometric Jacobian (fuction declaration)
//------------------------------------------------------------//
MatrixXd JointToPosJac(VectorXd q)
{
    //Input : Vector of generalized coordinates (joint angles)
    //Output : J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3, 6);
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_45(4, 4), T_56(4, 4), T_6E(4, 4);
    MatrixXd T_I1(4, 4), T_I2(4, 4), T_I3(4, 4), T_I4(4, 4), T_I5(4, 4), T_I6(4, 4);
    MatrixXd R_I1(3, 3), R_I2(3, 3), R_I3(3, 3), R_I4(3, 3), R_I5(3, 3), R_I6(3, 3);
    Vector3d r_I_I1, r_I_I2, r_I_I3, r_I_I4, r_I_I5, r_I_I6;
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;
    Vector3d n_I_1, n_I_2, n_I_3, n_I_4, n_I_5, n_I_6;
    Vector3d r_I_IE;

    //Compute the relative homogeneous transformation matrices.
    T_I0 = GetTransformI0();
    T_01 = JointToTransform01(q);
    T_12 = JointToTransform12(q);
    T_23 = JointToTransform23(q);
    T_34 = JointToTransform34(q);
    T_45 = JointToTransform45(q);
    T_56 = JointToTransform56(q);
    T_6E = GetTransform6E();

    //Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0 * T_01;
    T_I2 = T_I0 * T_01 * T_12;
    T_I3 = T_I0 * T_01 * T_12 * T_23;
    T_I4 = T_I0 * T_01 * T_12 * T_23 * T_34;
    T_I5 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45;
    T_I6 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45 * T_56;

    //Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0, 0, 3, 3);
    R_I2 = T_I2.block(0, 0, 3, 3);
    R_I3 = T_I3.block(0, 0, 3, 3);
    R_I4 = T_I4.block(0, 0, 3, 3);
    R_I5 = T_I5.block(0, 0, 3, 3);
    R_I6 = T_I6.block(0, 0, 3, 3);

    //Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = T_I1.block(0, 3, 3, 1);
    r_I_I2 = T_I2.block(0, 3, 3, 1);
    r_I_I3 = T_I3.block(0, 3, 3, 1);
    r_I_I4 = T_I4.block(0, 3, 3, 1);
    r_I_I5 = T_I5.block(0, 3, 3, 1);
    r_I_I6 = T_I6.block(0, 3, 3, 1);

    //Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0, 0, 1;
    n_2 << 0, 1, 0;
    n_3 << 0, 1, 0;
    n_4 << 1, 0, 0;
    n_5 << 0, 1, 0;
    n_6 << 1, 0, 0;

    //Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1 * n_1;
    n_I_2 = R_I2 * n_2;
    n_I_3 = R_I3 * n_3;
    n_I_4 = R_I4 * n_4;
    n_I_5 = R_I5 * n_5;
    n_I_6 = R_I6 * n_6;

    //Compute the end-effector position vector.
    r_I_IE = (T_I6 * T_6E).block(0, 3, 3, 1);       //T_I6 * T_6E = TIEs

    //Compute the translational Jacobian. Use cross of EIGEN.
    J_P.col(0) << n_I_1.cross(r_I_IE - r_I_I1);
    J_P.col(1) << n_I_2.cross(r_I_IE - r_I_I2);
    J_P.col(2) << n_I_3.cross(r_I_IE - r_I_I3);
    J_P.col(3) << n_I_4.cross(r_I_IE - r_I_I4);
    J_P.col(4) << n_I_5.cross(r_I_IE - r_I_I5);
    J_P.col(5) << n_I_6.cross(r_I_IE - r_I_I6);

    //std::cout << "Test, JP:" << std::endl << J_P << std::endl;

    return J_P;
}

MatrixXd JointToRotJac(VectorXd q)
{
    //Input : Vector of generalized coordinates (joint angles)
    //Output : J_R, Jacobian of the end-effector orientation which maps joint velocities to end-effector angular velocities in I frame.
    MatrixXd J_R(3, 6);
    MatrixXd T_I0(4, 4), T_01(4, 4), T_12(4, 4), T_23(4, 4), T_34(4, 4), T_45(4, 4), T_56(4, 4), T_6E(4, 4);
    MatrixXd T_I1(4, 4), T_I2(4, 4), T_I3(4, 4), T_I4(4, 4), T_I5(4, 4), T_I6(4, 4);
    MatrixXd R_I1(3, 3), R_I2(3, 3), R_I3(3, 3), R_I4(3, 3), R_I5(3, 3), R_I6(3, 3);
    Vector3d n_1, n_2, n_3, n_4, n_5, n_6;

    //Compute the relative homogeneous transformation matrices.
    T_I0 = GetTransformI0();
    T_01 = JointToTransform01(q);
    T_12 = JointToTransform12(q);
    T_23 = JointToTransform23(q);
    T_34 = JointToTransform34(q);
    T_45 = JointToTransform45(q);
    T_56 = JointToTransform56(q);
    T_6E = GetTransform6E();

    //Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = T_I0 * T_01;
    T_I2 = T_I0 * T_01 * T_12;
    T_I3 = T_I0 * T_01 * T_12 * T_23;
    T_I4 = T_I0 * T_01 * T_12 * T_23 * T_34;
    T_I5 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45;
    T_I6 = T_I0 * T_01 * T_12 * T_23 * T_34 * T_45 * T_56;

    //Extract the rotation matrices from each homogeneous transformation matrix.
    R_I1 = T_I1.block(0, 0, 3, 3);
    R_I2 = T_I2.block(0, 0, 3, 3);
    R_I3 = T_I3.block(0, 0, 3, 3);
    R_I4 = T_I4.block(0, 0, 3, 3);
    R_I5 = T_I5.block(0, 0, 3, 3);
    R_I6 = T_I6.block(0, 0, 3, 3);

    //Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0, 0, 1;
    n_2 << 0, 1, 0;
    n_3 << 0, 1, 0;
    n_4 << 1, 0, 0;
    n_5 << 0, 1, 0;
    n_6 << 1, 0, 0;

    //Compute the translational Jacobian.
    J_R.col(0) << R_I1 * n_1;
    J_R.col(1) << R_I2 * n_2;
    J_R.col(2) << R_I3 * n_3;
    J_R.col(3) << R_I4 * n_4;
    J_R.col(4) << R_I5 * n_5;
    J_R.col(5) << R_I6 * n_6;

    //std::cout << "Test, J_R:" << std::endl << J_R << std::endl;

    return J_R;
}

//------------------------------------------------------------//
// Practice 4. Pseudo-inverse (fuction declaration)
//------------------------------------------------------------//
MatrixXd PseudoInverseMat(MatrixXd A, double lambda)
{
    //Input : Any m-by-n matrix
    //Output : An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    MatrixXd pinvA, I;
    int m = A.rows(), n = A.cols();
    
    //실인수 행렬 A의 행 m과 열 n의 크기를 비교한다.
    //m > n이고 rank(A) = n인 경우, if()문 내의 left pseudo-inverse 코드를 실행한다.
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A);
    
    if (m > n && lu_decomp.rank() == n)
    {
        I = MatrixXd::Identity(n, n);
        pinvA = (A.transpose() * A + pow(lambda, 2) * I).inverse() * A.transpose();
    }
    else if (m < n && lu_decomp.rank() == m)
    {
        I = MatrixXd::Identity(m, m);
        pinvA = A.transpose() * (A * A.transpose() + pow(lambda, 2) * I).inverse();
    }
    else pinvA = A.inverse();
    
    return pinvA;
}

//------------------------------------------------------------//
// Practice 5. Inverse kinematics (fuction declaration)
//------------------------------------------------------------//
VectorXd InverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol)
{
    //Input : desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    //Output : joint angles which match desired end-effector position and orientation
    
    clock_gettime(CLOCK_MONOTONIC, &ik_start_time);         //IK 시작 시각을 ik_start_time 변수에 저장한다.
    
    double num_it = 0;
    MatrixXd J_P(3, 6), J_R(3, 6), J(6, 6), pinvJ(6, 6), C_err(3, 3), C_IE(3, 3);
    VectorXd q(6), dq(6), dXe(6);
    Vector3d dr, dph;
    double lambda;
    
    //Set maximum number of iterations
    double max_it = 200;
    
    //Initialize the solution with the initial guess
    q = q0;
    C_IE = JointToRotMat(q);     //q0로 구한 end-effector의 orientation
    C_err = C_des * C_IE.transpose();
    
    //Damping factor
    lambda = 0.001;
    
    //Initialize error
    dr = r_des - JointToPosition(q);
    dph = RotMatToRotVec(C_err);
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
    
    ////////////////////////////////////////////////
    // Iterative inverse kinematics
    ////////////////////////////////////////////////
    
    //Iterate until terminating condition
    while (num_it < max_it && dXe.norm() > tol)
    {
        
        //Compute Inverse Jacobian
        J_P = JointToPosJac(q);
        J_R = JointToRotJac(q);

        J.block(0, 0, 3, 6) = J_P;
        J.block(3, 0, 3, 6) = J_R; // Geometric Jacobian
        
        //Convert to Geometric Jacobian to Analytic Jacobian
        dq = PseudoInverseMat(J, lambda) * dXe;
        
        //Update law
        q += 0.5 * dq;
        
        //Update error
        C_IE = JointToRotMat(q);
        C_err = C_des * C_IE.transpose();
        
        dr = r_des - JointToPosition(q);
        dph = RotMatToRotVec(C_err);
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
                   
        num_it++;
    }
    
    clock_gettime(CLOCK_MONOTONIC, &ik_end_time);
    long nano_sec_dt = ik_end_time.tv_nsec - ik_start_time.tv_nsec;
    
    std::cout << "\niteration : " << num_it << ", value : \n" << q * R2D << std::endl;
    std::cout << "IK dt (us) : " << nano_sec_dt/1000 << std::endl;
    
    return q;
}

//------------------------------------------------------------//
// Practice 6. 1 - cos trajectory planning
//------------------------------------------------------------//
double Func_1_cos(double init, double final, double t, double T)
{
    double des;
    
    des = init + (final - init) * 0.5 * (1 - cos(PI * t/T));
    
    return des;
}

Vector3d Func_1_cos(Vector3d init, Vector3d final, double t, double T)
{
    Vector3d des;

    des(0) = init(0) + (final(0) - init(0)) * 0.5 * (1 - cos(PI * t/T));
    des(1) = init(1) + (final(1) - init(1)) * 0.5 * (1 - cos(PI * t/T));
    des(2) = init(2) + (final(2) - init(2)) * 0.5 * (1 - cos(PI * t/T));
    
    return des;
}

double Func_1_cos_yaw(double start, double end, double t, double T)
{
    double yaw;
    double delta = end - start;

    if (fabs(delta) > PI && delta >= 0)
        delta = delta - 2 * PI;
    else if (fabs(delta) > PI && delta < 0)
        delta = delta + 2 * PI;

    yaw = start + delta * 0.5 * (1 - cos(PI * (t/T)));
    cout << "\nstart = " << start*R2D << ", end = " << end*R2D << endl;
    cout << "delta = " << delta*R2D << endl;

    return yaw;
}

void Practice(void)
{
    //--------------------------------------------------//
    // Practice 5. Inverse kinematics
    //--------------------------------------------------//
    printf("\n==============================\n");
    std::cout << "Practice 5 start" << std::endl;
    printf("==============================\n");
    
    MatrixXd C_des(3, 3);
    VectorXd r_des(3), q(6), q_cal(6);
    
    q << 10, 20, 30, 40, 50, 60;
    q *= D2R;
    
    r_des = JointToPosition(q);
    C_des = JointToRotMat(q);
    
    std::cout << "\nMission 1 : \n" << std::endl;
    std::cout << "Answer q (deg) = \n" << q * R2D << endl;
    std::cout << "\nInitial guess q0 (deg) = \n" << q * R2D * 0.5 << endl;
    
    q_cal = InverseKinematics(r_des, C_des, q * 0.5, 0.001);
    std::cout << "\nq_cal (deg) = \n" << q_cal * R2D << endl;
    
    goal_posi << 0.6, 0.2, 0.05;
    goal_rot = MatrixXd::Identity(3, 3);
    q0 << 30, -30, -60, 10, 30, 10;
    q_goal = InverseKinematics(goal_posi, goal_rot, q0 * D2R, 0.001);
    goal_posi = JointToPosition(q_goal);
    cout << "\nq_goal = \n" << q_goal * R2D << endl;
    cout << "\ngoal_posi = \n" << goal_posi << endl;

    goal_posi(Z_) -= 0.02;
    goal_rot = MatrixXd::Identity(3, 3);
    q0 = q_goal;
    q_goal = InverseKinematics(goal_posi, goal_rot, q0, 0.001);
    goal_posi = JointToPosition(q_goal);
    cout << "\nq_goal = \n" << q_goal * R2D << endl;
    cout << "\ngoal_posi = \n" << goal_posi << endl;

    /*
    q_goal << 30, -30, -60, 0, 30, 0;
    goal_posi = JointToPosition(q_goal * D2R);
    cout << "\nq_goal = \n" << q_goal << endl;
    cout << "\ngoal_posi = \n" << goal_posi << endl;
    //goal_posi의 값 = 0.424, -0.243, 0.047

    goal_posi << 0.424, -0.243, 0.047;
    goal_rot = MatrixXd::Identity(3, 3);
    q0 << 30, -30, -60, 10, 30, 10;
    q_goal = InverseKinematics(goal_posi, goal_rot, q0 * D2R * 0.5, 0.001);
    goal_posi = JointToPosition(q_goal);
    cout << "\nq_goal = \n" << q_goal * R2D << endl;
    cout << "\ngoal_posi = \n" << goal_posi << endl;
    */
    printf("\n==============================\n");
    std::cout << "Practice 5 end" << std::endl;
    printf("==============================\n");
}

void Examination(void)
{
    //--------------------------------------------------//
    // Examination
    //--------------------------------------------------//
    printf("\n==============================\n");
    std::cout << "Examination start" << std::endl;
    printf("==============================\n");

    //1. 입력한 각도의 FK 결과물과 Gazebo 상 end-point의 절대 좌표 비교
    //q = [20; -20; -20; 20; 20; 20;]
    VectorXd q(6);
    q << 0, 75, -70, 0, -5, 0;
    q *= D2R;
    goal_posi = JointToPosition(q);
    goal_rot = JointToRotMat(q);
    cout << "\nq = \n" << q * R2D << endl;
    cout << "\ngoal position = \n" << goal_posi << endl;
    cout << "\ngoal_rotation = \n" << goal_rot << endl;

    //2. 각 joint별 위치 비교
    MatrixXd TI0(4, 4), T01(4, 4), T12(4, 4), T23(4, 4), T34(4, 4), T45(4, 4),\
            T56(4, 4), T6E(4, 4), TIE(4, 4), rotMat(3, 3);
    VectorXd posVec(3);
    TI0 = GetTransformI0();
    T01 = JointToTransform01(q);
    posVec = (TI0 * T01).block(0, 3, 3, 1);
    rotMat = (TI0 * T01).block(0, 0, 3, 3);
    cout << "\nPosition of Joint 1 = \n" << posVec << endl;
    cout << "\nRotation of Joint 1 = \n" << rotMat << endl;

    T12 = JointToTransform12(q);
    posVec = (TI0 * T01 * T12).block(0, 3, 3, 1);
    rotMat = (TI0 * T01 * T12).block(0, 0, 3, 3);
    cout << "\nPosition of Joint 2 = \n" << posVec << endl;
    cout << "\nRotation of Joint 2 = \n" << rotMat << endl;

    T23 = JointToTransform23(q);
    posVec = (TI0 * T01 * T12 * T23).block(0, 3, 3, 1);
    rotMat = (TI0 * T01 * T12 * T23).block(0, 0, 3, 3);
    cout << "\nPosition of Joint 3 = \n" << posVec << endl;
    cout << "\nRotation of Joint 3 = \n" << rotMat << endl;

    T34 = JointToTransform34(q);
    posVec = (TI0 * T01 * T12 * T23 * T34).block(0, 3, 3, 1);
    rotMat = (TI0 * T01 * T12 * T23 * T34).block(0, 0, 3, 3);
    cout << "\nPosition of Joint 4 = \n" << posVec << endl;
    cout << "\nRotation of Joint 4 = \n" << rotMat << endl;

    T45 = JointToTransform45(q);
    posVec = (TI0 * T01 * T12 * T23 * T34 * T45).block(0, 3, 3, 1);
    rotMat = (TI0 * T01 * T12 * T23 * T34 * T45).block(0, 0, 3, 3);
    cout << "\nPosition of Joint 5 = \n" << posVec << endl;
    cout << "\nRotation of Joint 5 = \n" << rotMat << endl;

    T56 = JointToTransform56(q);
    posVec = (TI0 * T01 * T12 * T23 * T34 * T45 * T56).block(0, 3, 3, 1);
    rotMat = (TI0 * T01 * T12 * T23 * T34 * T45 * T56).block(0, 0, 3, 3);
    cout << "\nPosition of Joint 6 = \n" << posVec << endl;
    cout << "\nRotation of Joint 6 = \n" << rotMat << endl;

    goal_posi << 0.4, 0, 0.8;//0.7, 0, 0.8;
    //goal_rot = EulerZyxToRotMat(90*D2R, -90*D2R, 90*D2R);
    goal_rot = MatrixXd::Identity(3, 3);
    //a_axis.n << 0, 1, 0;
    //a_axis.th = 0;
    //goal_rot = AngleAxisToRotMat(a_axis);
    q0 << 0, 45, -55, 0, 10, 0;//0, 75, -70, 0, 5, 0;
    q0 *= D2R;
    q_goal = InverseKinematics(goal_posi, goal_rot, q0, 0.001);
    cout << "\nq0 = \n" << q0 * R2D << endl;
    cout << "\nq_goal = \n" << q_goal * R2D << endl;

    double yaw;
    yaw = Func_1_cos_yaw(-30*D2R, 91*D2R, 0.001, 4);
    yaw = Func_1_cos_yaw(170*D2R, -170*D2R, 0.001, 4);
    yaw = Func_1_cos_yaw(40*D2R, 290*D2R, 0.001, 4);
    yaw = Func_1_cos_yaw(20*D2R, 200*D2R, 0.001, 4);
    yaw = Func_1_cos_yaw(-200*D2R, 20*D2R, 0.001, 4);

    printf("\n==============================\n");
    std::cout << "Examination end" << std::endl;
    printf("==============================\n");
}

void gazebo::drok4_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    /*
     * Loading model data and initializing the system before simulation 
     */

    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    //* [physics::ModelPtr model] based model update
    GetJoints();



    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_GREEN "RBDL API version = %d\n" C_RESET, version_test);

    //* model.urdf file based model data input to [Model* solid_arm_3] for using RBDL
    Model* solid_arm_3 = new Model();
    Addons::URDFReadFromFile("/home/drokrc/.gazebo/models/solid_arm_3/urdf/solid_arm_3.urdf", solid_arm_3, true, true);
    //↑↑↑ Check File Path ↑↑↑
    nDoF = solid_arm_3->dof_count - 6; // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct
    
    
    //* initialize and setting for robot control in gazebo simulation
    initializeJoint();
    SetJointPIDgain();
    
    
    //* setting for getting dt
    //last_update_time = model->GetWorld()->GetSimTime();
    #if GAZEBO_MAJOR_VERSION >= 8
        last_update_time = model->GetWorld()->SimTime();
    #else
        last_update_time = model->GetWorld()->GetSimTime();
    #endif

    q_init << 0, 0, 0, 0, 0, 0;
    q_goal << 0, 0, 0, 0, 0, 0;
    q_present << 0, 0, 0, 0, 0, 0;
    q_command << 0, 0, 0, 0, 0, 0;
    ori.roll = 0;
    ori.pitch = 0;
    ori.yaw = 0;

    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&drok4_plugin::UpdateAlgorithm, this));

    //* ROS subscriber
    joySub = nh.subscribe("joy", 1000, &gazebo::drok4_plugin::JoyCallback, this);
    //joySub = nh.subscribe("joy", 1000, JoyCallback);
    
    //* Kinematics Test
   /// Examination();
    //RpyToQuaternion();

    //* ROS publishers
    J1_pub = nh.advertise<drok4_arm_pkgs::Gripper>("command_joint/J1", 1000);
    J2_pub = nh.advertise<drok4_arm_pkgs::Gripper>("command_joint/J2", 1000);
    J3_pub = nh.advertise<drok4_arm_pkgs::Gripper>("command_joint/J3", 1000);
    J4_pub = nh.advertise<drok4_arm_pkgs::Gripper>("command_joint/J4", 1000);
    J5_pub = nh.advertise<drok4_arm_pkgs::Gripper>("command_joint/J5", 1000);
    J6_pub = nh.advertise<drok4_arm_pkgs::Gripper>("command_joint/J6", 1000);

    //J1_pub = nh.advertise<std_msgs::Float64>("command_joint / J1", 1000);
    //J2_pub = nh.advertise<std_msgs::Float64>("command_joint / J2", 1000);
    //J3_pub = nh.advertise<std_msgs::Float64>("command_joint / J3", 1000);
    //J4_pub = nh.advertise<std_msgs::Float64>("command_joint / J4", 1000);
    //J5_pub = nh.advertise<std_msgs::Float64>("command_joint / J5", 1000);
    //J6_pub = nh.advertise<std_msgs::Float64>("command_joint / J6", 1000);
}

void gazebo::drok4_plugin::UpdateAlgorithm()
{
    /*
     * Algorithm update while simulation
     */

    //* UPDATE TIME : 1ms
    //common::Time current_time = model->GetWorld()->GetSimTime();
    #if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = model->GetWorld()->SimTime();
    #else
        common::Time current_time = model->GetWorld()->GetSimTime();
    #endif

    dt = current_time.Double() - last_update_time.Double();
    //    cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;

    //updateTime += 1;
    //cout << "updateTime = " << updateTime << endl;

    //* Read Sensors data
    GetjointData();

    //--------------------------------------------------//
    // Joystick Control #1
    //--------------------------------------------------//

    int i;
    if (ctrl_mode == 0 && goal_cal_once == true)
    {
        // Home positioning //
        T = 4.0;
        if (phase == 0) {
            goal_posi << 0.7, 0, 0.8;
            goal_rot = MatrixXd::Identity(3, 3);
            q0 << 0, -75, -70, 0, 5, 0;
            q0 *= D2R;
        }
        q_goal = InverseKinematics(goal_posi, goal_rot, q0, 0.001);
        q0 = q_goal;
        goal_cal_once = false;
    }
    else if (ctrl_mode == 1)
    {
        // Position control //
        start_posi = goal_posi;
        start_rot = goal_rot;
        goal_posi(X_) = start_posi(X_) + joy.x;
        goal_posi(Y_) = start_posi(Y_) + joy.y;
        goal_posi(Z_) = start_posi(Z_) + joy.z;
        if (sqrt(pow(goal_posi(X_), 2) + pow(goal_posi(Y_), 2)) < 0.30) {
            // Position inner limint examination, r = 0.35 m //
            goal_posi(X_) = start_posi(X_);
            goal_posi(Y_) = start_posi(Y_);
        }
        command_posi = goal_posi;
        command_rot = goal_rot;
        q_command = InverseKinematics(command_posi, command_rot, q0, 0.001);
        q0 = q_command;
    }
    else if (ctrl_mode == 2)
    {
        // Rotation control //
        start_posi = goal_posi;
        start_rot = goal_rot;
        Vector3d temp_Euler;
        temp_Euler = RotMatToEulerZyx(start_rot);
        ori.roll = temp_Euler(2) + joy.x;
        ori.pitch = temp_Euler(1) + joy.y;
        ori.yaw = temp_Euler(0) + joy.z;
        goal_rot = EulerZyxToRotMat(ori.yaw, ori.pitch, ori.roll);
        //goal_rot = RpyToRotMat(ori.roll, ori.pitch, ori.yaw);
        //C_err = goal_rot * start_rot.transpose();
        //a_axis = RotMatToAngleAxis(C_err);
        command_posi = goal_posi;
        command_rot = goal_rot;
        //command_rot = start_rot * AngleAxisToRotMat(a_axis);
        q_command = InverseKinematics(command_posi, command_rot, q0, 0.001);
        q0 = q_command;
    }
    else if (ctrl_mode == 3)
    {
        if (goal_cal_once == true) {
            q_goal = q_command;
            goal_cal_once = false;
        }
        cout << "Manual joint control : Joint " << joint_id + 1 << endl;
        //q_goal = q_command;
        q_goal(joint_id) += joy.z;
        goal_posi = JointToPosition(q_goal);
        goal_rot = JointToRotMat(q_goal);
        q_command = q_goal;
        q0 = q_command;
    }

    if (ctrl_mode == 0)
    {
        T = 4.0;
        if (time < T) {
            joint[J1].targetRadian = Func_1_cos(q_init(0), q_goal(0), time, T);
            joint[J2].targetRadian = Func_1_cos(q_init(1), q_goal(1), time, T);
            joint[J3].targetRadian = Func_1_cos(q_init(2), q_goal(2), time, T);
            joint[J4].targetRadian = Func_1_cos(q_init(3), q_goal(3), time, T);
            joint[J5].targetRadian = Func_1_cos(q_init(4), q_goal(4), time, T);
            joint[J6].targetRadian = Func_1_cos(q_init(5), q_goal(5), time, T);
        }
        else if (time >= T) {
            joint[J1].targetRadian = q_goal(0);
            joint[J2].targetRadian = q_goal(1);
            joint[J3].targetRadian = q_goal(2);
            joint[J4].targetRadian = q_goal(3);
            joint[J5].targetRadian = q_goal(4);
            joint[J6].targetRadian = q_goal(5);
            q_init = q_goal;
            q_command = q_goal;
        }
    }
    else
    {
        VectorXd q_exam(6);
        q_exam = q_command - q_present;
        if (fabs(q_exam(J6)) > 5 * D2R || fabs(q_exam(J5)) > 5 * D2R ||
                fabs(q_exam(J4)) > 5 * D2R || fabs(q_exam(J3) > 5 * D2R) ||
                fabs(q_exam(J2)) > 5 * D2R || fabs(q_exam(J1) > 5 * D2R))
        {
            q_command = q_present;
            goal_posi = JointToPosition(q_command);
            goal_rot = JointToRotMat(q_command);
        }
        joint[J1].targetRadian = q_command(0);
        joint[J2].targetRadian = q_command(1);
        joint[J3].targetRadian = q_command(2);
        joint[J4].targetRadian = q_command(3);
        joint[J5].targetRadian = q_command(4);
        joint[J6].targetRadian = q_command(5);
        q_init = q_command;
    }

    if (ctrl_mode == 3)
        cout << "\nq_command = \n" << q_command << endl;

    //--------------------------------------------------//
    // Joint space -> Task space Control
    //--------------------------------------------------//
    //Home position : [0; 0.4; 0.4]
    //Homing(joint) -> Circle(task) -> Homing(joint)
    //AngleAxis_ term_a_axis = a_axis;
    //MatrixXd term_C;
    //int i;

    /*
    if (ctrl_mode == 0 && goal_cal_once == true)
    {
        if (phase == 0) {
            goal_posi << 0.4, 0, 0.8;//0.7, 0, 0.8;
            //goal_rot = EulerZyxToRotMat(90*D2R, -90*D2R, 90*D2R);
            goal_rot = MatrixXd::Identity(3, 3);
            //a_axis.n << 0, 1, 0;
            //a_axis.th = 0;
            //goal_rot = AngleAxisToRotMat(a_axis);
            q0 << 0, 45, -55, 0, 10, 0;//0, -75, -70, 0, 5, 0;
            q0 *= D2R;
            q_goal = InverseKinematics(goal_posi, goal_rot, q0, 0.001);
            q0 = q_goal;

            //start_posi = goal_posi;
            //start_rot = goal_rot;
            goal_cal_once = false;
        }
    }
    else if (ctrl_mode != 0)
    {
        if (time < T && phase == 0) {
            if (joy_goal_cal == true) {
                start_posi = goal_posi;
                start_rot = goal_rot;
                //goal_posi(X_) -= 0.2;
                //goal_posi(Y_) += 0.2;
                goal_rot = EulerZyxToRotMat(-10*D2R, 0*D2R, 0*D2R);
                //goal_rot = RpyToRotMat(30*D2R, 0*D2R, 90*D2R);
                C_err = goal_rot * start_rot.transpose();
                a_axis = RotMatToAngleAxis(C_err);
                joy_goal_cal = false;
            }
            AngleAxis_ command_a_axis = a_axis;
            command_posi = Func_1_cos(start_posi, goal_posi, time, T);
            command_a_axis.th = Func_1_cos(0, a_axis.th, time, T);
            cout << "\ncommand_a_axis.n = \n" << command_a_axis.n << endl;
            cout << "\ncommand_a_axis.th = " << command_a_axis.th*R2D << endl;
            command_rot = start_rot * AngleAxisToRotMat(command_a_axis);

            q_command = InverseKinematics(command_posi, command_rot, q0, 0.001);
            q0 = q_command;
        }
        else if (time < T && phase == 1) {
            if (joy_goal_cal == true) {
                start_posi = goal_posi;
                start_rot = goal_rot;
                //goal_posi(Y_) -= 0.2;
                goal_rot = MatrixXd::Identity(3, 3);
                C_err = goal_rot * start_rot.transpose();
                a_axis = RotMatToAngleAxis(C_err);
                joy_goal_cal = false;
            }
            AngleAxis_ command_a_axis = a_axis;
            command_posi = Func_1_cos(start_posi, goal_posi, time, T);
            command_a_axis.th = Func_1_cos(0, a_axis.th, time, T);
            cout << "\ncommand_a_axis.n = \n" << command_a_axis.n << endl;
            cout << "\ncommand_a_axis.th = " << command_a_axis.th*R2D << endl;
            command_rot = start_rot * AngleAxisToRotMat(command_a_axis);

            q_command = InverseKinematics(command_posi, command_rot, q0, 0.001);
            q0 = q_command;
        }
        else if (time >= T && phase == 0) {
            time = 0;
            phase ++;
            joy_goal_cal = true;
        }
        else if (time >= T && phase == 1) {
            time = 0;
            phase = 0;
            joy_goal_cal = true;
        }
        //command_rot = goal_rot;
        //q_command = InverseKinematics(command_posi, command_rot, q0, 0.001);
        //q0 = q_command;
        //cout << "\nphase = " << phase << endl;
    }

    if (homing == 1) {
        T = 4.0;
        if (time < T) {
            joint[J1].targetRadian = Func_1_cos(q_init(0), q_goal(0), time, T);
            joint[J2].targetRadian = Func_1_cos(q_init(1), q_goal(1), time, T);
            joint[J3].targetRadian = Func_1_cos(q_init(2), q_goal(2), time, T);
            joint[J4].targetRadian = Func_1_cos(q_init(3), q_goal(3), time, T);
            joint[J5].targetRadian = Func_1_cos(q_init(4), q_goal(4), time, T);
            joint[J6].targetRadian = Func_1_cos(q_init(5), q_goal(5), time, T);
        }
        else if (time >= T) {
            joint[J1].targetRadian = q_goal(0);
            joint[J2].targetRadian = q_goal(1);
            joint[J3].targetRadian = q_goal(2);
            joint[J4].targetRadian = q_goal(3);
            joint[J5].targetRadian = q_goal(4);
            joint[J6].targetRadian = q_goal(5);
        }
    }
    else if (homing == -1) {
        joint[J1].targetRadian = q_command(0);
        joint[J2].targetRadian = q_command(1);
        joint[J3].targetRadian = q_command(2);
        joint[J4].targetRadian = q_command(3);
        joint[J5].targetRadian = q_command(4);
        joint[J6].targetRadian = q_command(5);
    }
    */

    for (i = 0; i < nDoF; i++) {
        q_present(i) = joint[i].targetRadian;
    }
    present_posi = JointToPosition(q_present);
    present_rot = JointToRotMat(q_present);
    //cout << "\nEulerZYX = \n" << RotMatToEulerZyx(present_rot) * R2D << endl;

    //cout << "\nq_present = \n" << q_present * R2D << endl;
    //cout << "\nstart_position = \n" << start_posi << endl;
    //cout << "\ngoal_position = \n" << goal_posi << endl;
    //cout << "\npresent_position = \n" << present_posi << endl;
    //cout << "\ncommand_position = \n" << command_posi << endl;

    //cout << "\npresent_rotation = \n" << present_rot << endl;
    //cout << "\nEuler_matrix(deg) = \n" << RotMatToEulerZyx(present_rot) * R2D << endl;
    //term_a_axis = RotMatToAngleAxis(present_rot);
    //cout << "\nAngle_axis(n) = \n" << term_a_axis.n << endl;
    //cout << "\nAngle_axis(th, deg) = \n" << term_a_axis.th * R2D << endl;
    //cout << "\nRotation_vector(Euler_vector) = \n" << RotMatToRotVec(present_rot) << endl;

    //* Joint Controller
    jointController();

    /*
    for (i = 0; i < 6; i++) {
        cout << "targetTorque " << i + 1 << " : " << joint[i].targetTorque << endl;
    }
    putchar('\n');
    for (i = 0; i < 6; i++) {
        cout << "actualTorque " << i + 1 << " : " << joint[i].actualTorque << endl;
    }
    */

    //ros::spinOnce();
    //loop_rate.sleep();
    //ros::waitForShutdown();
}

/*
bool SmallerThanTolVec(VectorXd vector, VectorXd tol, int size)
{
    int i;
    for (i = 0; i < size; i++) {
        if (fabs(vector(i)) > tol(i))
            return 0;
    }
    return 1;
}

bool gazebo::drok4_plugin::Arrived(double dtParam)
{
    //Arrived의 조건 1. target과 actual의 오차가 허용 선량 이내일 것.
    //Arrived의 조건 2. t - 1의 actual과 t의 actual의 변화율(혹은 오차)이 허용 선량 이내일 것.
    VectorXd EP_err(3), q_err(6), EP_err_tol(3), q_err_tol(6), EP_diff(3), q_diff(6), \
            EP_diff_tol(3), q_diff_tol(6);
    //Vector3d EP_err, EP_tol, EP_diff;
    //VectorXd q_err(6), q_tol(6), q_diff(6);
    //VectorXd EP_err(3), q_err(6), EP_tol(3), q_tol(6), EP_diff(3), q_diff(6);

    EP_err = command_posi - present_posi;
    q_err = q_command - q_present;

    //EP_err = goal_posi - present_posi;
    //q_err = q_goal - q_present;
    cout << "\nEnd point 오차 = \n" << EP_err << endl;
    cout << "\n관절각 오차(deg) = \n" << q_err * R2D << endl;

    EP_err_tol << 0.03, 0.03, 0.03;
    q_err_tol << 3, 3, 3, 3, 3, 3;
    q_err_tol *= D2R;

    dtParam = 1 / dtParam;
    EP_diff = (present_posi - prev_posi) * dtParam;
    q_diff = (q_present - q_prev) * dtParam;
    cout << "\nEnd_point 변위 = \n" << present_posi - prev_posi << endl;
    cout << "\n관절각 변화 = \n" << (q_present - q_prev) * R2D << endl;
    //cout << "\nEnd_point 변화율(기울기) = \n" << EP_diff << endl;
    //cout << "\n관절각 변화율 = \n" << q_diff << endl;

    //EP_diff_tol << 0.1, 0.1, 0.1;
    //EP_diff_tol *= dtParam;
    //q_diff_tol << 10, 10, 10, 10, 10, 10;
    //q_diff_tol *= dtParam * D2R;
    EP_diff_tol = EP_err_tol * dtParam;
    q_diff_tol = q_diff_tol * dtParam;

    if (homing == 1) {
        if (SmallerThanTolVec(q_err, q_err_tol, 6) == true \
                && SmallerThanTolVec(q_diff, q_diff_tol, 6))
            return 1;
        else
            return 0;
    }
    else if (homing == -1) {
        if ((SmallerThanTolVec(EP_err, EP_err_tol, 3) == true || SmallerThanTolVec(q_err, q_err_tol, 6)) \
               && (SmallerThanTolVec(EP_diff, EP_diff_tol, 3) == true || SmallerThanTolVec(q_diff, q_diff_tol, 6)))
           return 1;
       else
           return 0;
    }
}
*/

void gazebo::drok4_plugin::jointController()
{
    /*
     * Joint Controller for each joint
     */

    //putchar('\n');
    // Update target torque by control
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetTorque = joint[j].Kp * (joint[j].targetRadian - joint[j].actualRadian)\
                              + joint[j].Kd * (joint[j].targetVelocity - joint[j].actualVelocity);
        //cout << "Target-Actual(deg) " << j + 1 << " : " << (joint[j].targetRadian - joint[j].actualRadian) * R2D << endl;
        //cout << "Target Torque(t+1) " << j + 1 << " : " << joint[j].targetTorque << " N·m" << endl;
        //cout << "Actual Torque(t)" << j + 1 << " : " << joint[j].actualTorque << " N·m" << endl;
    }

    // Update target torque in gazebo simulation
    Joint_1->SetForce(1, -1 * joint[J1].targetTorque);
    Joint_2->SetForce(1, -1 * joint[J2].targetTorque);
    Joint_3->SetForce(1, joint[J3].targetTorque);
    Joint_4->SetForce(1, joint[J4].targetTorque);
    Joint_5->SetForce(1, -1 * joint[J5].targetTorque);
    Joint_6->SetForce(1, joint[J6].targetTorque);

    //--------------------------------------------------//
    // Publish ROS messages
    //--------------------------------------------------//
    /*
    Joint_1_msg.joint[J1] = joint[J1].targetTorque;
    Joint_2_msg.joint[J2] = joint[J2].targetTorque;
    Joint_3_msg.joint[J3] = joint[J3].targetTorque;
    Joint_4_msg.joint[J4] = joint[J4].targetTorque;
    Joint_5_msg.joint[J5] = joint[J5].targetTorque;
    Joint_6_msg.joint[J6] = joint[J6].targetTorque;

    J1_pub.publish(Joint_1_msg);
    J2_pub.publish(Joint_2_msg);
    J3_pub.publish(Joint_3_msg);
    J4_pub.publish(Joint_4_msg);
    J5_pub.publish(Joint_5_msg);
    J6_pub.publish(Joint_6_msg);
    */
}

void gazebo::drok4_plugin::GetJoints()
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */
    //Joint specified in solid_arm_3.sdf
    Joint_1 = this->model->GetJoint("joint_1");
    Joint_2 = this->model->GetJoint("joint_2");
    Joint_3 = this->model->GetJoint("joint_3");
    Joint_4 = this->model->GetJoint("joint_4");
    Joint_5 = this->model->GetJoint("joint_5");
    Joint_6 = this->model->GetJoint("joint_6");
}

void gazebo::drok4_plugin::GetjointData()
{
    /*
     * Get encoder and velocity data of each joint
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */

    joint[J1].actualRadian = -1 * Joint_1->Position(0);
    joint[J2].actualRadian = -1 * Joint_2->Position(0);
    joint[J3].actualRadian = Joint_3->Position(0);
    joint[J4].actualRadian = Joint_4->Position(0);
    joint[J5].actualRadian = -1 * Joint_5->Position(0);
    joint[J6].actualRadian = Joint_6->Position(0);

    for (int j = 0; j < nDoF; j++) {
        joint[j].actualDegree = joint[j].actualRadian * R2D;
        //std::cout << "D_value : " << j + 1 << " : : " << joint[j].actualDegree << std::endl;
    }

    joint[J1].actualVelocity = -1 * Joint_1->GetVelocity(1);
    joint[J2].actualVelocity = -1 * Joint_2->GetVelocity(1);
    joint[J3].actualVelocity = Joint_3->GetVelocity(1);
    joint[J4].actualVelocity = Joint_4->GetVelocity(1);
    joint[J5].actualVelocity = -1 * Joint_5->GetVelocity(1);
    joint[J6].actualVelocity = Joint_6->GetVelocity(1);

    /*
    std::cout << "V_value_1 : " << joint[J1].actualVelocity << std::endl;
    std::cout << "V_value_2 : " << joint[J2].actualVelocity << std::endl;
    std::cout << "V_value_3 : " << joint[J3].actualVelocity << std::endl;
    std::cout << "V_value_4 : " << joint[J4].actualVelocity << std::endl;
    std::cout << "V_value_5 : " << joint[J5].actualVelocity << std::endl;
    std::cout << "V_value_6 : " << joint[J6].actualVelocity << std::endl;
    */

    //    for (int j = 0; j < nDoF; j++) {
    //        cout << "joint[" << j <<"]="<<joint[j].actualDegree<< endl;
    //    }

}

void gazebo::drok4_plugin::initializeJoint()
{
    /*
     * Initialize joint variables for joint control
     */
    
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetDegree = 0;
        joint[j].targetRadian = 0;
        joint[j].targetVelocity = 0;
        joint[j].targetTorque = 0;
        
        joint[j].actualDegree = 0;
        joint[j].actualRadian = 0;
        joint[j].actualVelocity = 0;
        joint[j].actualRPM = 0;
        joint[j].actualTorque = 0;
    }
}

void gazebo::drok4_plugin::SetJointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */

    /*
    joint[J1].Kp = 15;
    joint[J2].Kp = 125;
    joint[J3].Kp = 135;
    joint[J4].Kp = 25;
    joint[J5].Kp = 25;
    joint[J6].Kp = 10;

    joint[J1].Kd = 0.065;
    joint[J2].Kd = 0.120;
    joint[J3].Kd = 0.130;
    joint[J4].Kd = 0.010;
    joint[J5].Kd = 0.010;
    joint[J6].Kd = 0.005;
    */


    joint[J1].Kp = 1500;
    joint[J2].Kp = 15000;//300;//200;//125;     //가장 부하가 많이 걸리는 관절
    joint[J3].Kp = 10000;//300;//200;//155;    //가장 부하가 많이 걸리는 관절 2
    joint[J4].Kp = 250;//50;//25;//15;
    joint[J5].Kp = 300;//150;//100;//25;//15;
    joint[J6].Kp = 100;//10;

    joint[J1].Kd = 0.065;
    joint[J2].Kd = 80;//0.120;//0.300;//0.200;//0.130;
    joint[J3].Kd = 30;//0.135;//0.300;//0.200;//0.165;
    joint[J4].Kd = 0.020;//0.020;
    joint[J5].Kd = 0.300;//0.100;//0.010;
    joint[J6].Kd = 0.025;//0.005;

}

