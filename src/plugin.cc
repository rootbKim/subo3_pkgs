#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <functional>
#include <ignition/math/Vector3.hh>
#include <Eigen/Dense>
#include <fstream>
#include <rbdl/rbdl.h>
#include <subo3_pkgs/CTCMsg.h>

using namespace std;
using namespace Eigen;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef struct EndPoint
{
  Eigen::MatrixXd T_matrix;
} ENDPOINT;

typedef struct Joint_torque
{
  double torque;
} JOINT;

// ************* 단위변환 ************************//
#define PI 3.14159265358979
#define M2R 2*PI/4096
#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323

#define inner_dt 0.004
// ************* 12DOF IK ***********************//
MatrixXd J_12DOF_T(12,12);
MatrixXd J_12DOF(12,12);
int arr_flag = 0;
// ***************행렬 선언*********************//
double Theo_RL_th[6] = {0.,0.,0.,0.,0.,0.}, Theo_LL_th[6] = {0.,0.,0.,0.,0.,0.};
double Act_RL_th[6] = {0.,0.,0.,0.,0.,0.}, Act_LL_th[6] = {0.,0.,0.,0.,0.,0.};
//*************** Trajectroy Variables**************//    
double step_time = 0;
double cnt_time = 0;
unsigned int cnt = 0;
double chg_step_time = 0;
double chg_cnt_time = 0;
unsigned int chg_cnt = 0;
bool home_firstRun = true;
unsigned int Up_Down_iteration_cnt=0;

//***************LOW PASS FILTER**************//    
double LPF_1st(double in, double* preOut, double freq_cut, double dt);

//*************** 확인용 변수*********************//
unsigned int f_cnt = 0; //주기 확인용 
unsigned int c_cnt = 0; // 시행횟수

//*************** RBDL variable ***************//
typedef struct Air_rbdl_model_
{
  Model* rbdl_model = new Model();
  VectorNd Q, QDot, QDDot, prevQ, prevQDot, Tau, Foot_Pos, Foot_Pos_dot, Des_X, Des_XDot, Des_XDDot, torque_CTC, Old_Des_X, Old_Des_XDot, Old_Des_XDDot, New_Des_X, New_Des_XDot, New_Des_XDDot, Kp, Kv;
  MatrixNd A_Jacobian, prev_A_Jacobian, A_Jacobian_dot, Inv_A_Jacobian;

  unsigned int w_roll_id, w_pitch_id, body_Base_id, body_PELVIS_1_id, body_PELVIS_2_id, body_THIGH_id, body_SHANK_id, body_FOOT_id;//id have information of the body
  Body w_roll, w_pitch, body_Base, body_PELVIS_1, body_PELVIS_2, body_THIGH, body_SHANK, body_FOOT;//make body.
  Joint joint_w_roll, joint_w_pitch, joint_Base, joint_PELVIS_YAW, joint_PELVIS_ROLL, joint_PELVIS_PITCH, joint_KNEE, joint_ANKLE;//make joint
  Math::Matrix3d w_rollI, w_pitchI, bodyI_Base, bodyI_PELVIS_1, bodyI_PELVIS_2, bodyI_THIGH, bodyI_SHANK, bodyI_FOOT;//Inertia of Body
} A_RBDL;

typedef struct Ground_rbdl_model_
{
  int num_leg = 0;
  Model* rbdl_model = new Model();
  VectorNd Q, QDot, QDDot, prevQ, prevQDot, Tau, Foot_Pos, Foot_Pos_dot, Des_X, Des_XDot, Des_XDDot, torque_CTC, Old_Des_X, Old_Des_XDot, Old_Des_XDDot, New_Des_X, New_Des_XDot, New_Des_XDDot, Kp, Kv;
  MatrixNd A_Jacobian, prev_A_Jacobian, A_Jacobian_dot, Inv_A_Jacobian;

  unsigned int body_FOOT_id, body_SHANK_id, body_THIGH_id, body_PELVIS_1_id, body_PELVIS_2_id, body_id;//id have information of the body
  Body body_FOOT, body_SHANK, body_THIGH, body_PELVIS_1, body_PELVIS_2, body;//make body.
  Joint joint_ANKLE_ROLL, joint_ANKLE_PITCH, joint_KNEE_PITCH, joint_PELVIS_PITCH, joint_PELVIS_ROLL, joint_PELVIS_YAW;//make joint
  Math::Matrix3d bodyI_FOOT, bodyI_SHANK, bodyI_THIGH, bodyI_PELVIS_1, bodyI_PELVIS_2, bodyI;//Inertia of Body
} G_RBDL;

A_RBDL A_L, A_R;
G_RBDL G_L, G_R, O_L, O_R;

ENDPOINT Left_LEG, Right_LEG;

int plot_cnt = 0, plot_cnt2 = 0;

namespace gazebo
{
  class SUBO3_plugin : public ModelPlugin
  {
    // ************* Model variables ****************//
    physics::ModelPtr model;

    physics::LinkPtr UPPER_BODY_LINK;
    physics::LinkPtr PELVIS_LINK;
    physics::LinkPtr BODY_IMU_LINK;
    physics::LinkPtr L_PELVIS_YAW_LINK;
    physics::LinkPtr L_PELVIS_ROLL_LINK;
    physics::LinkPtr L_PELVIS_PITCH_LINK;
    physics::LinkPtr L_KNEE_PITCH_LINK;
    physics::LinkPtr L_ANKLE_PITCH_LINK;
    physics::LinkPtr L_ANKLE_ROLL_LINK;
    physics::LinkPtr L_TORQUE_SENSOR_LINK;
    physics::LinkPtr L_FOOTPAD_LINK;
    physics::LinkPtr L_LEG_IMU_LINK;
    physics::LinkPtr R_PELVIS_YAW_LINK;
    physics::LinkPtr R_PELVIS_ROLL_LINK;
    physics::LinkPtr R_PELVIS_PITCH_LINK;
    physics::LinkPtr R_KNEE_PITCH_LINK;
    physics::LinkPtr R_ANKLE_PITCH_LINK;
    physics::LinkPtr R_ANKLE_ROLL_LINK;
    physics::LinkPtr R_TORQUE_SENSOR_LINK;
    physics::LinkPtr R_FOOTPAD_LINK;
    physics::LinkPtr R_LEG_IMU_LINK;

    physics::JointPtr PELVIS_JOINT;
    physics::JointPtr BODY_IMU_JOINT;
    physics::JointPtr L_PELVIS_YAW_JOINT;
    physics::JointPtr L_PELVIS_ROLL_JOINT;
    physics::JointPtr L_PELVIS_PITCH_JOINT;
    physics::JointPtr L_KNEE_PITCH_JOINT;
    physics::JointPtr L_ANKLE_PITCH_JOINT;
    physics::JointPtr L_ANKLE_ROLL_JOINT;
    physics::JointPtr L_TORQUE_SENSOR_JOINT;
    physics::JointPtr L_FOOTPAD_JOINT;
    physics::JointPtr L_LEG_IMU_JOINT;
    physics::JointPtr R_PELVIS_YAW_JOINT;
    physics::JointPtr R_PELVIS_ROLL_JOINT;
    physics::JointPtr R_PELVIS_PITCH_JOINT;
    physics::JointPtr R_KNEE_PITCH_JOINT;
    physics::JointPtr R_ANKLE_PITCH_JOINT;
    physics::JointPtr R_ANKLE_ROLL_JOINT;
    physics::JointPtr R_TORQUE_SENSOR_JOINT;
    physics::JointPtr R_FOOTPAD_JOINT;
    physics::JointPtr R_LEG_IMU_JOINT;

    VectorXd target_tor = VectorXd::Zero(12);

    // ************* Joint space variables ****************// 기본이 열벡터임에 주의할것!
    VectorXd pre_target_joint_pos = VectorXd::Zero(12);
    VectorXd target_joint_pos = VectorXd::Zero(12);  // IK pos
    VectorXd target_joint_vel = VectorXd::Zero(12);
    VectorXd target_joint_acc = VectorXd::Zero(12);
    VectorXd actual_joint_pos = VectorXd::Zero(12);
    VectorXd pre_actual_joint_pos = VectorXd::Zero(12);
    VectorXd actual_joint_vel = VectorXd::Zero(12);
    VectorXd actual_joint_acc = VectorXd::Zero(12);
    VectorXd joint_pos_err = VectorXd::Zero(12);
    VectorXd joint_vel_err = VectorXd::Zero(12);
    VectorXd Kp_q = VectorXd::Zero(12);
    VectorXd Kd_q = VectorXd::Zero(12);
    VectorXd cancle_delay = VectorXd::Zero(12);
    VectorXd error = VectorXd::Zero(12);
    VectorXd goal_joint_pos = VectorXd::Zero(12); // FK pos

    // ************* Cartesian space variables ****************//
    VectorXd target_EP_pos = VectorXd::Zero(12);
    VectorXd target_EP_vel = VectorXd::Zero(12);
    VectorXd target_EP_acc = VectorXd::Zero(12);
    VectorXd actual_EP_pos = VectorXd::Zero(12);
    VectorXd actual_EP_vel = VectorXd::Zero(12);
    VectorXd actual_EP_acc = VectorXd::Zero(12);
    VectorXd init_EP_pos = VectorXd::Zero(12);
    VectorXd goal_EP_pos = VectorXd::Zero(12);
    VectorXd EP_pos_err = VectorXd::Zero(12);

    // *************Time variables ****************//
    common::Time last_update_time;
    event::ConnectionPtr update_connection;
    double dt;
    double time = 0;
    common::Time current_time;

    // *************IMU sensor variables ****************//
    math::Pose base_info;
    sensors::SensorPtr Sensor;
    sensors::ImuSensorPtr BODY_IMU;

    sensors::ImuSensorPtr L_IMU;

    sensors::ImuSensorPtr R_IMU;

    VectorXd BODY_ImuGyro = VectorXd::Zero(3);
    VectorXd BODY_ImuAcc = VectorXd::Zero(3);
    VectorXd accel = VectorXd::Zero(3);
    VectorXd gyro_rate = VectorXd::Zero(3);
    VectorXd imu_rate = VectorXd::Zero(3);
    VectorXd preLPF_BODY_ImuGyro = VectorXd::Zero(3);
    VectorXd LPF_BODY_ImuGyro = VectorXd::Zero(3);
    VectorXd body_EAngle = VectorXd::Zero(3);
    VectorXd LPF_BODY_ImuAcc = VectorXd::Zero(3);
    VectorXd preLPF_BODY_ImuAcc = VectorXd::Zero(3);

    VectorXd L_ImuGyro = VectorXd::Zero(3);
    VectorXd L_ImuAcc = VectorXd::Zero(3);

    VectorXd R_ImuGyro = VectorXd::Zero(3);
    VectorXd R_ImuAcc = VectorXd::Zero(3);
    // *************FT sensor variables ****************//
    physics::JointWrench wrench;
    ignition::math::Vector3d torque;
    ignition::math::Vector3d force;

    VectorXd L_Force_E = VectorXd::Zero(3);
    VectorXd R_Force_E = VectorXd::Zero(3);

    VectorXd L_Torque_E = VectorXd::Zero(3);
    VectorXd R_Torque_E = VectorXd::Zero(3);

    VectorXd L_Force_I = VectorXd::Zero(3);
    VectorXd R_Force_I = VectorXd::Zero(3);

    // ************* ROS Communication ****************//
    ros::NodeHandle n;
	  // ************* publisher ************************//
    ros::Publisher P_Times;
    ros::Publisher P_ros_msg;
    
    ros::Publisher P_actual_L_Pelvis_Y_J;
    ros::Publisher P_actual_L_Pelvis_R_J;
    ros::Publisher P_actual_L_Pelvis_P_J;
    ros::Publisher P_actual_L_Knee_P_J;
    ros::Publisher P_actual_L_Ankle_P_J;
    ros::Publisher P_actual_L_Ankle_R_J;
    ros::Publisher P_actual_R_Pelvis_Y_J;
    ros::Publisher P_actual_R_Pelvis_R_J;
    ros::Publisher P_actual_R_Pelvis_P_J;
    ros::Publisher P_actual_R_Knee_P_J;
    ros::Publisher P_actual_R_Ankle_P_J;
    ros::Publisher P_actual_R_Ankle_R_J;
    
    ros::Publisher P_goal_L_Pelvis_Y_J;
    ros::Publisher P_goal_L_Pelvis_R_J;
    ros::Publisher P_goal_L_Pelvis_P_J;
    ros::Publisher P_goal_L_Knee_P_J;
    ros::Publisher P_goal_L_Ankle_P_J;
    ros::Publisher P_goal_L_Ankle_R_J;
    ros::Publisher P_goal_R_Pelvis_Y_J;
    ros::Publisher P_goal_R_Pelvis_R_J;
    ros::Publisher P_goal_R_Pelvis_P_J;
    ros::Publisher P_goal_R_Knee_P_J;
    ros::Publisher P_goal_R_Ankle_P_J;
    ros::Publisher P_goal_R_Ankle_R_J;
    // ************ msg ***************** //
    std_msgs::Float64 m_Times;
    
    std_msgs::Float64MultiArray m_ros_msg;
    
    std_msgs::Float64 m_actual_L_Pelvis_Y_J;
    std_msgs::Float64 m_actual_L_Pelvis_R_J;
    std_msgs::Float64 m_actual_L_Pelvis_P_J;
    std_msgs::Float64 m_actual_L_Knee_P_J;
    std_msgs::Float64 m_actual_L_Ankle_P_J;
    std_msgs::Float64 m_actual_L_Ankle_R_J;
    std_msgs::Float64 m_actual_R_Pelvis_Y_J;
    std_msgs::Float64 m_actual_R_Pelvis_R_J;
    std_msgs::Float64 m_actual_R_Pelvis_P_J;
    std_msgs::Float64 m_actual_R_Knee_P_J;
    std_msgs::Float64 m_actual_R_Ankle_P_J;
    std_msgs::Float64 m_actual_R_Ankle_R_J;
    
    std_msgs::Float64 m_goal_L_Pelvis_Y_J;
    std_msgs::Float64 m_goal_L_Pelvis_R_J;
    std_msgs::Float64 m_goal_L_Pelvis_P_J;
    std_msgs::Float64 m_goal_L_Knee_P_J;
    std_msgs::Float64 m_goal_L_Ankle_P_J;
    std_msgs::Float64 m_goal_L_Ankle_R_J;
    std_msgs::Float64 m_goal_R_Pelvis_Y_J;
    std_msgs::Float64 m_goal_R_Pelvis_R_J;
    std_msgs::Float64 m_goal_R_Pelvis_P_J;
    std_msgs::Float64 m_goal_R_Knee_P_J;
    std_msgs::Float64 m_goal_R_Ankle_P_J;
    std_msgs::Float64 m_goal_R_Ankle_R_J;
	
    VectorXd TmpData = VectorXd::Zero(50);
    ros::Subscriber server_sub1;
    ros::Subscriber server_sub2;
    ros::Subscriber server_sub3;
    ros::Subscriber server_sub4;

    int start_flag = 0;

    // ************* Structure variables ****************//
    ENDPOINT L_LEG, R_LEG;
    JOINT* joint;
    JOINT* old_joint;
    JOINT* prev_out_joint;
    JOINT* out_joint;

    enum ControlMode
    {
      IDLE = 0,
      GRAVITY_CONTROL,
      CTC_CONTROL,
      CTC_CONTROL_POS,
      CTC_CONTROL_CONT_POS,
      GROUND_GRAVITY_CONTROL,
      GROUND_CTC_CONTROL,
      GROUND_CTC_CONTROL_POS,
      GROUND_CTC_CONTROL_CONT_POS,
      ONE_GROUND_CTC_CONTROL,
    };
    
    enum ControlMode CONTROL_MODE;

    // ************* Functions ****************//
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
    void UpdateAlgorithm();
    void RBDL_INIT();
    void rbdl_variable_init(G_RBDL &rbdl);
    void rbdl_variable_init(A_RBDL &rbdl);
    void L_Ground_Model(G_RBDL &rbdl);
    void R_Ground_Model(G_RBDL &rbdl);
    void L_Air_Model(A_RBDL &rbdl);
    void R_Air_Model(A_RBDL &rbdl);
    void GetLinks();
    void GetJoints();
    void InitROSPubSetting();
    void SensorSetting();
    void IMUSensorRead();
    void FTSensorRead();
    void EncoderRead();
    void torque_interpolation();
    void jointController();
    void ROSMsgPublish();
    void Callback1(const std_msgs::Int32Ptr &msg);
    void msgCallback(const subo3_pkgs::CTCMsg::ConstPtr &msg);
    void msgCallback2(const subo3_pkgs::CTCMsg::ConstPtr &msg);
    void msgCallback3(const subo3_pkgs::CTCMsg::ConstPtr &msg);

    void PostureGeneration();

    void RBDL_variable_update();
    void Calc_Feedback_Pos(A_RBDL &rbdl);
    void Calc_CTC_Torque(A_RBDL &rbdl);
    void Calc_Feedback_Pos(G_RBDL &rbdl);
    void Calc_CTC_Torque(G_RBDL &rbdl);
    void CalcBodyAngle();
    void Init_Pos_Traj();
    void Gravity_Cont();
    void CTC_Control();
    void CTC_Control_Pos();
    void CTC_Control_Cont_Pos();
    void GROUND_Gravity_Cont();
    void GROUND_CTC_Control();
    void GROUND_CTC_Control_Pos();
    void GROUND_CTC_Control_Cont_Pos();
    void ONE_GROUND_CTC_Control();

    VectorXd FK(VectorXd joint_pos_HS);
    VectorXd IK(VectorXd EP_pos);

    FILE* tmpdata0=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata0.txt","w");
    FILE* tmpdata1=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata1.txt","w");
    FILE* tmpdata2=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata2.txt","w");
    FILE* tmpdata3=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata3.txt","w");
    FILE* tmpdata4=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata4.txt","w");
    FILE* tmpdata5=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata5.txt","w");
    FILE* tmpdata6=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata6.txt","w");
    FILE* tmpdata7=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata7.txt","w");

    void Print(void); //Print function
  };
  GZ_REGISTER_MODEL_PLUGIN(SUBO3_plugin); //model plugin 등록함수
}

void gazebo::SUBO3_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) //처음키면 한번 실행되는 함수
{
  RBDL_INIT();

  this->model = _model;
  GetLinks();
  GetJoints();
  InitROSPubSetting();
  SensorSetting();
  
  joint = new JOINT[12];
  old_joint = new JOINT[12];

  prev_out_joint = new JOINT[12];
  out_joint = new JOINT[12];

  for(int i = 0; i < 12; i++)
  {
    joint[i].torque = 0;
    old_joint[i].torque = 0;
    prev_out_joint[i].torque = 0;
    out_joint[i].torque = 0;
  }

  CONTROL_MODE = IDLE;

  this->last_update_time = this->model->GetWorld()->GetSimTime();
  this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SUBO3_plugin::UpdateAlgorithm, this));
  std::cout << "Load..." << std::endl;
}

void gazebo::SUBO3_plugin::RBDL_INIT()
{
  //********************RBDL********************//
  rbdl_check_api_version(RBDL_API_VERSION);//check the rdbl version

  //********************MODEL********************//
  // Air model
  A_L.rbdl_model = new Model();//declare Model
  A_L.rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);//set gravity
  A_R.rbdl_model = new Model();//declare Model
  A_R.rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);//set gravity
  // Two Leg ground model
  G_L.rbdl_model = new Model();//declare Model
  G_L.rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);//set gravity
  G_R.rbdl_model = new Model();//declare Model
  G_R.rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);//set gravity
  // One Leg ground model
  O_L.rbdl_model = new Model();//declare Model
  O_L.rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);//set gravity
  O_R.rbdl_model = new Model();//declare Model
  O_R.rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);//set gravity

  G_L.num_leg = 0;
  G_R.num_leg = 0;
  O_L.num_leg = 1;
  O_R.num_leg = 1;

  L_Air_Model(A_L);
  R_Air_Model(A_R);

  L_Ground_Model(G_L);
  R_Ground_Model(G_R);
  L_Ground_Model(O_L);
  R_Ground_Model(O_R);

  rbdl_variable_init(A_L);
  rbdl_variable_init(A_R);
  rbdl_variable_init(G_L);
  rbdl_variable_init(G_R);
  rbdl_variable_init(O_L);
  rbdl_variable_init(O_R);
}

void gazebo::SUBO3_plugin::rbdl_variable_init(A_RBDL &rbdl)
{
  //set Q, QDot, QDDot, prevQ, prevQDot // dof_count = num of degree
	rbdl.Q = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.QDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.QDDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.prevQ = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.prevQDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
  rbdl.Tau = VectorNd::Zero(rbdl.rbdl_model->dof_count);
  rbdl.Foot_Pos = VectorNd::Zero(6);
  rbdl.Foot_Pos_dot = VectorNd::Zero(6);
  rbdl.A_Jacobian = MatrixNd::Zero(6,6);
  rbdl.prev_A_Jacobian = MatrixNd::Zero(6,6);
  rbdl.A_Jacobian_dot = MatrixNd::Zero(6,6);
  rbdl.Inv_A_Jacobian = MatrixNd::Zero(6,6);
  rbdl.Des_X = VectorNd::Zero(6);
  rbdl.Des_XDot = VectorNd::Zero(6);
  rbdl.Des_XDDot = VectorNd::Zero(6);
  rbdl.torque_CTC = VectorNd::Zero(6);
  rbdl.Old_Des_X = VectorNd::Zero(6);
  rbdl.Old_Des_XDot = VectorNd::Zero(6);
  rbdl.Old_Des_XDDot = VectorNd::Zero(6);
  rbdl.New_Des_X = VectorNd::Zero(6);
  rbdl.New_Des_XDot = VectorNd::Zero(6);
  rbdl.New_Des_XDDot = VectorNd::Zero(6);

  rbdl.Kp = VectorNd::Zero(6);
  rbdl.Kv = VectorNd::Zero(6);
}

void gazebo::SUBO3_plugin::rbdl_variable_init(G_RBDL &rbdl)
{
  //set Q, QDot, QDDot, prevQ, prevQDot // dof_count = num of degree
	rbdl.Q = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.QDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.QDDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.prevQ = VectorNd::Zero(rbdl.rbdl_model->dof_count);
	rbdl.prevQDot = VectorNd::Zero(rbdl.rbdl_model->dof_count);
  rbdl.Tau = VectorNd::Zero(rbdl.rbdl_model->dof_count);
  rbdl.Foot_Pos = VectorNd::Zero(6);
  rbdl.Foot_Pos_dot = VectorNd::Zero(6);
  rbdl.A_Jacobian = MatrixNd::Zero(6,6);
  rbdl.prev_A_Jacobian = MatrixNd::Zero(6,6);
  rbdl.A_Jacobian_dot = MatrixNd::Zero(6,6);
  rbdl.Inv_A_Jacobian = MatrixNd::Zero(6,6);
  rbdl.Des_X = VectorNd::Zero(6);
  rbdl.Des_XDot = VectorNd::Zero(6);
  rbdl.Des_XDDot = VectorNd::Zero(6);
  rbdl.torque_CTC = VectorNd::Zero(6);
  rbdl.Old_Des_X = VectorNd::Zero(6);
  rbdl.Old_Des_XDot = VectorNd::Zero(6);
  rbdl.Old_Des_XDDot = VectorNd::Zero(6);
  rbdl.New_Des_X = VectorNd::Zero(6);
  rbdl.New_Des_XDot = VectorNd::Zero(6);
  rbdl.New_Des_XDDot = VectorNd::Zero(6);

  rbdl.Kp = VectorNd::Zero(6);
  rbdl.Kv = VectorNd::Zero(6);
}

void gazebo::SUBO3_plugin::L_Air_Model(A_RBDL &rbdl)
{
  rbdl.w_rollI = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
  rbdl.w_roll = Body(0, Math::Vector3d(0, 0, 0), rbdl.w_rollI);
  rbdl.joint_w_roll = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
  rbdl.w_roll_id = rbdl.rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_w_roll, rbdl.w_roll);

  rbdl.w_pitchI = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
  rbdl.w_pitch = Body(0, Math::Vector3d(0, 0, 0), rbdl.w_pitchI);
  rbdl.joint_w_pitch = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
  rbdl.w_pitch_id = rbdl.rbdl_model->Model::AddBody(rbdl.w_roll_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_w_pitch, rbdl.w_pitch);

  rbdl.bodyI_Base = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
	rbdl.body_Base = Body(0, Math::Vector3d(0, 0, 0), rbdl.bodyI_Base);
	rbdl.joint_Base = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 0));
	rbdl.body_Base_id = rbdl.rbdl_model->Model::AddBody(rbdl.w_pitch_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_Base, rbdl.body_Base);

  //********************LEFT_LEG********************//
  //Inertia of Body
  rbdl.bodyI_PELVIS_1 = Math::Matrix3d(0.000419,-0.000005,0.000133, -0.000005,0.001001,-0.000018, 0.000133,-0.000018,0.000820);
  rbdl.bodyI_PELVIS_2 = Math::Matrix3d(0.000262,-0.000050,0.000001, -0.000050,0.000727,-0.000002, 0.000001,-0.000002,0.000757);
  rbdl.bodyI_THIGH = Math::Matrix3d(0.015886,-0.000021,-0.000012, -0.000021,0.014439,-0.000761, -0.000012,-0.000761,0.002793);
  rbdl.bodyI_SHANK = Math::Matrix3d(0.018454,0.000148,0.001029, 0.000148,0.017892,-0.000337, 0.001029,-0.000337,0.003522);
  rbdl.bodyI_FOOT = Math::Matrix3d(0.001594,0.000050,-0.000455, 0.000050,0.003143,-0.000103, -0.000455,-0.000103,0.002851);

  //set_body_PELVIS_1
	rbdl.body_PELVIS_1 = Body(0.530103, Math::Vector3d(-0.026622, -0.006227, -0.028069), rbdl.bodyI_PELVIS_1);
	rbdl.joint_PELVIS_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1)); // pelvis yaw
	rbdl.body_PELVIS_1_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_Base_id, Xtrans(Math::Vector3d(0, 0.07, 0)), rbdl.joint_PELVIS_YAW, rbdl.body_PELVIS_1);
	
  //set_body_PELVIS2
	rbdl.body_PELVIS_2 = Body(0.565726, Math::Vector3d(-0.029274, -0.002813, -0.000043), rbdl.bodyI_PELVIS_2);
	rbdl.joint_PELVIS_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0)); // pelvis roll
	rbdl.body_PELVIS_2_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_PELVIS_1_id, Xtrans(Math::Vector3d(-0.00025, 0, -0.074)), rbdl.joint_PELVIS_ROLL, rbdl.body_PELVIS_2);

  //set_body_THIGH
	rbdl.body_THIGH = Body(1.897, Math::Vector3d(0.003474, 0.02006, -0.16412), rbdl.bodyI_THIGH);
	rbdl.joint_PELVIS_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0)); // pelvis pitch
	rbdl.body_THIGH_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_PELVIS_2_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_PELVIS_PITCH, rbdl.body_THIGH);
  
	//set_body_SHANK
  rbdl.body_SHANK = Body(2.2, Math::Vector3d(-0.007394, 0.00838, -0.18208), rbdl.bodyI_SHANK);
	rbdl.joint_KNEE = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
  rbdl.body_SHANK_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_THIGH_id, Xtrans(Math::Vector3d(-0.00013, 0, -0.25)), rbdl.joint_KNEE, rbdl.body_SHANK);

	//set_body_FOOT
  rbdl.body_FOOT = Body(0.854, Math::Vector3d(0.012113, 0.004647, -0.068225), rbdl.bodyI_FOOT);
	rbdl.joint_ANKLE = Joint(SpatialVector(0, 1, 0, 0, 0, 0), SpatialVector(1, 0, 0, 0, 0, 0));
	rbdl.body_FOOT_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_SHANK_id, Xtrans(Math::Vector3d(0, 0, -0.25)), rbdl.joint_ANKLE, rbdl.body_FOOT);
}

void gazebo::SUBO3_plugin::R_Air_Model(A_RBDL &rbdl)
{
  rbdl.w_rollI = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
  rbdl.w_roll = Body(0, Math::Vector3d(0, 0, 0), rbdl.w_rollI);
  rbdl.joint_w_roll = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
  rbdl.w_roll_id = rbdl.rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_w_roll, rbdl.w_roll);

  rbdl.w_pitchI = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
  rbdl.w_pitch = Body(0, Math::Vector3d(0, 0, 0), rbdl.w_pitchI);
  rbdl.joint_w_pitch = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
  rbdl.w_pitch_id = rbdl.rbdl_model->Model::AddBody(rbdl.w_roll_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_w_pitch, rbdl.w_pitch);

  rbdl.bodyI_Base = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
	rbdl.body_Base = Body(0, Math::Vector3d(0, 0, 0), rbdl.bodyI_Base);
	rbdl.joint_Base = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 0));
	rbdl.body_Base_id = rbdl.rbdl_model->Model::AddBody(rbdl.w_pitch_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_Base, rbdl.body_Base);

  //********************RIGHT_LEG********************//
  //Inertia of Body
  rbdl.bodyI_PELVIS_1 = Math::Matrix3d(0.000419,0.000005,0.000133, 0.000005,0.001001,0.000018, 0.000133,0.000018,0.000820);
  rbdl.bodyI_PELVIS_2 = Math::Matrix3d(0.000262,0.000050,0.000001, 0.000050,0.000727,0.000002, 0.000001,0.000002,0.000757);
  rbdl.bodyI_THIGH = Math::Matrix3d(0.015886,0.000021,-0.000012, 0.000021,0.014439,0.000761, -0.000012,0.000761,0.002793);
  rbdl.bodyI_SHANK = Math::Matrix3d(0.018454,-0.000148,0.001029, -0.000148,0.017892,0.000337, 0.001029,0.000337,0.003522);
  rbdl.bodyI_FOOT = Math::Matrix3d(0.001594,-0.000050,-0.000455, -0.000050,0.003143,0.000103, -0.000455,0.000103,0.002851);
  
  //set_body_PELVIS_1
	rbdl.body_PELVIS_1 = Body(0.530103, Math::Vector3d(-0.026622, 0.006227, -0.028069), rbdl.bodyI_PELVIS_1);
	rbdl.joint_PELVIS_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1)); // pelvis yaw
	rbdl.body_PELVIS_1_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_Base_id, Xtrans(Math::Vector3d(0, -0.07, 0)), rbdl.joint_PELVIS_YAW, rbdl.body_PELVIS_1);
	
  //set_body_PELVIS2
	rbdl.body_PELVIS_2 = Body(0.565726, Math::Vector3d(-0.029274, 0.002813, -0.000043), rbdl.bodyI_PELVIS_2);
	rbdl.joint_PELVIS_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0)); // pelvis roll
	rbdl.body_PELVIS_2_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_PELVIS_1_id, Xtrans(Math::Vector3d(-0.00025, 0, -0.074)), rbdl.joint_PELVIS_ROLL, rbdl.body_PELVIS_2);

	//set_body_THIGH
	rbdl.body_THIGH = Body(1.897, Math::Vector3d(0.00335, -0.02006, -0.16411), rbdl.bodyI_THIGH);
	rbdl.joint_PELVIS_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0)); //pelvis pitch
	rbdl.body_THIGH_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_PELVIS_2_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_PELVIS_PITCH, rbdl.body_THIGH);

	//set_body_SHANK
  rbdl.body_SHANK = Body(2.2, Math::Vector3d(-0.00742, -0.00838, -0.18203), rbdl.bodyI_SHANK);
	rbdl.joint_KNEE = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
  rbdl.body_SHANK_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_THIGH_id, Xtrans(Math::Vector3d(-0.00013, 0, -0.25)), rbdl.joint_KNEE, rbdl.body_SHANK);

	//set_body_FOOT
  rbdl.body_FOOT = Body(0.854, Math::Vector3d(0.01206, -0.00475, -0.068478), rbdl.bodyI_FOOT);
	rbdl.joint_ANKLE = Joint(SpatialVector(0, 1, 0, 0, 0, 0), SpatialVector(1, 0, 0, 0, 0, 0));
	rbdl.body_FOOT_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_SHANK_id, Xtrans(Math::Vector3d(0, 0, -0.25)), rbdl.joint_ANKLE, rbdl.body_FOOT);
}

void gazebo::SUBO3_plugin::L_Ground_Model(G_RBDL &rbdl)
{
  //Inertia of Body
  rbdl.bodyI_FOOT = Math::Matrix3d(0.000148,0,0, 0,0.000174,-0.000002, 0,-0.000002,0.000153);
  rbdl.bodyI_SHANK = Math::Matrix3d(0.007532,-0.000015,-0.000035, -0.000015,0.006612,0.000288, -0.000035,0.000228,0.001592);
  rbdl.bodyI_THIGH = Math::Matrix3d(0.0024298,-0.000040,0.000187, -0.000040,0.022793,-0.000114, 0.000187,-0.000114,0.003027);
  rbdl.bodyI_PELVIS_1 = Math::Matrix3d(0.000262,-0.000050,-0.000001, -0.000050,0.000727,0.000002, -0.000001,0.000002,0.000757);
  rbdl.bodyI_PELVIS_2 = Math::Matrix3d(0.000419,-0.000005,-0.000133, -0.000005,0.001001,0.000018, -0.000133,0.000018,0.000820);

	//set_body_FOOT
  rbdl.body_FOOT = Body(0.398, Math::Vector3d(0.00004, -0.037051, 0.000415), rbdl.bodyI_FOOT);
	rbdl.joint_ANKLE_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
  rbdl.body_FOOT_id = rbdl.rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_ANKLE_ROLL, rbdl.body_FOOT);

  //set_body_SHANK
  rbdl.body_SHANK = Body(0.960, Math::Vector3d(-0.000138, -0.010092, 0.142584), rbdl.bodyI_SHANK);
	rbdl.joint_ANKLE_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
  rbdl.body_SHANK_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_FOOT_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_ANKLE_PITCH, rbdl.body_SHANK);

  //set_body_THIGH
	rbdl.body_THIGH = Body(2.258, Math::Vector3d(-0.002914, -0.022867, 0.112161), rbdl.bodyI_THIGH);
	rbdl.joint_KNEE_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
	rbdl.body_THIGH_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_SHANK_id, Xtrans(Math::Vector3d(0, 0, 0.25)), rbdl.joint_KNEE_PITCH, rbdl.body_THIGH);

  //set_body_PELVIS_1
	rbdl.body_PELVIS_1 = Body(0.565726, Math::Vector3d(0.029270, 0.002813, -0.000043), rbdl.bodyI_PELVIS_1);
	rbdl.joint_PELVIS_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
	rbdl.body_PELVIS_1_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_THIGH_id, Xtrans(Math::Vector3d(0, 0, 0.25)), rbdl.joint_PELVIS_PITCH, rbdl.body_PELVIS_1);
	
  //set_body_PELVIS2
	rbdl.body_PELVIS_2 = Body(0.530103, Math::Vector3d(0.026369, 0.006227, 0.045931), rbdl.bodyI_PELVIS_2);
	rbdl.joint_PELVIS_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
	rbdl.body_PELVIS_2_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_PELVIS_1_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_PELVIS_ROLL, rbdl.body_PELVIS_2);

  //set_body
  if(rbdl.num_leg == 0) // G_L
  {
    rbdl.bodyI = Math::Matrix3d(0.033070,0.000598,0.001242, 0.000598,0.034412,-0.002446, 0.001242,-0.002446,0.013281);
    rbdl.body = Body(4.226168, Math::Vector3d(0.019955, 0.005266, 0.142614), rbdl.bodyI);
    rbdl.joint_PELVIS_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1));
    rbdl.body_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_PELVIS_2_id, Xtrans(Math::Vector3d(0, 0, 0.074)), rbdl.joint_PELVIS_YAW, rbdl.body);
  }
  else if(rbdl.num_leg == 1)  // O_L
  {
    rbdl.bodyI = Math::Matrix3d(1.292935,-0.005110,0.037323, -0.005110,1.238202,-0.143039, 0.037323,-0.143039,0.099136);
    rbdl.body = Body(14.835537, Math::Vector3d(0.013367, 0.104708, 0.06775), rbdl.bodyI);
    rbdl.joint_PELVIS_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1));
    rbdl.body_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_PELVIS_2_id, Xtrans(Math::Vector3d(0, 0, 0.074)), rbdl.joint_PELVIS_YAW, rbdl.body);
  }
}

void gazebo::SUBO3_plugin::R_Ground_Model(G_RBDL &rbdl)
{
  //Inertia of Body
  rbdl.bodyI_FOOT = Math::Matrix3d(0.000148,0,0, 0,0.000174,0.000002, 0,0.000002,0.000153);
  rbdl.bodyI_SHANK = Math::Matrix3d(0.007532,0.000015,-0.000035, 0.000015,0.006612,-0.000288, -0.000035,-0.000228,0.001592);
  rbdl.bodyI_THIGH = Math::Matrix3d(0.0024298,0.000040,0.000187, 0.000040,0.022793,0.000114, 0.000187,0.000114,0.003027);
  rbdl.bodyI_PELVIS_1 = Math::Matrix3d(0.000262,0.000050,-0.000001, 0.000050,0.000727,-0.000002, -0.000001,-0.000002,0.000757);
  rbdl.bodyI_PELVIS_2 = Math::Matrix3d(0.000419,0.000005,-0.000133, 0.000005,0.001001,-0.000018, -0.000133,-0.000018,0.000820);

	//set_body_FOOT
  rbdl.body_FOOT = Body(0.398, Math::Vector3d(0.00004, 0.037051, 0.000415), rbdl.bodyI_FOOT);
	rbdl.joint_ANKLE_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
  rbdl.body_FOOT_id = rbdl.rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_ANKLE_ROLL, rbdl.body_FOOT);

  //set_body_SHANK
  rbdl.body_SHANK = Body(0.960, Math::Vector3d(-0.000138, 0.010092, 0.142584), rbdl.bodyI_SHANK);
	rbdl.joint_ANKLE_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
  rbdl.body_SHANK_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_FOOT_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_ANKLE_PITCH, rbdl.body_SHANK);

  //set_body_THIGH
	rbdl.body_THIGH = Body(2.258, Math::Vector3d(-0.002914, 0.022867, 0.112161), rbdl.bodyI_THIGH);
	rbdl.joint_KNEE_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
	rbdl.body_THIGH_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_SHANK_id, Xtrans(Math::Vector3d(0, 0, 0.25)), rbdl.joint_KNEE_PITCH, rbdl.body_THIGH);

  //set_body_PELVIS_1
	rbdl.body_PELVIS_1 = Body(0.565726, Math::Vector3d(0.029270, -0.002813, -0.000043), rbdl.bodyI_PELVIS_1);
	rbdl.joint_PELVIS_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
	rbdl.body_PELVIS_1_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_THIGH_id, Xtrans(Math::Vector3d(0, 0, 0.25)), rbdl.joint_PELVIS_PITCH, rbdl.body_PELVIS_1);
	
  //set_body_PELVIS2
	rbdl.body_PELVIS_2 = Body(0.530103, Math::Vector3d(0.026369, -0.006227, 0.045931), rbdl.bodyI_PELVIS_2);
	rbdl.joint_PELVIS_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
	rbdl.body_PELVIS_2_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_PELVIS_1_id, Xtrans(Math::Vector3d(0, 0, 0)), rbdl.joint_PELVIS_ROLL, rbdl.body_PELVIS_2);

  //set_body
  if(rbdl.num_leg == 0) // G_R
  {
    rbdl.bodyI = Math::Matrix3d(0.033070,-0.000598,0.001242, -0.000598,0.034412,0.002446, 0.001242,0.002446,0.013281);
    rbdl.body = Body(4.226168, Math::Vector3d(0.019955, -0.005266, 0.142614), rbdl.bodyI);
    rbdl.joint_PELVIS_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1));
    rbdl.body_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_PELVIS_2_id, Xtrans(Math::Vector3d(0, 0, 0.074)), rbdl.joint_PELVIS_YAW, rbdl.body);
  }
  else if(rbdl.num_leg == 1)  // O_R
  {
    rbdl.bodyI = Math::Matrix3d(1.292935,0.005110,0.037323, 0.005110,1.238202,0.143039, 0.037323,0.143039,0.099136);
    rbdl.body = Body(14.835537, Math::Vector3d(0.013367, -0.104708, -0.06775), rbdl.bodyI);
    rbdl.joint_PELVIS_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1));
    rbdl.body_id = rbdl.rbdl_model->Model::AddBody(rbdl.body_PELVIS_2_id, Xtrans(Math::Vector3d(0, 0, 0.074)), rbdl.joint_PELVIS_YAW, rbdl.body);
  }
}

void gazebo::SUBO3_plugin::UpdateAlgorithm() // 여러번 실행되는 함수
{
  //************************** Time ********************************//
  current_time = this->model->GetWorld()->GetSimTime();
  dt = current_time.Double() - this->last_update_time.Double();

  f_cnt++;
  if(f_cnt >= 4)
  {
    for(int i = 0; i < 12; i++)
    {
      prev_out_joint[i].torque = joint[i].torque;
    }

    IMUSensorRead();
    FTSensorRead();
    EncoderRead(); //FK 푸는것도 포함.

    CalcBodyAngle();
    RBDL_variable_update();

    PostureGeneration(); // PostureGeneration 하위에 Trajectory 하위에 IK푸는것 포함.

    f_cnt = 0;
  }
  
  // torque_interpolation();

  jointController();

  ROSMsgPublish();
  Print();

  this->last_update_time = current_time;

}

void gazebo::SUBO3_plugin::GetLinks() 
{
  //LINK DEFINITION
  this->UPPER_BODY_LINK = this->model->GetLink("UPPER_BODY_LINK");
  this->PELVIS_LINK = this->model->GetLink("PELVIS_LINK");
  this->BODY_IMU_LINK = this->model->GetLink("BODY_IMU_LINK");

  this->L_PELVIS_YAW_LINK = this->model->GetLink("L_PELVIS_YAW_LINK");
  this->L_PELVIS_ROLL_LINK = this->model->GetLink("L_PELVIS_ROLL_LINK");
  this->L_PELVIS_PITCH_LINK = this->model->GetLink("L_PELVIS_PITCH_LINK");
  this->L_KNEE_PITCH_LINK = this->model->GetLink("L_KNEE_PITCH_LINK");
  this->L_ANKLE_PITCH_LINK = this->model->GetLink("L_ANKLE_PITCH_LINK");
  this->L_ANKLE_ROLL_LINK = this->model->GetLink("L_ANKLE_ROLL_LINK");
  this->L_TORQUE_SENSOR_LINK = this->model->GetLink("L_TORQUE_SENSOR_LINK");
  this->L_FOOTPAD_LINK = this->model->GetLink("L_FOOTPAD_LINK");
  this->L_LEG_IMU_LINK = this->model->GetLink("L_LEG_IMU_LINK");

  this->R_PELVIS_YAW_LINK = this->model->GetLink("R_PELVIS_YAW_LINK");
  this->R_PELVIS_ROLL_LINK = this->model->GetLink("R_PELVIS_ROLL_LINK");
  this->R_PELVIS_PITCH_LINK = this->model->GetLink("R_PELVIS_PITCH_LINK");
  this->R_KNEE_PITCH_LINK = this->model->GetLink("R_KNEE_PITCH_LINK");
  this->R_ANKLE_PITCH_LINK = this->model->GetLink("R_ANKLE_PITCH_LINK");
  this->R_ANKLE_ROLL_LINK = this->model->GetLink("R_ANKLE_ROLL_LINK");
  this->R_TORQUE_SENSOR_LINK = this->model->GetLink("R_TORQUE_SENSOR_LINK");
  this->R_FOOTPAD_LINK = this->model->GetLink("R_FOOTPAD_LINK");
  this->R_LEG_IMU_LINK = this->model->GetLink("R_LEG_IMU_LINK");
}

void gazebo::SUBO3_plugin::GetJoints()
{
  //JOINT DEFINITION
  this->PELVIS_JOINT = this->model->GetJoint("PELVIS_JOINT");
  this->BODY_IMU_JOINT = this->model->GetJoint("BODY_IMU_JOINT");

  this->L_PELVIS_YAW_JOINT = this->model->GetJoint("L_PELVIS_YAW_JOINT");
  this->L_PELVIS_ROLL_JOINT = this->model->GetJoint("L_PELVIS_ROLL_JOINT");
  this->L_PELVIS_PITCH_JOINT = this->model->GetJoint("L_PELVIS_PITCH_JOINT");
  this->L_KNEE_PITCH_JOINT = this->model->GetJoint("L_KNEE_PITCH_JOINT");
  this->L_ANKLE_PITCH_JOINT = this->model->GetJoint("L_ANKLE_PITCH_JOINT");
  this->L_ANKLE_ROLL_JOINT = this->model->GetJoint("L_ANKLE_ROLL_JOINT");
  this->L_TORQUE_SENSOR_JOINT = this->model->GetJoint("L_TORQUE_SENSOR_JOINT");
  this->L_FOOTPAD_JOINT = this->model->GetJoint("L_FOOTPAD_JOINT");
  this->L_LEG_IMU_JOINT = this->model->GetJoint("L_LEG_IMU_JOINT");

  this->R_PELVIS_YAW_JOINT = this->model->GetJoint("R_PELVIS_YAW_JOINT");
  this->R_PELVIS_ROLL_JOINT = this->model->GetJoint("R_PELVIS_ROLL_JOINT");
  this->R_PELVIS_PITCH_JOINT = this->model->GetJoint("R_PELVIS_PITCH_JOINT");
  this->R_KNEE_PITCH_JOINT = this->model->GetJoint("R_KNEE_PITCH_JOINT");
  this->R_ANKLE_PITCH_JOINT = this->model->GetJoint("R_ANKLE_PITCH_JOINT");
  this->R_ANKLE_ROLL_JOINT = this->model->GetJoint("R_ANKLE_ROLL_JOINT");
  this->R_TORQUE_SENSOR_JOINT = this->model->GetJoint("R_TORQUE_SENSOR_JOINT");
  this->R_FOOTPAD_JOINT = this->model->GetJoint("R_FOOTPAD_JOINT");
  this->R_LEG_IMU_JOINT = this->model->GetJoint("R_LEG_IMU_JOINT");
}

void gazebo::SUBO3_plugin::InitROSPubSetting()
{
  //************************ROS Msg Setting*********************************//
  P_Times = n.advertise<std_msgs::Float64>("times", 1);
  
  P_actual_R_Pelvis_Y_J = n.advertise<std_msgs::Float64>("actual_R_Pelvis_Y_J", 10);
  P_actual_R_Pelvis_R_J = n.advertise<std_msgs::Float64>("actual_R_Pelvis_R_J", 10);
  P_actual_R_Pelvis_P_J = n.advertise<std_msgs::Float64>("actual_R_Pelvis_P_J", 10);
  P_actual_R_Knee_P_J = n.advertise<std_msgs::Float64>("actual_R_Knee_P_J", 10);
  P_actual_R_Ankle_P_J = n.advertise<std_msgs::Float64>("actual_R_Ankle_P_J", 10);
  P_actual_R_Ankle_R_J = n.advertise<std_msgs::Float64>("actual_R_Ankle_R_J", 10);
  P_actual_L_Pelvis_Y_J = n.advertise<std_msgs::Float64>("actual_L_Pelvis_Y_J", 10);
  P_actual_L_Pelvis_R_J = n.advertise<std_msgs::Float64>("actual_L_Pelvis_R_J", 10);
  P_actual_L_Pelvis_P_J = n.advertise<std_msgs::Float64>("actual_L_Pelvis_P_J", 10);
  P_actual_L_Knee_P_J = n.advertise<std_msgs::Float64>("actual_L_Knee_P_J", 10);
  P_actual_L_Ankle_P_J = n.advertise<std_msgs::Float64>("actual_L_Ankle_P_J", 10);
  P_actual_L_Ankle_R_J = n.advertise<std_msgs::Float64>("actual_L_Ankle_R_J", 10);

  P_goal_R_Pelvis_Y_J = n.advertise<std_msgs::Float64>("goal_R_Pelvis_Y_J", 10);
  P_goal_R_Pelvis_R_J = n.advertise<std_msgs::Float64>("goal_R_Pelvis_R_J", 10);
  P_goal_R_Pelvis_P_J = n.advertise<std_msgs::Float64>("goal_R_Pelvis_P_J", 10);
  P_goal_R_Knee_P_J = n.advertise<std_msgs::Float64>("goal_R_Knee_P_J", 10);
  P_goal_R_Ankle_P_J = n.advertise<std_msgs::Float64>("goal_R_Ankle_P_J", 10);
  P_goal_R_Ankle_R_J = n.advertise<std_msgs::Float64>("goal_R_Ankle_R_J", 10);
  P_goal_L_Pelvis_Y_J = n.advertise<std_msgs::Float64>("goal_L_Pelvis_Y_J", 10);
  P_goal_L_Pelvis_R_J = n.advertise<std_msgs::Float64>("goal_L_Pelvis_R_J", 10);
  P_goal_L_Pelvis_P_J = n.advertise<std_msgs::Float64>("goal_L_Pelvis_P_J", 10);
  P_goal_L_Knee_P_J = n.advertise<std_msgs::Float64>("goal_L_Knee_P_J", 10);
  P_goal_L_Ankle_P_J = n.advertise<std_msgs::Float64>("goal_L_Ankle_P_J", 10);
  P_goal_L_Ankle_R_J = n.advertise<std_msgs::Float64>("goal_L_Ankle_R_J", 10); 
  
  P_ros_msg = n.advertise<std_msgs::Float64MultiArray>("TmpData", 50); // topicname, queue_size = 50
  m_ros_msg.data.resize(50);
  server_sub1 = n.subscribe("Ctrl_mode", 1, &gazebo::SUBO3_plugin::Callback1, this);
  server_sub2 = n.subscribe("CTC_msg", 100, &gazebo::SUBO3_plugin::msgCallback, this);
  server_sub3 = n.subscribe("GROUND_CTC_msg", 100, &gazebo::SUBO3_plugin::msgCallback2, this);
  server_sub4 = n.subscribe("ONE_GROUND_CTC_msg", 100, &gazebo::SUBO3_plugin::msgCallback3, this);
}

void gazebo::SUBO3_plugin::SensorSetting()
{
  //************************Imu Setting*********************************//
  //setting for IMU sensor
  this->Sensor = sensors::get_sensor("BODY_IMU");
  this->BODY_IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);

  this->Sensor = sensors::get_sensor("L_IMU");
  this->L_IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);

  this->Sensor = sensors::get_sensor("R_IMU");
  this->R_IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);
}

void gazebo::SUBO3_plugin::IMUSensorRead()
{
  base_info = this->model->GetWorldPose();

  BODY_ImuGyro(0) = this->BODY_IMU->AngularVelocity(true)[0]; //noiseFree = true
  BODY_ImuGyro(1) = this->BODY_IMU->AngularVelocity(true)[1];
  BODY_ImuGyro(2) = this->BODY_IMU->AngularVelocity(true)[2];
  BODY_ImuAcc(0) = this->BODY_IMU->LinearAcceleration(true)[0];
  BODY_ImuAcc(1) = this->BODY_IMU->LinearAcceleration(true)[1];
  BODY_ImuAcc(2) = this->BODY_IMU->LinearAcceleration(true)[2];

  L_ImuGyro(0) = this->L_IMU->AngularVelocity(true)[0];
  L_ImuGyro(1) = this->L_IMU->AngularVelocity(true)[1];
  L_ImuGyro(2) = this->L_IMU->AngularVelocity(true)[2];
  L_ImuAcc(0) = this->L_IMU->LinearAcceleration(true)[0];
  L_ImuAcc(1) = this->L_IMU->LinearAcceleration(true)[1];
  L_ImuAcc(2) = this->L_IMU->LinearAcceleration(true)[2];

  R_ImuGyro(0) = this->R_IMU->AngularVelocity(true)[0];
  R_ImuGyro(1) = this->R_IMU->AngularVelocity(true)[1];
  R_ImuGyro(2) = this->R_IMU->AngularVelocity(true)[2];
  R_ImuAcc(0) = this->R_IMU->LinearAcceleration(true)[0];
  R_ImuAcc(1) = this->R_IMU->LinearAcceleration(true)[1];
  R_ImuAcc(2) = this->R_IMU->LinearAcceleration(true)[2];   
}

void gazebo::SUBO3_plugin::FTSensorRead()
{
  // Force applied on left leg
  wrench = this->L_FOOTPAD_JOINT->GetForceTorque(0);
  force = wrench.body2Force.Ign();
  torque = wrench.body2Torque.Ign();
  L_Force_E[0] = force.X();
  L_Force_E[1] = force.Y();
  L_Force_E[2] = force.Z();
  L_Torque_E[0] = torque.X();
  L_Torque_E[1] = torque.Y();
  L_Torque_E[2] = torque.Z();
  
  // Force applied on right leg
  
  wrench = this->R_FOOTPAD_JOINT->GetForceTorque(0);
  force = wrench.body2Force.Ign(); // Force on the second link(first = L_TORQUE_SENSOR_LINK, parent. second = L_FOOTPAD_LINK, child.)
  torque = wrench.body2Torque.Ign();
  R_Force_E[0] = force.X();
  R_Force_E[1] = force.Y();
  R_Force_E[2] = force.Z();
  R_Torque_E[0] = torque.X();
  R_Torque_E[1] = torque.Y();
  R_Torque_E[2] = torque.Z();
}

void gazebo::SUBO3_plugin::EncoderRead()
{
  //************************** Encoder ********************************//
  actual_joint_pos[0] = this->L_PELVIS_YAW_JOINT->GetAngle(2).Radian();
  actual_joint_pos[1] = this->L_PELVIS_ROLL_JOINT->GetAngle(0).Radian();
  actual_joint_pos[2] = this->L_PELVIS_PITCH_JOINT->GetAngle(1).Radian();
  actual_joint_pos[3] = this->L_KNEE_PITCH_JOINT->GetAngle(1).Radian();
  actual_joint_pos[4] = this->L_ANKLE_PITCH_JOINT->GetAngle(1).Radian();
  actual_joint_pos[5] = this->L_ANKLE_ROLL_JOINT->GetAngle(0).Radian();
  actual_joint_pos[6] = this->R_PELVIS_YAW_JOINT->GetAngle(2).Radian();
  actual_joint_pos[7] = this->R_PELVIS_ROLL_JOINT->GetAngle(0).Radian();
  actual_joint_pos[8] = this->R_PELVIS_PITCH_JOINT->GetAngle(1).Radian();
  actual_joint_pos[9] = this->R_KNEE_PITCH_JOINT->GetAngle(1).Radian();
  actual_joint_pos[10] = this->R_ANKLE_PITCH_JOINT->GetAngle(1).Radian();
  actual_joint_pos[11] = this->R_ANKLE_ROLL_JOINT->GetAngle(0).Radian();

  //************************** Encoder (dot)********************************//
  for (int i = 0; i < 12; i++)
  {
    actual_joint_vel[i] = (actual_joint_pos[i] - pre_actual_joint_pos[i]) / inner_dt;
    pre_actual_joint_pos[i] = actual_joint_pos[i];
  }
  for (int i = 0; i < 6; i++)
  {
    Act_LL_th[i] = double(actual_joint_pos[i]);
    Act_RL_th[i] = double(actual_joint_pos[i+6]);
  }
}

double LPF_1st(double in, double* preOut, double freq_cut, double dt)
{
double tau, res, alpha;

tau = 1 / (2 * PI * freq_cut);
alpha = dt / (tau + dt);

res = alpha * in + (1- alpha) * *preOut;

*preOut = res;
return res;
}

void gazebo::SUBO3_plugin::Calc_Feedback_Pos(A_RBDL &rbdl)
{
  Math::Vector3d Foot_Pos;
  Math::Matrix3d R, R_Tmp;
  VectorNd QDot;
  MatrixNd L_Jacobian8, L_Jacobian, L_Jacobian_tmp, L_Bmatrix;

  QDot = VectorNd::Zero(6);

  L_Jacobian8 = MatrixNd::Zero(6,9);
  L_Jacobian = MatrixNd::Zero(6,6);
  L_Jacobian_tmp = MatrixNd::Zero(6,6);
  L_Bmatrix = MatrixNd::Zero(6,6);
  
  double pi = 0, theta = 0, psi = 0;

  //*********************Left Leg**********************//
  // Get the End Effector's Aixs
  Foot_Pos = CalcBodyToBaseCoordinates(*rbdl.rbdl_model, rbdl.Q, rbdl.body_FOOT_id, Math::Vector3d(0, 0, 0), true);
  
  // Get the End Effector's Rotation Matrix
  R_Tmp = CalcBodyWorldOrientation(*rbdl.rbdl_model, rbdl.Q, rbdl.body_FOOT_id, true);
  R = R_Tmp.transpose();

  // Get the Euler Angle - pi, theta, psi
  pi = atan2(R(1,0),R(0,0));
  theta = atan2(-R(2,0), cos(pi)*R(0,0) + sin(pi)*R(1,0));
  psi = atan2(sin(pi)*R(0,2) - cos(pi)*R(1,2), -sin(pi)*R(0,1) + cos(pi)*R(1,1));

  // Get the Jacobian
  CalcPointJacobian6D(*rbdl.rbdl_model, rbdl.Q, rbdl.body_FOOT_id, Math::Vector3d(0, 0, 0), L_Jacobian8, true);
  
  // Get the Jacobian for 6 by 6
  for(int i = 0; i < 6; i++)
  {
    for(int j = 0; j < 6; j++)
    {
      L_Jacobian_tmp(i, j) = L_Jacobian8(i, j+3);
    }
  }
  
  // Chage the Row
  for(int j = 0; j < 6; j++)
  {
    for(int i = 0; i < 3; i++)
    {
      L_Jacobian(i,j) = L_Jacobian_tmp(i+3,j);  // linear
    }
    for(int i = 3; i < 6; i++)
    {
      L_Jacobian(i,j) = L_Jacobian_tmp(i-3,j);  // angular
    }
  }

  // Calculate the Analytical Jacobian & Inverse of Analytical Jacobian
  L_Bmatrix << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, cos(pi)/cos(theta), sin(pi)/cos(theta), 0, 0, 0, 0, -sin(pi), cos(pi), 0, 0, 0, 0, cos(pi)*tan(theta), sin(pi)*tan(theta), 1;
  rbdl.A_Jacobian = L_Bmatrix*L_Jacobian;
  rbdl.Inv_A_Jacobian = rbdl.A_Jacobian.inverse();

  // Calculate the Jacobian dot
  for(int i = 0; i < 6; i++)
  {
    for(int j = 0 ; j< 6; j++)
    {
      rbdl.A_Jacobian_dot(i,j) = (rbdl.A_Jacobian(i,j) - rbdl.prev_A_Jacobian(i,j)) / inner_dt;
      rbdl.prev_A_Jacobian(i,j) = rbdl.A_Jacobian(i,j);
    }
    QDot(i) = rbdl.QDot(i+3);
  }

  // Current Pos & Pos_dot
  rbdl.Foot_Pos(0) = Foot_Pos(0);
  rbdl.Foot_Pos(1) = Foot_Pos(1);
  rbdl.Foot_Pos(2) = Foot_Pos(2);
  rbdl.Foot_Pos(3) = psi;
  rbdl.Foot_Pos(4) = theta;
  rbdl.Foot_Pos(5) = pi;
  rbdl.Foot_Pos_dot = rbdl.A_Jacobian*QDot;
}

void gazebo::SUBO3_plugin::Calc_CTC_Torque(A_RBDL &rbdl)
{
  VectorNd QDot;
  QDot = VectorNd::Zero(6);

  rbdl.Kp << 4000, 4000, 10000, 4000, 10000, 4000;
  rbdl.Kv << 100, 100, 250, 100, 250, 100;

  //*********************Left Leg**********************//
  VectorNd X_CTC;
  X_CTC = VectorNd::Zero(6);

  VectorNd q_CTC;
  q_CTC = VectorNd::Zero(6);

  VectorNd NE_Tau;
  NE_Tau = VectorNd::Zero(6);

  MatrixNd I_Matrix_tmp, I_Matrix;
  I_Matrix = MatrixNd::Zero(6,6);
  I_Matrix_tmp = MatrixNd::Zero(9,9);

  for(int i = 0; i < 6; i++)
  {
    X_CTC(i) = rbdl.Des_XDDot(i) + rbdl.Kp(i) * (rbdl.Des_X(i) - rbdl.Foot_Pos(i)) + rbdl.Kv(i) * (rbdl.Des_XDot(i) - rbdl.Foot_Pos_dot(i));
    QDot(i) = rbdl.QDot(i+3);
  }

  q_CTC = rbdl.Inv_A_Jacobian * (X_CTC - rbdl.A_Jacobian_dot*QDot);

  NonlinearEffects(*rbdl.rbdl_model, rbdl.Q, rbdl.QDot, rbdl.Tau, NULL);
  CompositeRigidBodyAlgorithm(*rbdl.rbdl_model, rbdl.Q, I_Matrix_tmp, true);

  for(int i = 0; i < 6; i++)
  {
    NE_Tau(i) = rbdl.Tau(i+3);
    for(int j = 0; j < 6; j++)
    {
      I_Matrix(i,j) = I_Matrix_tmp(i+3,j+3);
    }
  }

  rbdl.torque_CTC = I_Matrix * q_CTC + NE_Tau;
}

void gazebo::SUBO3_plugin::Calc_Feedback_Pos(G_RBDL &rbdl)
{
  Math::Vector3d Body_Pos;
  Math::Matrix3d R, R_Tmp;
  MatrixNd L_Jacobian, L_Jacobian_tmp, L_Bmatrix;

  L_Jacobian = MatrixNd::Zero(6,6);
  L_Jacobian_tmp = MatrixNd::Zero(6,6);
  L_Bmatrix = MatrixNd::Zero(6,6);
  
  double pi = 0, theta = 0, psi = 0;

  //*********************Left Leg**********************//
  // Get the End Effector's Aixs
  Body_Pos = CalcBodyToBaseCoordinates(*rbdl.rbdl_model, rbdl.Q, rbdl.body_id, Math::Vector3d(0, 0, 0), true);
  
  // Get the End Effector's Rotation Matrix
  R_Tmp = CalcBodyWorldOrientation(*rbdl.rbdl_model, rbdl.Q, rbdl.body_id, true);
  R = R_Tmp.transpose();

  // Get the Euler Angle - pi, theta, psi
  pi = atan2(R(1,0),R(0,0));
  theta = atan2(-R(2,0), cos(pi)*R(0,0) + sin(pi)*R(1,0));
  psi = atan2(sin(pi)*R(0,2) - cos(pi)*R(1,2), -sin(pi)*R(0,1) + cos(pi)*R(1,1));

  // Get the Jacobian
  CalcPointJacobian6D(*rbdl.rbdl_model, rbdl.Q, rbdl.body_id, Math::Vector3d(0, 0, 0), L_Jacobian_tmp, true);

  // Chage the Row
  for(int j = 0; j < 6; j++)
  {
    for(int i = 0; i < 3; i++)
    {
      L_Jacobian(i,j) = L_Jacobian_tmp(i+3,j);  // linear
    }
    for(int i = 3; i < 6; i++)
    {
      L_Jacobian(i,j) = L_Jacobian_tmp(i-3,j);  // angular
    }
  }

  // Calculate the Analytical Jacobian & Inverse of Analytical Jacobian
  L_Bmatrix << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, cos(pi)/cos(theta), sin(pi)/cos(theta), 0, 0, 0, 0, -sin(pi), cos(pi), 0, 0, 0, 0, cos(pi)*tan(theta), sin(pi)*tan(theta), 1;
  rbdl.A_Jacobian = L_Bmatrix*L_Jacobian;
  rbdl.Inv_A_Jacobian = rbdl.A_Jacobian.inverse();

  // Calculate the Jacobian dot
  for(int i = 0; i < 6; i++)
  {
    for(int j = 0 ; j< 6; j++)
    {
      rbdl.A_Jacobian_dot(i,j) = (rbdl.A_Jacobian(i,j) - rbdl.prev_A_Jacobian(i,j)) / inner_dt;
      rbdl.prev_A_Jacobian(i,j) = rbdl.A_Jacobian(i,j);
    }
  }

  // Current Pos & Pos_dot
  rbdl.Foot_Pos(0) = Body_Pos(0);
  rbdl.Foot_Pos(1) = Body_Pos(1);
  rbdl.Foot_Pos(2) = Body_Pos(2);
  rbdl.Foot_Pos(3) = psi;
  rbdl.Foot_Pos(4) = theta;
  rbdl.Foot_Pos(5) = pi;
  rbdl.Foot_Pos_dot = rbdl.A_Jacobian*rbdl.QDot;
}

void gazebo::SUBO3_plugin::Calc_CTC_Torque(G_RBDL &rbdl)
{
  if(rbdl.num_leg == 0)
  {
    rbdl.Kp << 100, 3000, 3000, 3000, 3000, 3000;
    rbdl.Kv << 1, 6, 6, 6, 6, 6;
  }
  else if(rbdl.num_leg == 1)
  {
    if(start_flag == 0)
    {
      rbdl.Kp << 10, 1000, 1000, 1000, 1000, 1000;
      rbdl.Kv << 1, 6, 6, 6, 6, 6;
    }
  }

  //*********************Left Leg**********************//
  VectorNd X_CTC;
  X_CTC = VectorNd::Zero(6);

  VectorNd q_CTC;
  q_CTC = VectorNd::Zero(6);

  VectorNd NE_Tau;
  NE_Tau = VectorNd::Zero(6);

  MatrixNd I_Matrix_tmp, I_Matrix;
  I_Matrix = MatrixNd::Zero(6,6);

  for(int i = 0; i < 6; i++)
  {
    X_CTC(i) = rbdl.Des_XDDot(i) + rbdl.Kp(i) * (rbdl.Des_X(i) - rbdl.Foot_Pos(i)) + rbdl.Kv(i) * (rbdl.Des_XDot(i) - rbdl.Foot_Pos_dot(i));
  }

  q_CTC = rbdl.Inv_A_Jacobian * (X_CTC - rbdl.A_Jacobian_dot*rbdl.QDot);

  NonlinearEffects(*rbdl.rbdl_model, rbdl.Q, rbdl.QDot, NE_Tau, NULL);
  CompositeRigidBodyAlgorithm(*rbdl.rbdl_model, rbdl.Q, I_Matrix, true);

  rbdl.torque_CTC = I_Matrix * q_CTC + NE_Tau;
}

void gazebo::SUBO3_plugin::CalcBodyAngle()
{
  double calBuff;
  Math::Matrix3d R_rll, R_pit, R_matrix;
  Math::Vector3d tmp;

  // 57.29578 = 180 / pi : radian to deg
  calBuff = BODY_ImuAcc(1) * BODY_ImuAcc(1) + BODY_ImuAcc(2) * BODY_ImuAcc(2);
  accel(0) = atan2(BODY_ImuAcc(0), sqrt(calBuff)) * rad2deg;
  accel(1) = atan2(BODY_ImuAcc(1), BODY_ImuAcc(2)) * rad2deg;

  // raw data * (real scale) / (data scale) -> 250 / 32768 = 0.007629394
  gyro_rate(0) = (double)BODY_ImuGyro(0) * deg2rad;
  gyro_rate(1) = (double)BODY_ImuGyro(1) * deg2rad;
  gyro_rate(2) = (double)BODY_ImuGyro(2) * deg2rad;

  // 1st Low pass filter
  LPF_BODY_ImuAcc(0) = LPF_1st(accel(0), &preLPF_BODY_ImuAcc(0), 25, inner_dt);
  LPF_BODY_ImuAcc(1) = LPF_1st(accel(1), &preLPF_BODY_ImuAcc(1), 25, inner_dt);

  LPF_BODY_ImuGyro(0) = LPF_1st(gyro_rate(0), &preLPF_BODY_ImuGyro(0), 25, inner_dt);
  LPF_BODY_ImuGyro(1) = LPF_1st(gyro_rate(1), &preLPF_BODY_ImuGyro(1), 25, inner_dt);

  imu_rate(0) = LPF_BODY_ImuGyro(0);
  imu_rate(1) = LPF_BODY_ImuGyro(1);

  LPF_BODY_ImuGyro(2) = LPF_1st(gyro_rate(2), &preLPF_BODY_ImuGyro(2), 25, inner_dt);
  imu_rate(2) = LPF_BODY_ImuGyro(2);

  body_EAngle(0) = 0.997 * (body_EAngle(0) + imu_rate(0) * inner_dt) + 0.003 * LPF_BODY_ImuAcc(0);
  body_EAngle(1) = 0.997 * (body_EAngle(1) + imu_rate(1) * inner_dt) + 0.003 * LPF_BODY_ImuAcc(1);
  body_EAngle(2) = body_EAngle(2) + imu_rate(2) * inner_dt;

  R_rll << 1, 0, 0, 0, cos(body_EAngle(1)*deg2rad), sin(body_EAngle(1)*deg2rad), 0, -sin(body_EAngle(1)*deg2rad), cos(body_EAngle(1)*deg2rad);
  R_pit << cos(-body_EAngle(0)*deg2rad), 0, -sin(-body_EAngle(0)*deg2rad), 0, 1, 0, sin(-body_EAngle(0)*deg2rad), 0, cos(-body_EAngle(0)*deg2rad);

  A_L.Q(0) = -body_EAngle(0)*deg2rad;
  A_L.Q(1) = body_EAngle(1)*deg2rad;
  A_R.Q(0) = -body_EAngle(0)*deg2rad;
  A_R.Q(1) = body_EAngle(1)*deg2rad;
}

void gazebo::SUBO3_plugin::torque_interpolation()
{
  if(f_cnt == 0)
  {
    for(int i = 0 ; i < 12 ; i++)
    {
      out_joint[i].torque = prev_out_joint[i].torque + (joint[i].torque - prev_out_joint[i].torque) * 0.25;
    }
  }
  else if(f_cnt == 1)
  {
    for(int i = 0 ; i < 12 ; i++)
    {
      out_joint[i].torque = prev_out_joint[i].torque + (joint[i].torque - prev_out_joint[i].torque) * 0.5;
    }
  }
  else if(f_cnt == 2)
  {
    for(int i = 0 ; i < 12 ; i++)
    {
      out_joint[i].torque = prev_out_joint[i].torque + (joint[i].torque - prev_out_joint[i].torque) * 0.75;
    }
  }
  else if(f_cnt == 3)
  {
    for(int i = 0 ; i < 12 ; i++)
    {
      out_joint[i].torque = joint[i].torque;
    }
  }
}

void gazebo::SUBO3_plugin::jointController()
{
  //Torque Limit 감속기 정격토크참조함.
  for (unsigned int i = 0; i < 3; ++i)
  {
    if (joint[i].torque >= 8560*3)
    {
      joint[i].torque = 8560*3;
    }
    else if (joint[i].torque <= -8560*3)
    {
      joint[i].torque = -8560*3;
    }
  }

  for (unsigned int i = 4; i < 9; ++i)
  {
    if (joint[i].torque >= 8560*3)
    {
        joint[i].torque = 8560*3;
    }
    else if (joint[i].torque <= -8560*3)
    {
        joint[i].torque = -8560*3;
    }
  }
  
  for (unsigned int i = 10; i < 12; ++i)
  {
    if (joint[i].torque >= 8560*3)
    {
        joint[i].torque = 8560*3;
    }
    else if (joint[i].torque <= -8560*3)
    {
        joint[i].torque = -8560*3;
    }
  }
                  
  if (joint[3].torque >=12840*3)
  {
    joint[3].torque = 12840*3;
  }
  else if (joint[3].torque <= -12840*3)
  {
    joint[3].torque = -12840*3;
  }

  if (joint[9].torque >=12840*3)
  {
    joint[9].torque = 12840*3;
  }
  else if (joint[9].torque <= -12840*3)
  {
    joint[9].torque = -12840*3;
  }

  //* Applying torques
  this->L_PELVIS_YAW_JOINT->SetForce(2, joint[0].torque); //SUBO3.target_tor[0]);
  this->L_PELVIS_ROLL_JOINT->SetForce(0, joint[1].torque); //SUBO3.target_tor[1]);
  this->L_PELVIS_PITCH_JOINT->SetForce(1, joint[2].torque); //SUBO3.target_tor[2]);
  this->L_KNEE_PITCH_JOINT->SetForce(1, joint[3].torque); //SUBO3.target_tor[3]);
  this->L_ANKLE_PITCH_JOINT->SetForce(1, joint[4].torque); //SUBO3.target_tor[4]);
  this->L_ANKLE_ROLL_JOINT->SetForce(0, joint[5].torque); //SUBO3.target_tor[5]);

  this->R_PELVIS_YAW_JOINT->SetForce(2, joint[6].torque); //SUBO3.target_tor[6]);
  this->R_PELVIS_ROLL_JOINT->SetForce(0, joint[7].torque); //SUBO3.target_tor[7]);
  this->R_PELVIS_PITCH_JOINT->SetForce(1, joint[8].torque); //SUBO3.target_tor[8]);
  this->R_KNEE_PITCH_JOINT->SetForce(1, joint[9].torque); //SUBO3.target_tor[9]);
  this->R_ANKLE_PITCH_JOINT->SetForce(1, joint[10].torque); //SUBO3.target_tor[10]);
  this->R_ANKLE_ROLL_JOINT->SetForce(0, joint[11].torque); //SUBO3.target_tor[11]);

  cout << "Left Pelvis Yaw: " << joint[0].torque << endl;
  cout << "Left Pelvis Roll: " << joint[1].torque << endl;
  cout << "Left Pelvis Pitch: " << joint[2].torque << endl;
  cout << "Left Knee Pitch: " << joint[3].torque << endl;
  cout << "Left Ankle Pitch: " << joint[4].torque << endl;
  cout << "Left Ankle Roll: " << joint[5].torque << endl << endl;

  cout << "Right Pelvis Yaw: " << joint[6].torque << endl;
  cout << "Right Pelvis Roll: " << joint[7].torque << endl;
  cout << "Right Pelvis Pitch: " << joint[8].torque << endl;
  cout << "Right Knee Pitch: " << joint[9].torque << endl;
  cout << "Right Ankle Pitch: " << joint[10].torque << endl;
  cout << "Right Ankle Roll: " << joint[11].torque << endl;
  cout << "======================================================" << endl;
}

void gazebo::SUBO3_plugin::Callback1(const std_msgs::Int32Ptr &msg)
{
  cnt=0;       
  Up_Down_iteration_cnt=0;
  
  if (msg->data == 0) //button-6
  {
    cnt = 0;
    CONTROL_MODE = IDLE;
  }       
  else if (msg->data == 1) //button-6
  {
    cnt = 0;
    CONTROL_MODE = GRAVITY_CONTROL;
  }
  else if (msg->data == 2) //button-6
  {
    cnt = 0;
    CONTROL_MODE = CTC_CONTROL;
  }
  else if (msg->data == 3) //button-6
  {
    cnt = 0;
    CONTROL_MODE = CTC_CONTROL_POS;
  }
  else if (msg->data == 4) //button-6
  {
    cnt = 0;
    CONTROL_MODE = CTC_CONTROL_CONT_POS;
  }
  else if (msg->data == 5) //button-6
  {
    cnt = 0;
    CONTROL_MODE = GROUND_GRAVITY_CONTROL;
  }
  else if (msg->data == 6) //button-6
  {
    cnt = 0;
    CONTROL_MODE = GROUND_CTC_CONTROL;
  }
  else if (msg->data == 7) //button-6
  {
    cnt = 0;
    CONTROL_MODE = GROUND_CTC_CONTROL_POS;
  }
  else if (msg->data == 8) //button-6
  {
    cnt = 0;
    CONTROL_MODE = GROUND_CTC_CONTROL_CONT_POS;
  }
  else if (msg->data == 9) //button-6
  {
    cnt = 0;
    CONTROL_MODE = ONE_GROUND_CTC_CONTROL;
  }
  else
  {
    cnt = 0;
    CONTROL_MODE = IDLE;
  }
}

void gazebo::SUBO3_plugin::msgCallback(const subo3_pkgs::CTCMsg::ConstPtr &msg)
{
  start_flag = msg->TF;
  if(start_flag == 1)
  {
    A_L.New_Des_X(0) = msg->L_Des_X_x;A_L.New_Des_X(1) = msg->L_Des_X_y;A_L.New_Des_X(2) = msg->L_Des_X_z;
    A_L.New_Des_X(3) = msg->L_Des_X_rll;A_L.New_Des_X(4) = msg->L_Des_X_pit;A_L.New_Des_X(5) = msg->L_Des_X_yaw;
    A_L.New_Des_XDot(0) = msg->L_Des_XDot_x;A_L.New_Des_XDot(1) = msg->L_Des_XDot_y;A_L.New_Des_XDot(2) = msg->L_Des_XDot_z;
    A_L.New_Des_XDot(3) = msg->L_Des_XDot_rll;A_L.New_Des_XDot(4) = msg->L_Des_XDot_pit;A_L.New_Des_XDot(5) = msg->L_Des_XDot_yaw;
    A_L.New_Des_XDDot(0) = msg->L_Des_XDDot_x;A_L.New_Des_XDDot(1) = msg->L_Des_XDDot_y;A_L.New_Des_XDDot(2) = msg->L_Des_XDDot_z;
    A_L.New_Des_XDDot(3) = msg->L_Des_XDDot_rll;A_L.New_Des_XDDot(4) = msg->L_Des_XDDot_pit;A_L.New_Des_XDDot(5) = msg->L_Des_XDDot_yaw;

    A_R.New_Des_X(0) = msg->R_Des_X_x;A_R.New_Des_X(1) = msg->R_Des_X_y;A_R.New_Des_X(2) = msg->R_Des_X_z;
    A_R.New_Des_X(3) = msg->R_Des_X_rll;A_R.New_Des_X(4) = msg->R_Des_X_pit;A_R.New_Des_X(5) = msg->R_Des_X_yaw;
    A_R.New_Des_XDot(0) = msg->R_Des_XDot_x;A_R.New_Des_XDot(1) = msg->R_Des_XDot_y;A_R.New_Des_XDot(2) = msg->R_Des_XDot_z;
    A_R.New_Des_XDot(3) = msg->R_Des_XDot_rll;A_R.New_Des_XDot(4) = msg->R_Des_XDot_pit;A_R.New_Des_XDot(5) = msg->R_Des_XDot_yaw;
    A_R.New_Des_XDDot(0) = msg->R_Des_XDDot_x;A_R.New_Des_XDDot(1) = msg->R_Des_XDDot_y;A_R.New_Des_XDDot(2) = msg->R_Des_XDDot_z;
    A_R.New_Des_XDDot(3) = msg->R_Des_XDDot_rll;A_R.New_Des_XDDot(4) = msg->R_Des_XDDot_pit;A_R.New_Des_XDDot(5) = msg->R_Des_XDDot_yaw;

    A_L.Kp(0) = msg->Kp0;
    A_L.Kp(1) = msg->Kp1;
    A_L.Kp(2) = msg->Kp2;
    A_L.Kp(3) = msg->Kp3;
    A_L.Kp(4) = msg->Kp4;
    A_L.Kp(5) = msg->Kp5;
    A_L.Kv(0) = msg->Kv0;
    A_L.Kv(1) = msg->Kv1;
    A_L.Kv(2) = msg->Kv2;
    A_L.Kv(3) = msg->Kv3;
    A_L.Kv(4) = msg->Kv4;
    A_L.Kv(5) = msg->Kv5;

    A_R.Kp(0) = msg->Kp0;
    A_R.Kp(1) = msg->Kp1;
    A_R.Kp(2) = msg->Kp2;
    A_R.Kp(3) = msg->Kp3;
    A_R.Kp(4) = msg->Kp4;
    A_R.Kp(5) = msg->Kp5;
    A_R.Kv(0) = msg->Kv0;
    A_R.Kv(1) = msg->Kv1;
    A_R.Kv(2) = msg->Kv2;
    A_R.Kv(3) = msg->Kv3;
    A_R.Kv(4) = msg->Kv4;
    A_R.Kv(5) = msg->Kv5;
  }
}

void gazebo::SUBO3_plugin::msgCallback2(const subo3_pkgs::CTCMsg::ConstPtr &msg)
{
  start_flag = msg->TF;
  if(start_flag == 1)
  {
    G_L.New_Des_X(0) = msg->L_Des_X_x;G_L.New_Des_X(1) = msg->L_Des_X_y;G_L.New_Des_X(2) = msg->L_Des_X_z;
    G_L.New_Des_X(3) = msg->L_Des_X_rll;G_L.New_Des_X(4) = msg->L_Des_X_pit;G_L.New_Des_X(5) = msg->L_Des_X_yaw;
    G_L.New_Des_XDot(0) = msg->L_Des_XDot_x;G_L.New_Des_XDot(1) = msg->L_Des_XDot_y;G_L.New_Des_XDot(2) = msg->L_Des_XDot_z;
    G_L.New_Des_XDot(3) = msg->L_Des_XDot_rll;G_L.New_Des_XDot(4) = msg->L_Des_XDot_pit;G_L.New_Des_XDot(5) = msg->L_Des_XDot_yaw;
    G_L.New_Des_XDDot(0) = msg->L_Des_XDDot_x;G_L.New_Des_XDDot(1) = msg->L_Des_XDDot_y;G_L.New_Des_XDDot(2) = msg->L_Des_XDDot_z;
    G_L.New_Des_XDDot(3) = msg->L_Des_XDDot_rll;G_L.New_Des_XDDot(4) = msg->L_Des_XDDot_pit;G_L.New_Des_XDDot(5) = msg->L_Des_XDDot_yaw;

    G_R.New_Des_X(0) = msg->R_Des_X_x;G_R.New_Des_X(1) = msg->R_Des_X_y;G_R.New_Des_X(2) = msg->R_Des_X_z;
    G_R.New_Des_X(3) = msg->R_Des_X_rll;G_R.New_Des_X(4) = msg->R_Des_X_pit;G_R.New_Des_X(5) = msg->R_Des_X_yaw;
    G_R.New_Des_XDot(0) = msg->R_Des_XDot_x;G_R.New_Des_XDot(1) = msg->R_Des_XDot_y;G_R.New_Des_XDot(2) = msg->R_Des_XDot_z;
    G_R.New_Des_XDot(3) = msg->R_Des_XDot_rll;G_R.New_Des_XDot(4) = msg->R_Des_XDot_pit;G_R.New_Des_XDot(5) = msg->R_Des_XDot_yaw;
    G_R.New_Des_XDDot(0) = msg->R_Des_XDDot_x;G_R.New_Des_XDDot(1) = msg->R_Des_XDDot_y;G_R.New_Des_XDDot(2) = msg->R_Des_XDDot_z;
    G_R.New_Des_XDDot(3) = msg->R_Des_XDDot_rll;G_R.New_Des_XDDot(4) = msg->R_Des_XDDot_pit;G_R.New_Des_XDDot(5) = msg->R_Des_XDDot_yaw;

    G_L.Kp(0) = msg->Kp0;
    G_L.Kp(1) = msg->Kp1;
    G_L.Kp(2) = msg->Kp2;
    G_L.Kp(3) = msg->Kp3;
    G_L.Kp(4) = msg->Kp4;
    G_L.Kp(5) = msg->Kp5;
    G_L.Kv(0) = msg->Kv0;
    G_L.Kv(1) = msg->Kv1;
    G_L.Kv(2) = msg->Kv2;
    G_L.Kv(3) = msg->Kv3;
    G_L.Kv(4) = msg->Kv4;
    G_L.Kv(5) = msg->Kv5;

    G_R.Kp(0) = msg->Kp0;
    G_R.Kp(1) = msg->Kp1;
    G_R.Kp(2) = msg->Kp2;
    G_R.Kp(3) = msg->Kp3;
    G_R.Kp(4) = msg->Kp4;
    G_R.Kp(5) = msg->Kp5;
    G_R.Kv(0) = msg->Kv0;
    G_R.Kv(1) = msg->Kv1;
    G_R.Kv(2) = msg->Kv2;
    G_R.Kv(3) = msg->Kv3;
    G_R.Kv(4) = msg->Kv4;
    G_R.Kv(5) = msg->Kv5;
  }
}

void gazebo::SUBO3_plugin::msgCallback3(const subo3_pkgs::CTCMsg::ConstPtr &msg)
{
  start_flag = msg->TF;
  if(start_flag == 1)
  {
    O_L.New_Des_X(0) = msg->L_Des_X_x;O_L.New_Des_X(1) = msg->L_Des_X_y;O_L.New_Des_X(2) = msg->L_Des_X_z;
    O_L.New_Des_X(3) = msg->L_Des_X_rll;O_L.New_Des_X(4) = msg->L_Des_X_pit;O_L.New_Des_X(5) = msg->L_Des_X_yaw;
    O_L.New_Des_XDot(0) = msg->L_Des_XDot_x;O_L.New_Des_XDot(1) = msg->L_Des_XDot_y;O_L.New_Des_XDot(2) = msg->L_Des_XDot_z;
    O_L.New_Des_XDot(3) = msg->L_Des_XDot_rll;O_L.New_Des_XDot(4) = msg->L_Des_XDot_pit;O_L.New_Des_XDot(5) = msg->L_Des_XDot_yaw;
    O_L.New_Des_XDDot(0) = msg->L_Des_XDDot_x;O_L.New_Des_XDDot(1) = msg->L_Des_XDDot_y;O_L.New_Des_XDDot(2) = msg->L_Des_XDDot_z;
    O_L.New_Des_XDDot(3) = msg->L_Des_XDDot_rll;O_L.New_Des_XDDot(4) = msg->L_Des_XDDot_pit;O_L.New_Des_XDDot(5) = msg->L_Des_XDDot_yaw;

    O_R.New_Des_X(0) = msg->R_Des_X_x;O_R.New_Des_X(1) = msg->R_Des_X_y;O_R.New_Des_X(2) = msg->R_Des_X_z;
    O_R.New_Des_X(3) = msg->R_Des_X_rll;O_R.New_Des_X(4) = msg->R_Des_X_pit;O_R.New_Des_X(5) = msg->R_Des_X_yaw;
    O_R.New_Des_XDot(0) = msg->R_Des_XDot_x;O_R.New_Des_XDot(1) = msg->R_Des_XDot_y;O_R.New_Des_XDot(2) = msg->R_Des_XDot_z;
    O_R.New_Des_XDot(3) = msg->R_Des_XDot_rll;O_R.New_Des_XDot(4) = msg->R_Des_XDot_pit;O_R.New_Des_XDot(5) = msg->R_Des_XDot_yaw;
    O_R.New_Des_XDDot(0) = msg->R_Des_XDDot_x;O_R.New_Des_XDDot(1) = msg->R_Des_XDDot_y;O_R.New_Des_XDDot(2) = msg->R_Des_XDDot_z;
    O_R.New_Des_XDDot(3) = msg->R_Des_XDDot_rll;O_R.New_Des_XDDot(4) = msg->R_Des_XDDot_pit;O_R.New_Des_XDDot(5) = msg->R_Des_XDDot_yaw;

    O_L.Kp(0) = msg->Kp0;
    O_L.Kp(1) = msg->Kp1;
    O_L.Kp(2) = msg->Kp2;
    O_L.Kp(3) = msg->Kp3;
    O_L.Kp(4) = msg->Kp4;
    O_L.Kp(5) = msg->Kp5;
    O_L.Kv(0) = msg->Kv0;
    O_L.Kv(1) = msg->Kv1;
    O_L.Kv(2) = msg->Kv2;
    O_L.Kv(3) = msg->Kv3;
    O_L.Kv(4) = msg->Kv4;
    O_L.Kv(5) = msg->Kv5;

    O_R.Kp(0) = msg->Kp0;
    O_R.Kp(1) = msg->Kp1;
    O_R.Kp(2) = msg->Kp2;
    O_R.Kp(3) = msg->Kp3;
    O_R.Kp(4) = msg->Kp4;
    O_R.Kp(5) = msg->Kp5;
    O_R.Kv(0) = msg->Kv0;
    O_R.Kv(1) = msg->Kv1;
    O_R.Kv(2) = msg->Kv2;
    O_R.Kv(3) = msg->Kv3;
    O_R.Kv(4) = msg->Kv4;
    O_R.Kv(5) = msg->Kv5;
  }
}

void gazebo::SUBO3_plugin::PostureGeneration()
{
  int test_cnt = 0;
  // cout << dt << endl;
  if (CONTROL_MODE == IDLE)
  {
    //std::cout << "Mode is Init_Pos_Mode" << std::endl;
    Init_Pos_Traj();
  }
  else if (CONTROL_MODE == GRAVITY_CONTROL) 
  {
	  test_cnt++;
    Gravity_Cont();
  }
  else if (CONTROL_MODE == CTC_CONTROL) 
  {
	  test_cnt++;
    CTC_Control();
  }
  else if (CONTROL_MODE == CTC_CONTROL_POS) 
  {
	  test_cnt++;
    CTC_Control_Pos();
  }
  else if (CONTROL_MODE == CTC_CONTROL_CONT_POS) 
  {
    test_cnt++;
    CTC_Control_Cont_Pos();
  }
  else if (CONTROL_MODE == GROUND_GRAVITY_CONTROL) 
  {
    test_cnt++;
    GROUND_Gravity_Cont();
  }
  else if (CONTROL_MODE == GROUND_CTC_CONTROL) 
  {
    test_cnt++;
    GROUND_CTC_Control();
  }
  else if (CONTROL_MODE == GROUND_CTC_CONTROL_POS) 
  {
	  test_cnt++;
    GROUND_CTC_Control_Pos();
  }
  else if (CONTROL_MODE == GROUND_CTC_CONTROL_CONT_POS) 
  {
    test_cnt++;
    GROUND_CTC_Control_Cont_Pos();
  }
  else if (CONTROL_MODE == ONE_GROUND_CTC_CONTROL) 
  {
    test_cnt++;
    ONE_GROUND_CTC_Control();
  } 
}

void gazebo::SUBO3_plugin::RBDL_variable_update()
{
  // Air variable
  A_L.Q(3) = actual_joint_pos[0];
  A_L.Q(4) = actual_joint_pos[1];
  A_L.Q(5) = actual_joint_pos[2];
  A_L.Q(6) = actual_joint_pos[3];
  A_L.Q(7) = actual_joint_pos[4];
  A_L.Q(8) = actual_joint_pos[5];
  
  A_R.Q(3) = actual_joint_pos[6];
  A_R.Q(4) = actual_joint_pos[7];
  A_R.Q(5) = actual_joint_pos[8];
  A_R.Q(6) = actual_joint_pos[9];
  A_R.Q(7) = actual_joint_pos[10];
  A_R.Q(8) = actual_joint_pos[11];

  A_L.QDot = (A_L.Q - A_L.prevQ) / inner_dt;
  A_L.QDDot = (A_L.QDot - A_L.prevQDot) / inner_dt;
  A_R.QDot = (A_R.Q - A_R.prevQ) / inner_dt;
  A_R.QDDot = (A_R.QDot - A_R.prevQDot) / inner_dt;

  A_L.prevQ = A_L.Q;
  A_L.prevQDot = A_L.QDot;
  A_R.prevQ = A_R.Q;
  A_R.prevQDot = A_R.QDot;

  // Two leg Ground variable
  G_L.Q(0) = actual_joint_pos[5];
  G_L.Q(1) = actual_joint_pos[4];
  G_L.Q(2) = actual_joint_pos[3];
  G_L.Q(3) = actual_joint_pos[2];
  G_L.Q(4) = actual_joint_pos[1];
  G_L.Q(5) = actual_joint_pos[0];

  G_R.Q(0) = actual_joint_pos[11];
  G_R.Q(1) = actual_joint_pos[10];
  G_R.Q(2) = actual_joint_pos[9];
  G_R.Q(3) = actual_joint_pos[8];
  G_R.Q(4) = actual_joint_pos[7];
  G_R.Q(5) = actual_joint_pos[6];

  G_L.QDot = (G_L.Q - G_L.prevQ) / inner_dt;
  G_L.QDDot = (G_L.QDot - G_L.prevQDot) / inner_dt;
  G_R.QDot = (G_R.Q - G_R.prevQ) / inner_dt;
  G_R.QDDot = (G_R.QDot - G_R.prevQDot) / inner_dt;

  G_L.prevQ = G_L.Q;
  G_L.prevQDot = G_L.QDot;
  G_R.prevQ = G_R.Q;
  G_R.prevQDot = G_R.QDot;

  // One leg Ground variable
  O_L.Q(0) = actual_joint_pos[5];
  O_L.Q(1) = actual_joint_pos[4];
  O_L.Q(2) = actual_joint_pos[3];
  O_L.Q(3) = actual_joint_pos[2];
  O_L.Q(4) = actual_joint_pos[1];
  O_L.Q(5) = actual_joint_pos[0];

  O_R.Q(0) = actual_joint_pos[11];
  O_R.Q(1) = actual_joint_pos[10];
  O_R.Q(2) = actual_joint_pos[9];
  O_R.Q(3) = actual_joint_pos[8];
  O_R.Q(4) = actual_joint_pos[7];
  O_R.Q(5) = actual_joint_pos[6];

  O_L.QDot = (O_L.Q - O_L.prevQ) / inner_dt;
  O_L.QDDot = (O_L.QDot - O_L.prevQDot) / inner_dt;
  O_R.QDot = (O_R.Q - O_R.prevQ) / inner_dt;
  O_R.QDDot = (O_R.QDot - O_R.prevQDot) / inner_dt;

  O_L.prevQ = O_L.Q;
  O_L.prevQDot = O_L.QDot;
  O_R.prevQ = O_R.Q;
  O_R.prevQDot = O_R.QDot;
}

void gazebo::SUBO3_plugin::Init_Pos_Traj()  // 0
{
  Kp_q << 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200;
  Kd_q << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
  step_time = 2; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;

  double Init_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  if(cnt_time <= step_time)
  {
    // Theo_RL_th[0] = 0*Init_trajectory*deg2rad;
    // Theo_RL_th[1] = 0*Init_trajectory*deg2rad;
    // Theo_RL_th[2] = -30*Init_trajectory*deg2rad;
    // Theo_RL_th[3] = 60*Init_trajectory*deg2rad;
    // Theo_RL_th[4] = -30*Init_trajectory*deg2rad;
    // Theo_RL_th[5] = 0*Init_trajectory*deg2rad;

    // Theo_LL_th[0] = 0*Init_trajectory*deg2rad;
    // Theo_LL_th[1] = 0*Init_trajectory*deg2rad;
    // Theo_LL_th[2] = -30*Init_trajectory*deg2rad;
    // Theo_LL_th[3] = 60*Init_trajectory*deg2rad;
    // Theo_LL_th[4] = -30*Init_trajectory*deg2rad;
    // Theo_LL_th[5] = 0*Init_trajectory*deg2rad;

    //one leg
    Theo_RL_th[0] = 0*Init_trajectory*deg2rad;
    Theo_RL_th[1] = -5*Init_trajectory*deg2rad;
    Theo_RL_th[2] = -30*Init_trajectory*deg2rad;
    Theo_RL_th[3] = 60*Init_trajectory*deg2rad;
    Theo_RL_th[4] = -30*Init_trajectory*deg2rad;
    Theo_RL_th[5] = 5*Init_trajectory*deg2rad;

    Theo_LL_th[0] = 0*Init_trajectory*deg2rad;
    Theo_LL_th[1] = -5*Init_trajectory*deg2rad;
    Theo_LL_th[2] = -30*Init_trajectory*deg2rad;
    Theo_LL_th[3] = 60*Init_trajectory*deg2rad;
    Theo_LL_th[4] = -30*Init_trajectory*deg2rad;
    Theo_LL_th[5] = 5*Init_trajectory*deg2rad;
  }
  
  ///////////////토크 입력////////////////
  for (int i = 0; i < 6; i++) {
    joint[i].torque = Kp_q[i]*(Theo_RL_th[i] - actual_joint_pos[i]) + Kd_q[i] * (0 - actual_joint_vel[i]); // 기본 PV제어 코드
    joint[i+6].torque = Kp_q[i]*(Theo_LL_th[i] - actual_joint_pos[i+6]) + Kd_q[i] * (0 - actual_joint_vel[i+6]); // 기본 PV제어 코드
    
    old_joint[i].torque = joint[i].torque;
    old_joint[i+6].torque = joint[i+6].torque;
  }
}

void gazebo::SUBO3_plugin::Gravity_Cont() // 1
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;

  CalcBodyAngle();

  InverseDynamics(*A_L.rbdl_model, A_L.Q, VectorNd::Zero(9), VectorNd::Zero(9), A_L.Tau, NULL);
  InverseDynamics(*A_R.rbdl_model, A_R.Q, VectorNd::Zero(9), VectorNd::Zero(9), A_R.Tau, NULL);

  for (int i = 0; i < 6; i++)
  {
    joint[i].torque = A_L.Tau(i+3);
    joint[i+6].torque = A_R.Tau(i+3);
  }
}

void gazebo::SUBO3_plugin::CTC_Control()  // 2
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
  double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  A_L.Q(0) = 0;
  A_L.Q(1) = 0;
  A_R.Q(0) = 0;
  A_R.Q(1) = 0;

  // Target Pos, Pos Dot, Pos DDot
  A_L.Des_X(0) = 0;  A_L.Des_X(1) = 0.07;  A_L.Des_X(2) = -0.507;  A_L.Des_X(3) = 0;  A_L.Des_X(4) = 0;  A_L.Des_X(5) = 0;
  A_L.Des_XDot(0) = 0;  A_L.Des_XDot(1) = 0;  A_L.Des_XDot(2) = 0;  A_L.Des_XDot(3) = 0;  A_L.Des_XDot(4) = 0;  A_L.Des_XDot(5) = 0;
  A_L.Des_XDDot(0) = 0;  A_L.Des_XDDot(1) = 0;  A_L.Des_XDDot(2) = 0;  A_L.Des_XDDot(3) = 0;  A_L.Des_XDDot(4) = 0;  A_L.Des_XDDot(5) = 0;

  A_R.Des_X(0) = 0;  A_R.Des_X(1) = -0.07;  A_R.Des_X(2) = -0.507;  A_R.Des_X(3) = 0;  A_R.Des_X(4) = 0;  A_R.Des_X(5) = 0;
  A_R.Des_XDot(0) = 0;  A_R.Des_XDot(1) = 0;  A_R.Des_XDot(2) = 0;  A_R.Des_XDot(3) = 0;  A_R.Des_XDot(4) = 0;  A_R.Des_XDot(5) = 0;
  A_R.Des_XDDot(0) = 0;  A_R.Des_XDDot(1) = 0;  A_R.Des_XDDot(2) = 0;  A_R.Des_XDDot(3) = 0;  A_R.Des_XDDot(4) = 0;  A_R.Des_XDDot(5) = 0;

  Calc_Feedback_Pos(A_L);  // calculate the feedback
  Calc_Feedback_Pos(A_R);  // calculate the feedback
  Calc_CTC_Torque(A_L);    // calculate the CTC torque
  Calc_CTC_Torque(A_R);    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = old_joint[i].torque*old_trajectory + A_L.torque_CTC(i)*new_trajectory;
      joint[i+6].torque = old_joint[i+6].torque*old_trajectory + A_R.torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = A_L.torque_CTC(i);
      joint[i+6].torque = A_R.torque_CTC(i);
      old_joint[i].torque = joint[i].torque;
      old_joint[i+6].torque = joint[i+6].torque;
    }
  }
}

void gazebo::SUBO3_plugin::CTC_Control_Pos()  // 3
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
  double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  A_L.Q(0) = 0;
  A_L.Q(1) = 0;
  A_R.Q(0) = 0;
  A_R.Q(1) = 0;

  // Target Pos, Pos Dot, Pos DDot
  if(start_flag == 0 || start_flag==1)
  {
    A_L.Des_X(0) = 0;  A_L.Des_X(1) = 0.02;  A_L.Des_X(2) = -0.5;  A_L.Des_X(3) = 0;  A_L.Des_X(4) = 0;  A_L.Des_X(5) = 0;
    A_L.Des_XDot(0) = 0;  A_L.Des_XDot(1) = 0;  A_L.Des_XDot(2) = 0;  A_L.Des_XDot(3) = 0;  A_L.Des_XDot(4) = 0;  A_L.Des_XDot(5) = 0;
    A_L.Des_XDDot(0) = 0;  A_L.Des_XDDot(1) = 0;  A_L.Des_XDDot(2) = 0;  A_L.Des_XDDot(3) = 0;  A_L.Des_XDDot(4) = 0;  A_L.Des_XDDot(5) = 0;

    A_R.Des_X(0) = 0;  A_R.Des_X(1) = -0.12;  A_R.Des_X(2) = -0.5;  A_R.Des_X(3) = 0;  A_R.Des_X(4) = 0;  A_R.Des_X(5) = 0;
    A_R.Des_XDot(0) = 0;  A_R.Des_XDot(1) = 0;  A_R.Des_XDot(2) = 0;  A_R.Des_XDot(3) = 0;  A_R.Des_XDot(4) = 0;  A_R.Des_XDot(5) = 0;
    A_R.Des_XDDot(0) = 0;  A_R.Des_XDDot(1) = 0;  A_R.Des_XDDot(2) = 0;  A_R.Des_XDDot(3) = 0;  A_R.Des_XDDot(4) = 0;  A_R.Des_XDDot(5) = 0;

    A_L.Old_Des_X = A_L.Des_X; A_L.Old_Des_XDot = A_L.Des_XDot; A_L.Old_Des_XDDot = A_L.Des_XDDot;
    A_R.Old_Des_X = A_R.Des_X; A_R.Old_Des_XDot = A_R.Des_XDot; A_R.Old_Des_XDDot = A_R.Des_XDDot;
  }
  else if(start_flag == 2)
  {
    chg_step_time = 2;
    chg_cnt_time = chg_cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    chg_cnt++;
    double change_trajectory = 0.5*(1-cos(PI*(chg_cnt_time/chg_step_time)));
    if(chg_cnt_time <= chg_step_time)
    {
      A_L.Des_X = A_L.Old_Des_X + (A_L.New_Des_X - A_L.Old_Des_X)*change_trajectory;
      A_L.Des_XDot = A_L.Old_Des_XDot + (A_L.New_Des_XDot - A_L.Old_Des_XDot)*change_trajectory;
      A_L.Des_XDDot = A_L.Old_Des_XDDot + (A_L.New_Des_XDDot - A_L.Old_Des_XDDot)*change_trajectory;

      A_R.Des_X = A_R.Old_Des_X + (A_R.New_Des_X - A_R.Old_Des_X)*change_trajectory;
      A_R.Des_XDot = A_R.Old_Des_XDot + (A_R.New_Des_XDot - A_R.Old_Des_XDot)*change_trajectory;
      A_R.Des_XDDot = A_R.Old_Des_XDDot + (A_R.New_Des_XDDot - A_R.Old_Des_XDDot)*change_trajectory;
    }
    else
    {
      A_L.Des_X = A_L.New_Des_X; A_L.Des_XDot = A_L.New_Des_XDot; A_L.Des_XDDot = A_L.New_Des_XDDot;
      A_R.Des_X = A_R.New_Des_X; A_R.Des_XDot = A_R.New_Des_XDot; A_R.Des_XDDot = A_R.New_Des_XDDot;

      A_L.Old_Des_X = A_L.Des_X; A_L.Old_Des_XDot = A_L.Des_XDot; A_L.Old_Des_XDDot = A_L.Des_XDDot;
      A_R.Old_Des_X = A_R.Des_X; A_R.Old_Des_XDot = A_R.Des_XDot; A_R.Old_Des_XDDot = A_R.Des_XDDot;

      start_flag = 3;
      chg_cnt = 0;
    }
  }

  Calc_Feedback_Pos(A_L);  // calculate the feedback
  Calc_Feedback_Pos(A_R);  // calculate the feedback
  Calc_CTC_Torque(A_L);    // calculate the CTC torque
  Calc_CTC_Torque(A_R);    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = old_joint[i].torque*old_trajectory + A_L.torque_CTC(i)*new_trajectory;
      joint[i+6].torque = old_joint[i+6].torque*old_trajectory + A_R.torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = A_L.torque_CTC(i);
      joint[i+6].torque = A_R.torque_CTC(i);
      old_joint[i].torque = joint[i].torque;
      old_joint[i+6].torque = joint[i+6].torque;
    }
  }
}

void gazebo::SUBO3_plugin::CTC_Control_Cont_Pos() // 4
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
  double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  A_L.Q(0) = 0;
  A_L.Q(1) = 0;
  A_R.Q(0) = 0;
  A_R.Q(1) = 0;
  
  // Target Pos, Pos Dot, Pos DDot
  if(start_flag == 0)
  {
    A_L.Des_X(0) = 0;  A_L.Des_X(1) = 0.07;  A_L.Des_X(2) = -0.507;  A_L.Des_X(3) = 0;  A_L.Des_X(4) = 0;  A_L.Des_X(5) = 0;
    A_L.Des_XDot(0) = 0;  A_L.Des_XDot(1) = 0;  A_L.Des_XDot(2) = 0;  A_L.Des_XDot(3) = 0;  A_L.Des_XDot(4) = 0;  A_L.Des_XDot(5) = 0;
    A_L.Des_XDDot(0) = 0;  A_L.Des_XDDot(1) = 0;  A_L.Des_XDDot(2) = 0;  A_L.Des_XDDot(3) = 0;  A_L.Des_XDDot(4) = 0;  A_L.Des_XDDot(5) = 0;

    A_R.Des_X(0) = 0;  A_R.Des_X(1) = -0.07;  A_R.Des_X(2) = -0.507;  A_R.Des_X(3) = 0;  A_R.Des_X(4) = 0;  A_R.Des_X(5) = 0;
    A_R.Des_XDot(0) = 0;  A_R.Des_XDot(1) = 0;  A_R.Des_XDot(2) = 0;  A_R.Des_XDot(3) = 0;  A_R.Des_XDot(4) = 0;  A_R.Des_XDot(5) = 0;
    A_R.Des_XDDot(0) = 0;  A_R.Des_XDDot(1) = 0;  A_R.Des_XDDot(2) = 0;  A_R.Des_XDDot(3) = 0;  A_R.Des_XDDot(4) = 0;  A_R.Des_XDDot(5) = 0;

    A_L.Old_Des_X = A_L.Des_X; A_L.Old_Des_XDot = A_L.Des_XDot; A_L.Old_Des_XDDot = A_L.Des_XDDot;
    A_R.Old_Des_X = A_R.Des_X; A_R.Old_Des_XDot = A_R.Des_XDot; A_R.Old_Des_XDDot = A_R.Des_XDDot;
  }
  else if(start_flag == 1)
  {
    chg_step_time = 2;
    chg_cnt_time = chg_cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    chg_cnt++;
    double change_trajectory = 0.5*(1-cos(PI*(chg_cnt_time/chg_step_time)));
    
    A_L.New_Des_X << 0, 0.07, -0.4, 0, 0, 0;
    A_L.New_Des_XDot << 0, 0, 0, 0, 0, 0;
    A_L.New_Des_XDDot << 0, 0, 0, 0, 0, 0;
    A_R.New_Des_X << 0, -0.07, -0.55, 0, 0, 0;
    A_R.New_Des_XDot << 0, 0, 0, 0, 0, 0;
    A_R.New_Des_XDDot << 0, 0, 0, 0, 0, 0;

    if(chg_cnt_time <= chg_step_time)
    {
      A_L.Des_X = A_L.Old_Des_X + (A_L.New_Des_X - A_L.Old_Des_X)*change_trajectory;
      A_L.Des_XDot = A_L.Old_Des_XDot + (A_L.New_Des_XDot - A_L.Old_Des_XDot)*change_trajectory;
      A_L.Des_XDDot = A_L.Old_Des_XDDot + (A_L.New_Des_XDDot - A_L.Old_Des_XDDot)*change_trajectory;

      A_R.Des_X = A_R.Old_Des_X + (A_R.New_Des_X - A_R.Old_Des_X)*change_trajectory;
      A_R.Des_XDot = A_R.Old_Des_XDot + (A_R.New_Des_XDot - A_R.Old_Des_XDot)*change_trajectory;
      A_R.Des_XDDot = A_R.Old_Des_XDDot + (A_R.New_Des_XDDot - A_R.Old_Des_XDDot)*change_trajectory;
    }
    else
    {
      A_L.Des_X = A_L.New_Des_X; A_L.Des_XDot = A_L.New_Des_XDot; A_L.Des_XDDot = A_L.New_Des_XDDot;
      A_R.Des_X = A_R.New_Des_X; A_R.Des_XDot = A_R.New_Des_XDot; A_R.Des_XDDot = A_R.New_Des_XDDot;

      A_L.Old_Des_X = A_L.Des_X; A_L.Old_Des_XDot = A_L.Des_XDot; A_L.Old_Des_XDDot = A_L.Des_XDDot;
      A_R.Old_Des_X = A_R.Des_X; A_R.Old_Des_XDot = A_R.Des_XDot; A_R.Old_Des_XDDot = A_R.Des_XDDot;

      start_flag = 2;
      chg_cnt = 0;
    }
  }
  else if(start_flag == 2)
  {
    chg_step_time = 2;
    chg_cnt_time = chg_cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    chg_cnt++;
    double change_trajectory = 0.5*(1-cos(PI*(chg_cnt_time/chg_step_time)));
    
    A_L.New_Des_X << 0, 0.07, -0.55, 0, 0, 0;
    A_L.New_Des_XDot << 0, 0, 0, 0, 0, 0;
    A_L.New_Des_XDDot << 0, 0, 0, 0, 0, 0;
    A_R.New_Des_X << 0, -0.07, -0.4, 0, 0, 0;
    A_R.New_Des_XDot << 0, 0, 0, 0, 0, 0;
    A_R.New_Des_XDDot << 0, 0, 0, 0, 0, 0;
    
    if(chg_cnt_time <= chg_step_time)
    {
      A_L.Des_X = A_L.Old_Des_X + (A_L.New_Des_X - A_L.Old_Des_X)*change_trajectory;
      A_L.Des_XDot = A_L.Old_Des_XDot + (A_L.New_Des_XDot - A_L.Old_Des_XDot)*change_trajectory;
      A_L.Des_XDDot = A_L.Old_Des_XDDot + (A_L.New_Des_XDDot - A_L.Old_Des_XDDot)*change_trajectory;

      A_R.Des_X = A_R.Old_Des_X + (A_R.New_Des_X - A_R.Old_Des_X)*change_trajectory;
      A_R.Des_XDot = A_R.Old_Des_XDot + (A_R.New_Des_XDot - A_R.Old_Des_XDot)*change_trajectory;
      A_R.Des_XDDot = A_R.Old_Des_XDDot + (A_R.New_Des_XDDot - A_R.Old_Des_XDDot)*change_trajectory;
    }
    else
    {
      A_L.Des_X = A_L.New_Des_X; A_L.Des_XDot = A_L.New_Des_XDot; A_L.Des_XDDot = A_L.New_Des_XDDot;
      A_R.Des_X = A_R.New_Des_X; A_R.Des_XDot = A_R.New_Des_XDot; A_R.Des_XDDot = A_R.New_Des_XDDot;

      A_L.Old_Des_X = A_L.Des_X; A_L.Old_Des_XDot = A_L.Des_XDot; A_L.Old_Des_XDDot = A_L.Des_XDDot;
      A_R.Old_Des_X = A_R.Des_X; A_R.Old_Des_XDot = A_R.Des_XDot; A_R.Old_Des_XDDot = A_R.Des_XDDot;

      start_flag = 1;
      chg_cnt = 0;
    }
  }

  Calc_Feedback_Pos(A_L);  // calculate the feedback
  Calc_Feedback_Pos(A_R);  // calculate the feedback
  Calc_CTC_Torque(A_L);    // calculate the CTC torque
  Calc_CTC_Torque(A_R);    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = old_joint[i].torque*old_trajectory + A_L.torque_CTC(i)*new_trajectory;
      joint[i+6].torque = old_joint[i+6].torque*old_trajectory + A_R.torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = A_L.torque_CTC(i);
      joint[i+6].torque = A_R.torque_CTC(i);
      old_joint[i].torque = joint[i].torque;
      old_joint[i+6].torque = joint[i+6].torque;
    }
  }
}

void gazebo::SUBO3_plugin::GROUND_Gravity_Cont()  // 5
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;

  InverseDynamics(*G_L.rbdl_model, G_L.Q, VectorNd::Zero(6), VectorNd::Zero(6), G_L.Tau, NULL);
  InverseDynamics(*G_R.rbdl_model, G_R.Q, VectorNd::Zero(6), VectorNd::Zero(6), G_R.Tau, NULL);

  for (int i = 0; i < 6; i++)
  {
    joint[5-i].torque = G_L.Tau(i);
    joint[11-i].torque = G_R.Tau(i);
  }
}

void gazebo::SUBO3_plugin::GROUND_CTC_Control() // 6
{
  step_time = 0.5; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
  double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  // Target Pos, Pos Dot, Pos DDot
  G_L.Des_X(0) = 0;  G_L.Des_X(1) = -0.045;  G_L.Des_X(2) = 0.507;  G_L.Des_X(3) = 0;  G_L.Des_X(4) = 0;  G_L.Des_X(5) = 0;
  G_L.Des_XDot(0) = 0;  G_L.Des_XDot(1) = 0;  G_L.Des_XDot(2) = 0;  G_L.Des_XDot(3) = 0;  G_L.Des_XDot(4) = 0;  G_L.Des_XDot(5) = 0;
  G_L.Des_XDDot(0) = 0;  G_L.Des_XDDot(1) = 0;  G_L.Des_XDDot(2) = 0;  G_L.Des_XDDot(3) = 0;  G_L.Des_XDDot(4) = 0;  G_L.Des_XDDot(5) = 0;

  G_R.Des_X(0) = 0;  G_R.Des_X(1) = -0.045;  G_R.Des_X(2) = 0.507;  G_R.Des_X(3) = 0;  G_R.Des_X(4) = 0;  G_R.Des_X(5) = 0;
  G_R.Des_XDot(0) = 0;  G_R.Des_XDot(1) = 0;  G_R.Des_XDot(2) = 0;  G_R.Des_XDot(3) = 0;  G_R.Des_XDot(4) = 0;  G_R.Des_XDot(5) = 0;
  G_R.Des_XDDot(0) = 0;  G_R.Des_XDDot(1) = 0;  G_R.Des_XDDot(2) = 0;  G_R.Des_XDDot(3) = 0;  G_R.Des_XDDot(4) = 0;  G_R.Des_XDDot(5) = 0;

  Calc_Feedback_Pos(G_L);  // calculate the feedback
  Calc_Feedback_Pos(G_R);  // calculate the feedback
  Calc_CTC_Torque(G_L);    // calculate the CTC torque
  Calc_CTC_Torque(G_R);    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 6; i++)
    {
      joint[5-i].torque = old_joint[5-i].torque*old_trajectory + G_L.torque_CTC(i)*new_trajectory;
      joint[11-i].torque = old_joint[11-i].torque*old_trajectory + G_R.torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[5-i].torque = G_L.torque_CTC(i);
      joint[11-i].torque = G_R.torque_CTC(i);
      
      old_joint[5-i].torque = joint[5-i].torque;
      old_joint[11-i].torque = joint[11-i].torque;
    }
  }
}

void gazebo::SUBO3_plugin::GROUND_CTC_Control_Pos() // 7
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
  double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  // Target Pos, Pos Dot, Pos DDot
  if(start_flag == 0)
  {
    G_L.Des_X(0) = 0;  G_L.Des_X(1) = 0;  G_L.Des_X(2) = 0.507;  G_L.Des_X(3) = 0;  G_L.Des_X(4) = 0;  G_L.Des_X(5) = 0;
    G_L.Des_XDot(0) = 0;  G_L.Des_XDot(1) = 0;  G_L.Des_XDot(2) = 0;  G_L.Des_XDot(3) = 0;  G_L.Des_XDot(4) = 0;  G_L.Des_XDot(5) = 0;
    G_L.Des_XDDot(0) = 0;  G_L.Des_XDDot(1) = 0;  G_L.Des_XDDot(2) = 0;  G_L.Des_XDDot(3) = 0;  G_L.Des_XDDot(4) = 0;  G_L.Des_XDDot(5) = 0;

    G_R.Des_X(0) = 0;  G_R.Des_X(1) = 0;  G_R.Des_X(2) = 0.507;  G_R.Des_X(3) = 0;  G_R.Des_X(4) = 0;  G_R.Des_X(5) = 0;
    G_R.Des_XDot(0) = 0;  G_R.Des_XDot(1) = 0;  G_R.Des_XDot(2) = 0;  G_R.Des_XDot(3) = 0;  G_R.Des_XDot(4) = 0;  G_R.Des_XDot(5) = 0;
    G_R.Des_XDDot(0) = 0;  G_R.Des_XDDot(1) = 0;  G_R.Des_XDDot(2) = 0;  G_R.Des_XDDot(3) = 0;  G_R.Des_XDDot(4) = 0;  G_R.Des_XDDot(5) = 0;

    G_L.Old_Des_X = G_L.Des_X; G_L.Old_Des_XDot = G_L.Des_XDot; G_L.Old_Des_XDDot = G_L.Des_XDDot;
    G_R.Old_Des_X = G_R.Des_X; G_R.Old_Des_XDot = G_R.Des_XDot; G_R.Old_Des_XDDot = G_R.Des_XDDot;
  }
  else if(start_flag == 1)
  {
    chg_step_time = 2;
    chg_cnt_time = chg_cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    chg_cnt++;
    double change_trajectory = 0.5*(1-cos(PI*(chg_cnt_time/chg_step_time)));
    if(chg_cnt_time <= chg_step_time)
    {
      G_L.Des_X = G_L.Old_Des_X + (G_L.New_Des_X - G_L.Old_Des_X)*change_trajectory;
      G_L.Des_XDot = G_L.Old_Des_XDot + (G_L.New_Des_XDot - G_L.Old_Des_XDot)*change_trajectory;
      G_L.Des_XDDot = G_L.Old_Des_XDDot + (G_L.New_Des_XDDot - G_L.Old_Des_XDDot)*change_trajectory;

      G_R.Des_X = G_R.Old_Des_X + (G_R.New_Des_X - G_R.Old_Des_X)*change_trajectory;
      G_R.Des_XDot = G_R.Old_Des_XDot + (G_R.New_Des_XDot - G_R.Old_Des_XDot)*change_trajectory;
      G_R.Des_XDDot = G_R.Old_Des_XDDot + (G_R.New_Des_XDDot - G_R.Old_Des_XDDot)*change_trajectory;
    }
    else
    {
      G_L.Des_X = G_L.New_Des_X; G_L.Des_XDot = G_L.New_Des_XDot; G_L.Des_XDDot = G_L.New_Des_XDDot;
      G_R.Des_X = G_R.New_Des_X; G_R.Des_XDot = G_R.New_Des_XDot; G_R.Des_XDDot = G_R.New_Des_XDDot;

      G_L.Old_Des_X = G_L.Des_X; G_L.Old_Des_XDot = G_L.Des_XDot; G_L.Old_Des_XDDot = G_L.Des_XDDot;
      G_R.Old_Des_X = G_R.Des_X; G_R.Old_Des_XDot = G_R.Des_XDot; G_R.Old_Des_XDDot = G_R.Des_XDDot;

      start_flag = 2;
      chg_cnt = 0;
    }
  }

  Calc_Feedback_Pos(G_L);  // calculate the feedback
  Calc_Feedback_Pos(G_R);  // calculate the feedback
  Calc_CTC_Torque(G_L);    // calculate the CTC torque
  Calc_CTC_Torque(G_R);    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 6; i++)
    {
      joint[5-i].torque = old_joint[5-i].torque*old_trajectory + G_L.torque_CTC(i)*new_trajectory;
      joint[11-i].torque = old_joint[11-i].torque*old_trajectory + G_R.torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[5-i].torque = G_L.torque_CTC(i);
      joint[11-i].torque = G_R.torque_CTC(i);

      old_joint[5-i].torque = joint[5-i].torque;
      old_joint[11-i].torque = joint[11-i].torque;
    }
  }
}

void gazebo::SUBO3_plugin::GROUND_CTC_Control_Cont_Pos()  // 8
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
  double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  // Target Pos, Pos Dot, Pos DDot
  if(start_flag == 0)
  {
    G_L.Des_X(0) = 0;  G_L.Des_X(1) = 0;  G_L.Des_X(2) = 0.507;  G_L.Des_X(3) = 0;  G_L.Des_X(4) = 0;  G_L.Des_X(5) = 0;
    G_L.Des_XDot(0) = 0;  G_L.Des_XDot(1) = 0;  G_L.Des_XDot(2) = 0;  G_L.Des_XDot(3) = 0;  G_L.Des_XDot(4) = 0;  G_L.Des_XDot(5) = 0;
    G_L.Des_XDDot(0) = 0;  G_L.Des_XDDot(1) = 0;  G_L.Des_XDDot(2) = 0;  G_L.Des_XDDot(3) = 0;  G_L.Des_XDDot(4) = 0;  G_L.Des_XDDot(5) = 0;

    G_R.Des_X(0) = 0;  G_R.Des_X(1) = 0;  G_R.Des_X(2) = 0.507;  G_R.Des_X(3) = 0;  G_R.Des_X(4) = 0;  G_R.Des_X(5) = 0;
    G_R.Des_XDot(0) = 0;  G_R.Des_XDot(1) = 0;  G_R.Des_XDot(2) = 0;  G_R.Des_XDot(3) = 0;  G_R.Des_XDot(4) = 0;  G_R.Des_XDot(5) = 0;
    G_R.Des_XDDot(0) = 0;  G_R.Des_XDDot(1) = 0;  G_R.Des_XDDot(2) = 0;  G_R.Des_XDDot(3) = 0;  G_R.Des_XDDot(4) = 0;  G_R.Des_XDDot(5) = 0;

    G_L.Old_Des_X = G_L.Des_X; G_L.Old_Des_XDot = G_L.Des_XDot; G_L.Old_Des_XDDot = G_L.Des_XDDot;
    G_R.Old_Des_X = G_R.Des_X; G_R.Old_Des_XDot = G_R.Des_XDot; G_R.Old_Des_XDDot = G_R.Des_XDDot;
  }
  else if(start_flag == 1)
  {
    chg_step_time = 2;
    chg_cnt_time = chg_cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    chg_cnt++;
    double change_trajectory = 0.5*(1-cos(PI*(chg_cnt_time/chg_step_time)));

    G_L.New_Des_X << 0.01, 0.05, 0.45, 0, 0, 0;
    G_L.New_Des_XDot << 0, 0, 0, 0, 0, 0;
    G_L.New_Des_XDDot << 0, 0, 0, 0, 0, 0;
    G_R.New_Des_X << 0.01, 0.05, 0.45, 0, 0, 0;
    G_R.New_Des_XDot << 0, 0, 0, 0, 0, 0;
    G_R.New_Des_XDDot << 0, 0, 0, 0, 0, 0;

    if(chg_cnt_time <= chg_step_time)
    {
      G_L.Des_X = G_L.Old_Des_X + (G_L.New_Des_X - G_L.Old_Des_X)*change_trajectory;
      G_L.Des_XDot = G_L.Old_Des_XDot + (G_L.New_Des_XDot - G_L.Old_Des_XDot)*change_trajectory;
      G_L.Des_XDDot = G_L.Old_Des_XDDot + (G_L.New_Des_XDDot - G_L.Old_Des_XDDot)*change_trajectory;

      G_R.Des_X = G_R.Old_Des_X + (G_R.New_Des_X - G_R.Old_Des_X)*change_trajectory;
      G_R.Des_XDot = G_R.Old_Des_XDot + (G_R.New_Des_XDot - G_R.Old_Des_XDot)*change_trajectory;
      G_R.Des_XDDot = G_R.Old_Des_XDDot + (G_R.New_Des_XDDot - G_R.Old_Des_XDDot)*change_trajectory;
    }
    else
    {
      G_L.Des_X = G_L.New_Des_X; G_L.Des_XDot = G_L.New_Des_XDot; G_L.Des_XDDot = G_L.New_Des_XDDot;
      G_R.Des_X = G_R.New_Des_X; G_R.Des_XDot = G_R.New_Des_XDot; G_R.Des_XDDot = G_R.New_Des_XDDot;

      G_L.Old_Des_X = G_L.Des_X; G_L.Old_Des_XDot = G_L.Des_XDot; G_L.Old_Des_XDDot = G_L.Des_XDDot;
      G_R.Old_Des_X = G_R.Des_X; G_R.Old_Des_XDot = G_R.Des_XDot; G_R.Old_Des_XDDot = G_R.Des_XDDot;

      start_flag = 2;
      chg_cnt = 0;
    }
  }
  else if(start_flag == 2)
  {
    chg_step_time = 2;
    chg_cnt_time = chg_cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    chg_cnt++;
    double change_trajectory = 0.5*(1-cos(PI*(chg_cnt_time/chg_step_time)));
    
    G_L.New_Des_X << -0.01, -0.05, 0.45, 0, 0, 0;
    G_L.New_Des_XDot << 0, 0, 0, 0, 0, 0;
    G_L.New_Des_XDDot << 0, 0, 0, 0, 0, 0;
    G_R.New_Des_X << -0.01, -0.05, 0.45, 0, 0, 0;
    G_R.New_Des_XDot << 0, 0, 0, 0, 0, 0;
    G_R.New_Des_XDDot << 0, 0, 0, 0, 0, 0;
    
    if(chg_cnt_time <= chg_step_time)
    {
      G_L.Des_X = G_L.Old_Des_X + (G_L.New_Des_X - G_L.Old_Des_X)*change_trajectory;
      G_L.Des_XDot = G_L.Old_Des_XDot + (G_L.New_Des_XDot - G_L.Old_Des_XDot)*change_trajectory;
      G_L.Des_XDDot = G_L.Old_Des_XDDot + (G_L.New_Des_XDDot - G_L.Old_Des_XDDot)*change_trajectory;

      G_R.Des_X = G_R.Old_Des_X + (G_R.New_Des_X - G_R.Old_Des_X)*change_trajectory;
      G_R.Des_XDot = G_R.Old_Des_XDot + (G_R.New_Des_XDot - G_R.Old_Des_XDot)*change_trajectory;
      G_R.Des_XDDot = G_R.Old_Des_XDDot + (G_R.New_Des_XDDot - G_R.Old_Des_XDDot)*change_trajectory;
    }
    else
    {
      G_L.Des_X = G_L.New_Des_X; G_L.Des_XDot = G_L.New_Des_XDot; G_L.Des_XDDot = G_L.New_Des_XDDot;
      G_R.Des_X = G_R.New_Des_X; G_R.Des_XDot = G_R.New_Des_XDot; G_R.Des_XDDot = G_R.New_Des_XDDot;

      G_L.Old_Des_X = G_L.Des_X; G_L.Old_Des_XDot = G_L.Des_XDot; G_L.Old_Des_XDDot = G_L.Des_XDDot;
      G_R.Old_Des_X = G_R.Des_X; G_R.Old_Des_XDot = G_R.Des_XDot; G_R.Old_Des_XDDot = G_R.Des_XDDot;

      start_flag = 1;
      chg_cnt = 0;
    }
  }

  Calc_Feedback_Pos(G_L);  // calculate the feedback
  Calc_Feedback_Pos(G_R);  // calculate the feedback
  Calc_CTC_Torque(G_L);    // calculate the CTC torque
  Calc_CTC_Torque(G_R);    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 6; i++)
    {
      joint[5-i].torque = old_joint[5-i].torque*old_trajectory + G_L.torque_CTC(i)*new_trajectory;
      joint[11-i].torque = old_joint[11-i].torque*old_trajectory + G_R.torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[5-i].torque = G_L.torque_CTC(i);
      joint[11-i].torque = G_R.torque_CTC(i);

      old_joint[5-i].torque = joint[5-i].torque;
      old_joint[11-i].torque = joint[11-i].torque;
    }
  }
}

void gazebo::SUBO3_plugin::ONE_GROUND_CTC_Control() // 9
{
  step_time = 1; //주기설정 (초) 변수
  cnt_time = cnt*inner_dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = 0.5*(cos(PI*(cnt_time/step_time)));
  double new_trajectory = 0.5*(1-cos(PI*(cnt_time/step_time)));

  A_L.Q(0) = 0;
  A_L.Q(1) = 0;
  A_R.Q(0) = 0;
  A_R.Q(1) = 0;

  // Target Pos, Pos Dot, Pos DDot
  A_R.Des_X(0) = 0;  A_R.Des_X(1) = -0.12;  A_R.Des_X(2) = -0.4;  A_R.Des_X(3) = 0;  A_R.Des_X(4) = 0;  A_R.Des_X(5) = 0;
  A_R.Des_XDot(0) = 0;  A_R.Des_XDot(1) = 0;  A_R.Des_XDot(2) = 0;  A_R.Des_XDot(3) = 0;  A_R.Des_XDot(4) = 0;  A_R.Des_XDot(5) = 0;
  A_R.Des_XDDot(0) = 0;  A_R.Des_XDDot(1) = 0;  A_R.Des_XDDot(2) = 0;  A_R.Des_XDDot(3) = 0;  A_R.Des_XDDot(4) = 0;  A_R.Des_XDDot(5) = 0;

  O_L.Des_X(0) = 0;  O_L.Des_X(1) = -0.045;  O_L.Des_X(2) = 0.507;  O_L.Des_X(3) = 0;  O_L.Des_X(4) = 0;  O_L.Des_X(5) = 0;
  O_L.Des_XDot(0) = 0;  O_L.Des_XDot(1) = 0;  O_L.Des_XDot(2) = 0;  O_L.Des_XDot(3) = 0;  O_L.Des_XDot(4) = 0;  O_L.Des_XDot(5) = 0;
  O_L.Des_XDDot(0) = 0;  O_L.Des_XDDot(1) = 0;  O_L.Des_XDDot(2) = 0;  O_L.Des_XDDot(3) = 0;  O_L.Des_XDDot(4) = 0;  O_L.Des_XDDot(5) = 0;

  Calc_Feedback_Pos(A_R);  // calculate the feedback
  Calc_CTC_Torque(A_R);    // calculate the CTC torque

  Calc_Feedback_Pos(O_L);  // calculate the feedback
  Calc_CTC_Torque(O_L);    // calculate the CTC torque


  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 6; i++)
    {
      joint[5-i].torque = old_joint[5-i].torque*old_trajectory + O_L.torque_CTC(i)*new_trajectory;
      joint[i+6].torque = old_joint[i+6].torque*old_trajectory + A_R.torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[5-i].torque = O_L.torque_CTC(i);
      joint[i+6].torque = A_R.torque_CTC(i);
            
      old_joint[5-i].torque = joint[5-i].torque;
      old_joint[11-i].torque = joint[11-i].torque;
    }
  }
}

void gazebo::SUBO3_plugin::Print() // 한 싸이클 돌때마다 데이터 플로팅
{
  if(CONTROL_MODE != IDLE)
  {
    fprintf(tmpdata0, "%f,%f,%f,%f,%f,%f\n", joint[0].torque,joint[1].torque,joint[2].torque,joint[3].torque,joint[4].torque,joint[5].torque);
    fprintf(tmpdata1, "%f,%f,%f,%f,%f,%f\n", out_joint[0].torque,out_joint[1].torque,out_joint[2].torque,out_joint[3].torque,out_joint[4].torque,out_joint[5].torque);
    fprintf(tmpdata2, "%f,%f,%f,%f,%f,%f\n", A_L.Des_X(0),A_L.Des_X(1),A_L.Des_X(2),A_L.Des_X(3),A_L.Des_X(4),A_L.Des_X(5));
    fprintf(tmpdata3, "%f,%f,%f,%f,%f,%f\n", A_L.Foot_Pos(0),A_L.Foot_Pos(1),A_L.Foot_Pos(2),A_L.Foot_Pos(3),A_L.Foot_Pos(4),A_L.Foot_Pos(5));
    fprintf(tmpdata4, "%f,%f,%f,%f,%f,%f\n", G_L.Des_X(0),G_L.Des_X(1),G_L.Des_X(2),G_L.Des_X(3),G_L.Des_X(4),G_L.Des_X(5));
    fprintf(tmpdata5, "%f,%f,%f,%f,%f,%f\n", G_L.Foot_Pos(0),G_L.Foot_Pos(1),G_L.Foot_Pos(2),G_L.Foot_Pos(3),G_L.Foot_Pos(4),G_L.Foot_Pos(5));
    fprintf(tmpdata6, "%f,%f,%f,%f,%f,%f\n", O_L.Des_X(0),O_L.Des_X(1),O_L.Des_X(2),O_L.Des_X(3),O_L.Des_X(4),O_L.Des_X(5));
    fprintf(tmpdata7, "%f,%f,%f,%f,%f,%f\n", O_L.Foot_Pos(0),O_L.Foot_Pos(1),O_L.Foot_Pos(2),O_L.Foot_Pos(3),O_L.Foot_Pos(4),O_L.Foot_Pos(5));        
  }
}

void gazebo::SUBO3_plugin::ROSMsgPublish()
{
  //********************* Data_plot - IMU***************************//
  TmpData[0] = BODY_ImuGyro(0);
  TmpData[1] = BODY_ImuGyro(1);
  TmpData[2] = BODY_ImuGyro(2);
  TmpData[3] = BODY_ImuAcc(0);
  TmpData[4] = BODY_ImuAcc(1);
  TmpData[5] = BODY_ImuAcc(2);
  TmpData[6] = L_ImuGyro(0);
  TmpData[7] = L_ImuGyro(1);
  TmpData[8] = L_ImuGyro(2);
  TmpData[9] = L_ImuAcc(0);
  TmpData[10] = L_ImuAcc(1);
  TmpData[11] = L_ImuAcc(2);
  TmpData[12] = R_ImuGyro(0);
  TmpData[13] = R_ImuGyro(1);
  TmpData[14] = R_ImuGyro(2);
  TmpData[15] = R_ImuAcc(0);
  TmpData[16] = R_ImuAcc(1);
  TmpData[17] = R_ImuAcc(2);
  
  //********************* Data_plot - FT SENSOR***************************//
  TmpData[18] = L_Force_E(0);
  TmpData[19] = L_Force_E(1);
  TmpData[20] = L_Force_E(2);
  TmpData[21] = L_Torque_E(0);
  TmpData[22] = L_Torque_E(1);
  TmpData[23] = L_Torque_E(2);
  TmpData[24] = R_Force_E(0);
  TmpData[25] = R_Force_E(1);
  TmpData[26] = R_Force_E(2);
  TmpData[27] = R_Torque_E(0);
  TmpData[28] = R_Torque_E(1);
  TmpData[29] = R_Torque_E(2);

  //********************* Data_plot - FT SENSOR***************************//
  TmpData[30] = joint[0].torque;  // L_PELVIS_YAW_JOINT
  TmpData[31] = joint[1].torque;  // L_PELVIS_ROLL_JOINT
  TmpData[32] = joint[2].torque;  // L_PELVIS_PITCH_JOINT
  TmpData[33] = joint[3].torque;  // L_KNEE_PITCH_JOINT
  TmpData[34] = joint[4].torque;  // L_ANKLE_PITCH_JOINT
  TmpData[35] = joint[5].torque;  // L_ANKLE_ROLL_JOINT
  TmpData[36] = joint[6].torque;  // R_PELVIS_YAW_JOINT
  TmpData[37] = joint[7].torque;  // R_PELVIS_ROLL_JOINT
  TmpData[38] = joint[8].torque;  // R_PELVIS_PITCH_JOINT
  TmpData[39] = joint[9].torque;  // R_KNEE_PITCH_JOINT
  TmpData[40] = joint[10].torque; // R_ANKLE_PITCH_JOINT
  TmpData[41] = joint[11].torque; // R_ANKLE_ROLL_JOINT

  //****************** msg에 데이터 저장 *****************//
  for (unsigned int i = 0; i < 42; ++i)
  {
      m_ros_msg.data[i] = TmpData[i];
  }

  /*
  m_actual_R_Pelvis_Y_J.data = actual_joint_pos(0)*rad2deg;
  m_actual_R_Pelvis_R_J.data = actual_joint_pos(1)*rad2deg;
  m_actual_R_Pelvis_P_J.data = actual_joint_pos(2)*rad2deg;
  m_actual_R_Knee_P_J.data = actual_joint_pos(3)*rad2deg;
  m_actual_R_Ankle_P_J.data = actual_joint_pos(4)*rad2deg;
  m_actual_R_Ankle_R_J.data = actual_joint_pos(5)*rad2deg;
  m_actual_L_Pelvis_Y_J.data = actual_joint_pos(6)*rad2deg;
  m_actual_L_Pelvis_R_J.data = actual_joint_pos(7)*rad2deg;
  m_actual_L_Pelvis_P_J.data = actual_joint_pos(8)*rad2deg;
  m_actual_L_Knee_P_J.data = actual_joint_pos(9)*rad2deg;
  m_actual_L_Ankle_P_J.data = actual_joint_pos(10)*rad2deg;
  m_actual_L_Ankle_R_J.data = actual_joint_pos(11)*rad2deg;
  */
  //m_goal_R_Pelvis_Y_J.data = goal_joint_pos(0);
  //m_goal_R_Pelvis_R_J.data = goal_joint_pos(1);
  ///m_goal_R_Pelvis_P_J.data = goal_joint_pos(2);
  //m_goal_R_Knee_P_J.data = goal_joint_pos(3);
  //m_goal_R_Ankle_P_J.data = goal_joint_pos(4);
  //m_goal_R_Ankle_R_J.data = goal_joint_pos(5);
  //m_goal_L_Pelvis_Y_J.data = goal_joint_pos(6);
  //m_goal_L_Pelvis_R_J.data = goal_joint_pos(7);
  //m_goal_L_Pelvis_P_J.data = goal_joint_pos(8);
  //m_goal_L_Knee_P_J.data = goal_joint_pos(9);
  //m_goal_L_Ankle_P_J.data = goal_joint_pos(10);
  //m_goal_L_Ankle_R_J.data = goal_joint_pos(11);
  
  //******************* 퍼블리시 요청 ********************//
  /*
  P_actual_R_Pelvis_Y_J.publish(m_actual_R_Pelvis_Y_J);
  P_actual_R_Pelvis_R_J.publish(m_actual_R_Pelvis_R_J);
  P_actual_R_Pelvis_P_J.publish(m_actual_R_Pelvis_P_J);
  P_actual_R_Knee_P_J.publish(m_actual_R_Knee_P_J);
  P_actual_R_Ankle_P_J.publish(m_actual_R_Ankle_P_J);
  P_actual_R_Ankle_R_J.publish(m_actual_R_Ankle_R_J);
  P_actual_L_Pelvis_Y_J.publish(m_actual_L_Pelvis_Y_J);
  P_actual_L_Pelvis_R_J.publish(m_actual_L_Pelvis_R_J);
  P_actual_L_Pelvis_P_J.publish(m_actual_L_Pelvis_P_J);
  P_actual_L_Knee_P_J.publish(m_actual_L_Knee_P_J);
  P_actual_L_Ankle_P_J.publish(m_actual_L_Ankle_P_J);
  P_actual_L_Ankle_R_J.publish(m_actual_L_Ankle_R_J);
  */
  //P_goal_R_Pelvis_Y_J.publish(m_goal_R_Pelvis_Y_J);
  //P_goal_R_Pelvis_R_J.publish(m_goal_R_Pelvis_R_J);
  //P_goal_R_Pelvis_P_J.publish(m_goal_R_Pelvis_P_J);
  //P_goal_R_Knee_P_J.publish(m_goal_R_Knee_P_J);
  //P_goal_R_Ankle_P_J.publish(m_goal_R_Ankle_P_J);
  //P_goal_R_Ankle_R_J.publish(m_goal_R_Ankle_R_J);
  //P_goal_L_Pelvis_Y_J.publish(m_goal_L_Pelvis_Y_J);
  //P_goal_L_Pelvis_R_J.publish(m_goal_L_Pelvis_R_J);
  //P_goal_L_Pelvis_P_J.publish(m_goal_L_Pelvis_P_J);
  //P_goal_L_Knee_P_J.publish(m_goal_L_Knee_P_J);
  //P_goal_L_Ankle_P_J.publish(m_goal_L_Ankle_P_J);
  //P_goal_L_Ankle_R_J.publish(m_goal_L_Ankle_R_J);    
  
  P_ros_msg.publish(m_ros_msg);
}