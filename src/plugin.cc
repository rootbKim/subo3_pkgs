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

// ************** 
#define del_t		0.005  // Sampling time [sec]
#define	FW	120.0
#define	FL	184.0
#define L0	0.070
#define L1	0.000
#define L2	0.000
#define L3	0.250
#define L4	0.250
#define L5	0.000
#define L6	0.1105

// ************* 12DOF IK ***********************//
MatrixXd J_12DOF_T(12,12);
MatrixXd J_12DOF(12,12);
int arr_flag = 0;
// ***************행렬 선언*********************//
double Theo_RL_PR[6] = {0.,0.,0.,0.,0.,0.}, Theo_LL_PR[6] = {0.,0.,0.,0.,0.,0.}, Theo_PR[12] = {0.,};
double Act_RL_PR[6] = {0.,0.,0.,0.,0.,0.}, Act_LL_PR[6] = {0.,0.,0.,0.,0.,0.};
double Theo_RL_th[6] = {0.,0.,0.,0.,0.,0.}, Theo_LL_th[6] = {0.,0.,0.,0.,0.,0.};
double Act_RL_th[6] = {0.,0.,0.,0.,0.,0.}, Act_LL_th[6] = {0.,0.,0.,0.,0.,0.};
double RL_th[6] = {0.,0.,0.,0.,0.,0.}, LL_th[6] = {0.,0.,0.,0.,0.,0.};
double RL_th_IK[6] = {0.,0.,0.,0.,0.,0.}, LL_th_IK[6] = {0.,0.,0.,0.,0.,0.};
double Ref_RL_PR[6] = {0.,0.,0.,0.,0.,0.}, Ref_LL_PR[6] = {0.,0.,0.,0.,0.,0.};
double Trans_Ref_RL_PR[6] = {0.,0.,0.,0.,0.,0.}, Trans_Ref_LL_PR[6] = {0.,0.,0.,0.,0.,0.};
double Trans_Ref_L_PR[12] = {0.,};
double RL_T_80[4][4] = {{0.,},};
//*************** Trajectroy Variables**************//    
double step_time = 0;
double cnt_time = 0;
unsigned int cnt = 0;
double chg_step_time = 0;
double chg_cnt_time = 0;
unsigned int chg_cnt = 0;
bool home_firstRun = true;
unsigned int Up_Down_iteration_cnt=0;

////////////IK & FK Function /////////////////
void BRP_RL_IK(double Ref_RL_RP[6], double Init_th[6], double IK_th[6]);
void BRP_LL_IK(double Ref_LL_RP[6], double Init_th[6], double IK_th[6]);
void BRP_RL_FK(double th[6], double PR[6]);
void BRP_LL_FK(double th[6], double PR[6]);
void inv_mat6(int m, int n, double Mat4[][4], double Mat6[][6], double c_inv4[4][4], double c_inv[6][6]);
void inv_mat12(int m, int n, double Mat12[][12], double c_inv[12][12]);
void BCHT(double Ref_Body_Roll_deg, double Ref_Body_Pitch_deg, double Ref_Body_Yaw_deg, double Ref_RL_BC[3], double Ref_RL_BC_TF[3], double O[3]);
void BodyRotation(double Trans_Ref_PR[6], double Ref_PR[6], double o_fcp[0], double alpha, double beta, double gamma);
void Inverse_Matrix(int size, double input_matrix[12][24], double output_matrix[12][24]);
void BRP_RL2PC_FK(double th[12], double PR[12]);
void BRP_R2L_IK(double Ref_RP[12], double Init_th[12], double IK_th[12]);
double LPF_1st(double in, double* preOut, double freq_cut, double dt);
////////////12DOF IK Function /////////////////
double Ref_L_PR[12] = {0.,};
double Ref_R2L_PR[12] = {0.,};
double L_th[12] = {0.,};
double L_th_IK[12] = {0.,};    
double test_matrix[12][24] = {{0.,},{0.,},{0.,},{0.,},};
double result_matrix[12][24] = {{0.,},{0.,},{0.,},{0.,},};
//*************** 확인용 변수*********************//
unsigned int f_cnt = 0; //주기 확인용 
unsigned int c_cnt = 0; // 시행횟수

//*************** Cofactor 역행렬 변수 ***************//
float calcDeterminant(float **matrix, int n);
void cofactor(float **matrix, float **holder, int r, int c, int n);
void adjugate(float **matrix, float **adju, int n);
bool inverseMatrix(float **matrix, float **inverse, int n);

//*************** RBDL variable ***************//
VectorNd Kp, Kv;
//Left_Leg
Model* L_rbdl_model = NULL;//make model but emty
unsigned int L_w_roll_id, L_w_pitch_id, L_body_Base_id, L_body_PELVIS_1_id, L_body_PELVIS_2_id, L_body_THIGH_id, L_body_SHANK_id, L_body_FOOT_id;//id have information of the body
Body L_w_roll, L_w_pitch, L_body_Base, L_body_PELVIS_1, L_body_PELVIS_2, L_body_THIGH, L_body_SHANK, L_body_FOOT;//make body.
Joint L_joint_w_roll, L_joint_w_pitch, L_joint_Base, L_joint_PELVIS_YAW, L_joint_PELVIS_ROLL, L_joint_PELVIS_PITCH, L_joint_KNEE, L_joint_ANKLE;//make joint
Math::Matrix3d L_w_rollI, L_w_pitchI, L_bodyI_Base, L_bodyI_PELVIS_1, L_bodyI_PELVIS_2, L_bodyI_THIGH, L_bodyI_SHANK, L_bodyI_FOOT;//Inertia of Body
//declare Q -> Q is theta
VectorNd L_Q, L_QDot, L_QDDot, L_prevQ, L_prevQDot, L_Tau, L_Foot_Pos, L_Foot_Pos_dot, L_Des_X, L_Des_XDot, L_Des_XDDot, L_torque_CTC, Old_L_Des_X, Old_L_Des_XDot, Old_L_Des_XDDot, New_L_Des_X, New_L_Des_XDot, New_L_Des_XDDot;
MatrixNd L_A_Jacobian, L_prev_A_Jacobian, L_A_Jacobian_dot, Inv_L_A_Jacobian;

//Right_Leg
Model* R_rbdl_model = NULL;//make model but emty
unsigned int R_w_roll_id, R_w_pitch_id, R_body_Base_id, R_body_PELVIS_1_id, R_body_PELVIS_2_id, R_body_THIGH_id, R_body_SHANK_id, R_body_FOOT_id;//id have information of the body
Body R_w_roll, R_w_pitch, R_body_Base, R_body_PELVIS_1, R_body_PELVIS_2, R_body_THIGH, R_body_SHANK, R_body_FOOT;//make body.
Joint R_joint_w_roll, R_joint_w_pitch, R_joint_Base, R_joint_PELVIS_YAW, R_joint_PELVIS_ROLL, R_joint_PELVIS_PITCH, R_joint_KNEE, R_joint_ANKLE;//make joint
Math::Matrix3d R_w_rollI, R_w_pitchI, R_bodyI_Base, R_bodyI_PELVIS_1, R_bodyI_PELVIS_2, R_bodyI_THIGH, R_bodyI_SHANK, R_bodyI_FOOT;//Inertia of Body
//declare Q -> Q is theta
VectorNd R_Q, R_QDot, R_QDDot, R_prevQ, R_prevQDot, R_Tau, R_Foot_Pos, R_Foot_Pos_dot, R_Des_X, R_Des_XDot, R_Des_XDDot, R_torque_CTC, Old_R_Des_X, Old_R_Des_XDot, Old_R_Des_XDDot, New_R_Des_X, New_R_Des_XDot, New_R_Des_XDDot;
MatrixNd R_A_Jacobian, R_prev_A_Jacobian, R_A_Jacobian_dot, Inv_R_A_Jacobian;

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
    VectorXd preLPF_BODY_ImuGyro = VectorXd::Zero(3);
    VectorXd LPF_BODY_ImuGyro = VectorXd::Zero(3);
    VectorXd body_EAngle = VectorXd::Zero(3);
    VectorXd Avel2body_EAngleV = VectorXd::Zero(3);
    VectorXd HPF_Avel2body_EAngle = VectorXd::Zero(3);
    VectorXd Pre_HPF_Avel2body_EAngle = VectorXd::Zero(3);
    VectorXd LPF_BODY_ImuAcc = VectorXd::Zero(3);
    VectorXd preLPF_BODY_ImuAcc = VectorXd::Zero(3);
    VectorXd ccel2body_EAngle = VectorXd::Zero(3);
    VectorXd accel2body_EAngle = VectorXd::Zero(3);
    VectorXd LPF_accel2body_EAngle = VectorXd::Zero(3);
    VectorXd Pre_LPF_accel2body_EAngle = VectorXd::Zero(3);
    VectorXd accel = VectorXd::Zero(3);
    VectorXd gyro_rate = VectorXd::Zero(3);
    VectorXd imu_rate = VectorXd::Zero(3);
    double inertial_temp_3_body = 0;

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

    int start_flag = 0;

    // ************* Structure variables ****************//
    ENDPOINT L_LEG, R_LEG;
    JOINT* joint;
    JOINT* old_joint;

    enum ControlMode
    {
      IDLE = 0,
      GRAVITY_CONTROL,
      CTC_CONTROL,
      CTC_CONTROL_POS,
    };
    
    enum ControlMode CONTROL_MODE;

    // ************* Functions ****************//
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
    void UpdateAlgorithm();
    void RBDL_INIT();
    void GetLinks();
    void GetJoints();
    void InitROSPubSetting();
    void SensorSetting();
    void IMUSensorRead();
    void FTSensorRead();
    void EncoderRead();
    void jointController();
    void ROSMsgPublish();
    void Callback1(const std_msgs::Int32Ptr &msg);
    void msgCallback(const subo3_pkgs::CTCMsg::ConstPtr &msg);
    void FTsensorTransformation();

    void PostureGeneration();

    void Calc_Feedback_Pos();
    void Calc_CTC_Torque();
    void CalcBodyAngle();
    void Init_Pos_Traj();
    void Gravity_Cont();
    void CTC_Control();
    void CTC_Control_Pos();

    VectorXd FK(VectorXd joint_pos_HS);
    VectorXd IK(VectorXd EP_pos);

    FILE* tmpdata0=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata0.txt","w");
    FILE* tmpdata1=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata1.txt","w");
    FILE* tmpdata2=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata2.txt","w");
    FILE* tmpdata3=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata3.txt","w");
    FILE* tmpdata4=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata4.txt","w");
    FILE* tmpdata5=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata5.txt","w");
    // FILE* tmpdata6=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata6.txt","w");
    // FILE* tmpdata7=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata7.txt","w");
    // FILE* tmpdata8=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata8.txt","w");
    // FILE* tmpdata9=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata9.txt","w");
    // FILE* tmpdata10=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata10.txt","w");
    // FILE* tmpdata11=fopen("/home/jiyong/catkin_ws/src/subo3_pkgs/MATLAB/tmpdata11.txt","w");

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
  CONTROL_MODE = IDLE;

  this->last_update_time = this->model->GetWorld()->GetSimTime();
  this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SUBO3_plugin::UpdateAlgorithm, this));
  std::cout << "Load..." << std::endl;
}

void gazebo::SUBO3_plugin::RBDL_INIT()
{
  //********************RBDL********************//
  rbdl_check_api_version(RBDL_API_VERSION);//check the rdbl version

  Kp = VectorNd::Zero(6);
  Kv = VectorNd::Zero(6);

  //RBDL_MODEL
  L_rbdl_model = new Model();//declare Model
  L_rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);//set gravity
  R_rbdl_model = new Model();//declare Model
  R_rbdl_model->gravity = Math::Vector3d(0., 0., -9.81);//set gravity

  
  L_w_rollI = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
	L_w_roll = Body(0, Math::Vector3d(0, 0, 0), L_w_rollI);
	L_joint_w_roll = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
	L_w_roll_id = L_rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, 0, 0)), L_joint_w_roll, L_w_roll);

  L_w_pitchI = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
	L_w_pitch = Body(0, Math::Vector3d(0, 0, 0), L_w_pitchI);
	L_joint_w_pitch = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
	L_w_pitch_id = L_rbdl_model->Model::AddBody(L_w_roll_id, Xtrans(Math::Vector3d(0, 0, 0)), L_joint_w_pitch, L_w_pitch);

  L_bodyI_Base = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
	L_body_Base = Body(0, Math::Vector3d(0, 0, 0), L_bodyI_Base);
	L_joint_Base = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 0));
	L_body_Base_id = L_rbdl_model->Model::AddBody(L_w_pitch_id, Xtrans(Math::Vector3d(0, 0, 0)), L_joint_Base, L_body_Base);

  R_w_rollI = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
	R_w_roll = Body(0, Math::Vector3d(0, 0, 0), R_w_rollI);
	R_joint_w_roll = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0));
	R_w_roll_id = R_rbdl_model->Model::AddBody(0, Xtrans(Math::Vector3d(0, 0, 0)), R_joint_w_roll, R_w_roll);

  R_w_pitchI = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
	R_w_pitch = Body(0, Math::Vector3d(0, 0, 0), R_w_pitchI);
	R_joint_w_pitch = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
	R_w_pitch_id = R_rbdl_model->Model::AddBody(R_w_roll_id, Xtrans(Math::Vector3d(0, 0, 0)), R_joint_w_pitch, R_w_pitch);

  R_bodyI_Base = Math::Matrix3d(0,0,0,0,0,0,0,0,0);
	R_body_Base = Body(0, Math::Vector3d(0, 0, 0), R_bodyI_Base);
	R_joint_Base = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 0));
	R_body_Base_id = R_rbdl_model->Model::AddBody(R_w_pitch_id, Xtrans(Math::Vector3d(0, 0, 0)), R_joint_Base, R_body_Base);

  //********************LEFT_LEG********************//
  //Inertia of Body
  L_bodyI_PELVIS_1 = Math::Matrix3d(0.000419,-0.000005,0.000133, -0.000005,0.001001,-0.000018, 0.000133,-0.000018,0.000820);
  L_bodyI_PELVIS_2 = Math::Matrix3d(0.000262,-0.000050,0.000001, -0.000050,0.000727,-0.000002, 0.000001,-0.000002,0.000757);
  L_bodyI_THIGH = Math::Matrix3d(0.015886,-0.000021,-0.000012, -0.000021,0.014439,-0.000761, -0.000012,-0.000761,0.002793);
  L_bodyI_SHANK = Math::Matrix3d(0.018454,0.000148,0.001029, 0.000148,0.017892,-0.000337, 0.001029,-0.000337,0.003522);
  L_bodyI_FOOT = Math::Matrix3d(0.001594,0.000050,-0.000455, 0.000050,0.003143,-0.000103, -0.000455,-0.000103,0.002851);

  //set_body_PELVIS_1
	L_body_PELVIS_1 = Body(0.530103, Math::Vector3d(-0.026622, -0.006227, -0.028069), L_bodyI_PELVIS_1);
	L_joint_PELVIS_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1)); // pelvis yaw
	L_body_PELVIS_1_id = L_rbdl_model->Model::AddBody(L_body_Base_id, Xtrans(Math::Vector3d(0, 0.07, 0)), L_joint_PELVIS_YAW, L_body_PELVIS_1);
	
  //set_body_PELVIS2
	L_body_PELVIS_2 = Body(0.565726, Math::Vector3d(-0.029274, -0.002813, -0.000043), L_bodyI_PELVIS_2);
	L_joint_PELVIS_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0)); // pelvis roll
	L_body_PELVIS_2_id = L_rbdl_model->Model::AddBody(L_body_PELVIS_1_id, Xtrans(Math::Vector3d(-0.00025, 0, -0.074)), L_joint_PELVIS_ROLL, L_body_PELVIS_2);

  //set_body_THIGH
	L_body_THIGH = Body(1.897, Math::Vector3d(0.003474, 0.02006, -0.16412), L_bodyI_THIGH);
	L_joint_PELVIS_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0)); // pelvis pitch
	L_body_THIGH_id = L_rbdl_model->Model::AddBody(L_body_PELVIS_2_id, Xtrans(Math::Vector3d(0, 0, 0)), L_joint_PELVIS_PITCH, L_body_THIGH);
  
	//set_body_SHANK
  L_body_SHANK = Body(2.2, Math::Vector3d(-0.007394, 0.00838, -0.18208), L_bodyI_SHANK);
	L_joint_KNEE = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
  L_body_SHANK_id = L_rbdl_model->Model::AddBody(L_body_THIGH_id, Xtrans(Math::Vector3d(-0.00013, 0, -0.25)), L_joint_KNEE, L_body_SHANK);

	//set_body_FOOT
  L_body_FOOT = Body(0.854, Math::Vector3d(0.012113, 0.004647, -0.068225), L_bodyI_FOOT);
	L_joint_ANKLE = Joint(SpatialVector(0, 1, 0, 0, 0, 0), SpatialVector(1, 0, 0, 0, 0, 0));
	L_body_FOOT_id = L_rbdl_model->Model::AddBody(L_body_SHANK_id, Xtrans(Math::Vector3d(0, 0, -0.25)), L_joint_ANKLE, L_body_FOOT);

  //set Q, QDot, QDDot, prevQ, prevQDot // dof_count = num of degree
	L_Q = VectorNd::Zero(L_rbdl_model->dof_count);
	L_QDot = VectorNd::Zero(L_rbdl_model->dof_count);
	L_QDDot = VectorNd::Zero(L_rbdl_model->dof_count);
	L_prevQ = VectorNd::Zero(L_rbdl_model->dof_count);
	L_prevQDot = VectorNd::Zero(L_rbdl_model->dof_count);
  L_Tau = VectorNd::Zero(L_rbdl_model->dof_count);
  L_Foot_Pos = VectorNd::Zero(6);
  L_Foot_Pos_dot = VectorNd::Zero(6);
  L_A_Jacobian = MatrixNd::Zero(6,6);
  L_prev_A_Jacobian = MatrixNd::Zero(6,6);
  L_A_Jacobian_dot = MatrixNd::Zero(6,6);
  Inv_L_A_Jacobian = MatrixNd::Zero(6,6);
  L_Des_X = VectorNd::Zero(6);
  L_Des_XDot = VectorNd::Zero(6);
  L_Des_XDDot = VectorNd::Zero(6);
  L_torque_CTC = VectorNd::Zero(6);
  Old_L_Des_X = VectorNd::Zero(6);
  Old_L_Des_XDot = VectorNd::Zero(6);
  Old_L_Des_XDDot = VectorNd::Zero(6);
  New_L_Des_X = VectorNd::Zero(6);
  New_L_Des_XDot = VectorNd::Zero(6);
  New_L_Des_XDDot = VectorNd::Zero(6);

  //init Q
  L_Q(0) = 0, L_prevQ(0) = 0, L_prevQDot(0) = 0;  // base roll
  L_Q(1) = 0, L_prevQ(1) = 0, L_prevQDot(1) = 0;  // base pitch
  L_Q(2) = 0, L_prevQ(2) = 0, L_prevQDot(2) = 0;
  L_Q(3) = 0, L_prevQ(3) = 0, L_prevQDot(3) = 0;  // pelvis yaw
  L_Q(4) = 0, L_prevQ(4) = 0, L_prevQDot(4) = 0;  // pelvis roll
  L_Q(5) = 0, L_prevQ(5) = 0, L_prevQDot(5) = 0;  // pelvis pitch
  L_Q(6) = 0, L_prevQ(6) = 0, L_prevQDot(6) = 0;  // knee pitch
  L_Q(7) = 0, L_prevQ(7) = 0, L_prevQDot(7) = 0;  // ankle pitch
  L_Q(8) = 0, L_prevQ(8) = 0, L_prevQDot(8) = 0;  // ankle roll

  //********************RIGHT_LEG********************//
  //Inertia of Body
  R_bodyI_PELVIS_1 = Math::Matrix3d(0.000419,0.000005,0.000133, 0.000005,0.001001,0.000018, 0.000133,0.000018,0.000820);
  R_bodyI_PELVIS_2 = Math::Matrix3d(0.000262,0.000050,0.000001, 0.000050,0.000727,0.000002, 0.000001,0.000002,0.000757);
  R_bodyI_THIGH = Math::Matrix3d(0.015886,0.000021,-0.000012, 0.000021,0.014439,0.000761, -0.000012,0.000761,0.002793);
  R_bodyI_SHANK = Math::Matrix3d(0.018454,-0.000148,0.001029, -0.000148,0.017892,0.000337, 0.001029,0.000337,0.003522);
  R_bodyI_FOOT = Math::Matrix3d(0.001594,-0.000050,-0.000455, -0.000050,0.003143,0.000103, -0.000455,0.000103,0.002851);
  
  //set_body_PELVIS_1
	R_body_PELVIS_1 = Body(0.530103, Math::Vector3d(-0.026622, 0.006227, -0.028069), R_bodyI_PELVIS_1);
	R_joint_PELVIS_YAW = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 0, 1)); // pelvis yaw
	R_body_PELVIS_1_id = R_rbdl_model->Model::AddBody(R_body_Base_id, Xtrans(Math::Vector3d(0, -0.07, 0)), R_joint_PELVIS_YAW, R_body_PELVIS_1);
	
  //set_body_PELVIS2
	R_body_PELVIS_2 = Body(0.565726, Math::Vector3d(-0.029274, 0.002813, -0.000043), R_bodyI_PELVIS_2);
	R_joint_PELVIS_ROLL = Joint(JointType::JointTypeRevolute, Math::Vector3d(1, 0, 0)); // pelvis roll
	R_body_PELVIS_2_id = R_rbdl_model->Model::AddBody(R_body_PELVIS_1_id, Xtrans(Math::Vector3d(-0.00025, 0, -0.074)), R_joint_PELVIS_ROLL, R_body_PELVIS_2);

	//set_body_THIGH
	R_body_THIGH = Body(1.897, Math::Vector3d(0.00335, -0.02006, -0.16411), R_bodyI_THIGH);
	R_joint_PELVIS_PITCH = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0)); //pelvis pitch
	R_body_THIGH_id = R_rbdl_model->Model::AddBody(R_body_PELVIS_2_id, Xtrans(Math::Vector3d(0, 0, 0)), R_joint_PELVIS_PITCH, R_body_THIGH);

	//set_body_SHANK
  R_body_SHANK = Body(2.2, Math::Vector3d(-0.00742, -0.00838, -0.18203), R_bodyI_SHANK);
	R_joint_KNEE = Joint(JointType::JointTypeRevolute, Math::Vector3d(0, 1, 0));
  R_body_SHANK_id = R_rbdl_model->Model::AddBody(R_body_THIGH_id, Xtrans(Math::Vector3d(-0.00013, 0, -0.25)), R_joint_KNEE, R_body_SHANK);

	//set_body_FOOT
  R_body_FOOT = Body(0.854, Math::Vector3d(0.01206, -0.00475, -0.068478), R_bodyI_FOOT);
	R_joint_ANKLE = Joint(SpatialVector(0, 1, 0, 0, 0, 0), SpatialVector(1, 0, 0, 0, 0, 0));
	R_body_FOOT_id = R_rbdl_model->Model::AddBody(R_body_SHANK_id, Xtrans(Math::Vector3d(0, 0, -0.25)), R_joint_ANKLE, R_body_FOOT);

  //set Q, QDot, QDDot, prevQ, prevQDot // dof_count = num of degree
	R_Q = VectorNd::Zero(R_rbdl_model->dof_count);
	R_QDot = VectorNd::Zero(R_rbdl_model->dof_count);
	R_QDDot = VectorNd::Zero(R_rbdl_model->dof_count);
	R_prevQ = VectorNd::Zero(R_rbdl_model->dof_count);
	R_prevQDot = VectorNd::Zero(R_rbdl_model->dof_count);
  R_Tau = VectorNd::Zero(R_rbdl_model->dof_count);
  R_Foot_Pos = VectorNd::Zero(6);
  R_Foot_Pos_dot = VectorNd::Zero(6);
  R_A_Jacobian = MatrixNd::Zero(6,6);
  R_prev_A_Jacobian = MatrixNd::Zero(6,6);
  R_A_Jacobian_dot = MatrixNd::Zero(6,6);
  Inv_R_A_Jacobian = MatrixNd::Zero(6,6);
  R_Des_X = VectorNd::Zero(6);
  R_Des_XDot = VectorNd::Zero(6);
  R_Des_XDDot = VectorNd::Zero(6);
  R_torque_CTC = VectorNd::Zero(6);
  Old_R_Des_X = VectorNd::Zero(6);
  Old_R_Des_XDot = VectorNd::Zero(6);
  Old_R_Des_XDDot = VectorNd::Zero(6);
  New_R_Des_X = VectorNd::Zero(6);
  New_R_Des_XDot = VectorNd::Zero(6);
  New_R_Des_XDDot = VectorNd::Zero(6); 

  //init Q
  R_Q(0) = 0, R_prevQ(0) = 0, R_prevQDot(0) = 0;  // base roll
  R_Q(1) = 0, R_prevQ(1) = 0, R_prevQDot(1) = 0;  // base pitch
  R_Q(2) = 0, R_prevQ(2) = 0, R_prevQDot(2) = 0;
  R_Q(3) = 0, R_prevQ(3) = 0, R_prevQDot(3) = 0;  // pelvis yaw
  R_Q(4) = 0, R_prevQ(4) = 0, R_prevQDot(4) = 0;  // pelvis roll
  R_Q(5) = 0, R_prevQ(5) = 0, R_prevQDot(5) = 0;  // pelvis pitch
  R_Q(6) = 0, R_prevQ(6) = 0, R_prevQDot(6) = 0;  // knee pitch
  R_Q(7) = 0, R_prevQ(7) = 0, R_prevQDot(7) = 0;  // ankle pitch
  R_Q(8) = 0, R_prevQ(8) = 0, R_prevQDot(8) = 0;  // ankle roll
}

void gazebo::SUBO3_plugin::UpdateAlgorithm() // 여러번 실행되는 함수
{
  //************************** Time ********************************//
  current_time = this->model->GetWorld()->GetSimTime();
  dt = current_time.Double() - this->last_update_time.Double();

  IMUSensorRead();
  FTSensorRead();
  EncoderRead(); //FK 푸는것도 포함.
  PostureGeneration(); // PostureGeneration 하위에 Trajectory 하위에 IK푸는것 포함.
  jointController();

  f_cnt++;
  this->last_update_time = current_time;

  ROSMsgPublish();
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
  FTsensorTransformation();
  
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
  L_Force_I = L_LEG.T_matrix*L_Force_E;
  
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
  R_Force_I = R_LEG.T_matrix*R_Force_E;
}

void gazebo::SUBO3_plugin::FTsensorTransformation()
{
  MatrixXd L_PELVIS_YAW(3, 3), L_PELVIS_ROLL(3, 3), L_PELVIS_PITCH(3, 3), L_KNEE_PITCH(3, 3), L_ANKLE_PITCH(3, 3), L_ANKLE_ROLL(3, 3);
  MatrixXd R_PELVIS_YAW(3, 3), R_PELVIS_ROLL(3, 3), R_PELVIS_PITCH(3, 3), R_KNEE_PITCH(3, 3), R_ANKLE_PITCH(3, 3), R_ANKLE_ROLL(3, 3);
  
  L_PELVIS_YAW << cos(actual_joint_pos[0]), -sin(actual_joint_pos[0]), 0, sin(actual_joint_pos[0]), cos(actual_joint_pos[0]), 0, 0, 0, 1;
  L_PELVIS_ROLL << 1, 0, 0, 0, cos(actual_joint_pos[1]), -sin(actual_joint_pos[1]), 0, sin(actual_joint_pos[1]), cos(actual_joint_pos[1]);
  L_PELVIS_PITCH << cos(actual_joint_pos[2]), 0, sin(actual_joint_pos[2]), 0, 1, 0, -sin(actual_joint_pos[2]), 0, cos(actual_joint_pos[2]);
  L_KNEE_PITCH << cos(actual_joint_pos[3]), 0, sin(actual_joint_pos[3]), 0, 1, 0, -sin(actual_joint_pos[3]), 0, cos(actual_joint_pos[3]);
  L_ANKLE_PITCH << cos(actual_joint_pos[4]), 0, sin(actual_joint_pos[4]), 0, 1, 0, -sin(actual_joint_pos[4]), 0, cos(actual_joint_pos[4]);
  L_ANKLE_ROLL << 1, 0, 0, 0, cos(actual_joint_pos[5]), -sin(actual_joint_pos[5]), 0, sin(actual_joint_pos[5]), cos(actual_joint_pos[5]);
  L_LEG.T_matrix = L_PELVIS_YAW * L_PELVIS_ROLL * L_PELVIS_PITCH * L_KNEE_PITCH * L_ANKLE_PITCH * L_ANKLE_ROLL;
  Left_LEG.T_matrix = L_PELVIS_YAW * L_PELVIS_ROLL * L_PELVIS_PITCH * L_KNEE_PITCH * L_ANKLE_PITCH * L_ANKLE_ROLL;

  R_PELVIS_YAW << cos(actual_joint_pos[6]), -sin(actual_joint_pos[6]), 0, sin(actual_joint_pos[6]), cos(actual_joint_pos[6]), 0, 0, 0, 1;
  R_PELVIS_ROLL << 1, 0, 0, 0, cos(actual_joint_pos[7]), -sin(actual_joint_pos[7]), 0, sin(actual_joint_pos[7]), cos(actual_joint_pos[7]);
  R_PELVIS_PITCH << cos(actual_joint_pos[8]), 0, sin(actual_joint_pos[8]), 0, 1, 0, -sin(actual_joint_pos[8]), 0, cos(actual_joint_pos[8]);
  R_KNEE_PITCH << cos(actual_joint_pos[9]), 0, sin(actual_joint_pos[9]), 0, 1, 0, -sin(actual_joint_pos[9]), 0, cos(actual_joint_pos[9]);
  R_ANKLE_PITCH << cos(actual_joint_pos[10]), 0, sin(actual_joint_pos[10]), 0, 1, 0, -sin(actual_joint_pos[10]), 0, cos(actual_joint_pos[10]);
  R_ANKLE_ROLL << 1, 0, 0, 0, cos(actual_joint_pos[11]), -sin(actual_joint_pos[11]), 0, sin(actual_joint_pos[11]), cos(actual_joint_pos[11]);
  R_LEG.T_matrix = R_PELVIS_YAW * R_PELVIS_ROLL * R_PELVIS_PITCH * R_KNEE_PITCH * R_ANKLE_PITCH * R_ANKLE_ROLL;
  Right_LEG.T_matrix = R_PELVIS_YAW * R_PELVIS_ROLL * R_PELVIS_PITCH * R_KNEE_PITCH * R_ANKLE_PITCH * R_ANKLE_ROLL;
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
    actual_joint_vel[i] = (actual_joint_pos[i] - pre_actual_joint_pos[i]) / dt;
    pre_actual_joint_pos[i] = actual_joint_pos[i];
  }
  for (int i = 0; i < 6; i++)
  {
    Act_LL_th[i] = double(actual_joint_pos[i]);
    Act_RL_th[i] = double(actual_joint_pos[i+6]);
  }
}

void BRP_RL_IK(double Ref_RL_RP[6], double Init_th[6], double IK_th[6])  // (2015_07_29)
{
	int iter,i,j,k;
	double th[6] = {0.,0.,0.,0.,0.,0.}, PR[6] = {0.,0.,0.,0.,0.,0.}, old_PR[6] = {0.,0.,0.,0.,0.,0.}, F[6] = {0.,0.,0.,0.,0.,0.}, old_Q[6] = {0.,0.,0.,0.,0.,0.}, 
			New_Q4J[6] = {0.,0.,0.,0.,0.,0.}, ERR = 0., sum = 0.,
			New_PR[6][6] = {{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.}}, 
			J[6][6] = {{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.}},
			Inv_J[6][6] = {{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.}};
	const double del_Q = 0.0001;
	
	// Initial Joint angles //
	for (i=0; i < 6; i++) th[i] = Init_th[i];
	
	for (iter = 0; iter < 100; iter++)
  {
    for (i=0; i < 6; i++)	old_Q[i] = th[i];

    // Forward Kinematics
    BRP_RL_FK(th, PR);

    // Error calculation
    for (i=0; i < 6; i++) F[i] = Ref_RL_RP[i] - PR[i];
	
    ERR = sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5]);

    if(iter == 5)
    {
      
    }

    if (ERR < 0.0000001)
    { 
      for(i=0; i < 6; i++)	IK_th[i] = th[i];
      std::cout << "iter = " << iter << std::endl;
      break;
    }
    else if(iter == 99)
    {
      for(i=0; i < 6; i++)	IK_th[i] = Init_th[i];
      std::cout << "iter = 99" << std::endl;
      break;
    }

    // Jacobian Cacluation using perturbation //
    for(i=0; i<6; i++) old_PR[i] = PR[i];

    for(i=0; i<6; i++)
    {
    
      for(j=0; j<6; j++) New_Q4J[j] = old_Q[j];  // Reset

      New_Q4J[i] = old_Q[i] + del_Q; // Perturb

      // Forward Kinematics again //
      BRP_RL_FK(New_Q4J, PR);
  
      for(j = 0; j<6; j++) New_PR[j][i] = PR[j];
    } // End of for(i=0; i<6; i++){

    for(i = 0; i<6; i++)
      for(j = 0; j<6; j++)
        J[i][j] = (New_PR[i][j] - old_PR[i])/del_Q;

    inv_mat6(0,6,0,J,0,Inv_J);
   
    for(k=0; k < 6; k++)
    {
      sum = 0.;
      for(j=0; j < 6;j++)
      {
        sum = sum + Inv_J[k][j]*F[j];
      }
      
      th[k] = old_Q[k] + sum;
    }

    if (th[3] < 0)
    {
      th[3] = -th[3];
    }
	} // End of for (iter = 0; iter <= 100; iter++) {
}

void BRP_LL_IK(double Ref_LL_RP[6], double Init_th[6], double IK_th[6])  // (2015_07_29)
{
	int iter,i,j,k;
	double th[6] = {0.,0.,0.,0.,0.,0.}, PR[6] = {0.,0.,0.,0.,0.,0.}, old_PR[6] = {0.,0.,0.,0.,0.,0.}, F[6] = {0.,0.,0.,0.,0.,0.}, old_Q[6] = {0.,0.,0.,0.,0.,0.},
			New_Q4J[6] = {0.,0.,0.,0.,0.,0.}, ERR = 0., sum = 0.,
			New_PR[6][6] = {{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.}}, 
			J[6][6] = {{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.}},
			Inv_J[6][6] = {{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.},{0.,0.,0.,0.,0.,0.}};
	const double del_Q = 0.0001;
	
	// Initial Joint angles //
	for (i=0; i < 6; i++) th[i] = Init_th[i];
	
	for (iter = 0; iter < 100; iter++)
  {
		for (i=0; i < 6; i++)	old_Q[i] = th[i];

    // Forward Kinematics
    BRP_LL_FK(th, PR);

    // Error calculation
    for (i=0; i < 6; i++) F[i] = Ref_LL_RP[i] - PR[i];

    ERR = sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5]);
    if(iter == 5)
    {
      //std::cout << "LL_ERR = " << ERR << std::endl;
    }

    if (ERR < 0.0000001) { 
      for(i=0; i < 6; i++)	IK_th[i] = th[i];
      break;
    }
    else if(iter == 99){
      for(i=0; i < 6; i++)	IK_th[i] = Init_th[i];
      std::cout << "iter = 99" << std::endl;
      break;
    }

    // Jacobian Cacluation using perturbation //
    for(i=0; i<6; i++) old_PR[i] = PR[i];

    for(i=0; i<6; i++)
    {
      for(j=0; j<6; j++) New_Q4J[j] = old_Q[j];  // Reset

      New_Q4J[i] = old_Q[i] + del_Q; // Perturb

      // Forward Kinematics again //
      BRP_LL_FK(New_Q4J, PR);
  
      for(j = 0; j<6; j++) New_PR[j][i] = PR[j];
    } // End of for(i=0; i<6; i++){

    for(i = 0; i<6; i++)
      for(j = 0; j<6; j++)
        J[i][j] = (New_PR[i][j] - old_PR[i])/del_Q;

    inv_mat6(0,6,0,J,0,Inv_J);

    for(k=0; k < 6; k++)
    {
      sum = 0.;
      for(j=0; j < 6;j++)
      {
        sum = sum + Inv_J[k][j]*F[j];
      }

      th[k] = old_Q[k] + sum;
    }

    if (th[3] < 0)
    {
      th[3] = -th[3];
    }
	} // End of for (iter = 0; iter <= 100; iter++) {
}

void BRP_R2L_IK(double Ref_RP[12], double Init_th[12], double IK_th[12])  // 12DOF IK푸는식
{
	int iter,i,j,k;
	double th[12] = {0.,},th_R[6] = {0.,}, th_L[6] = {0.,}, PR[12] = {0.,}, old_PR[12] = {0.,}, F[12] = {0.,}, old_Q[12] = {0.,},
			New_Q4J[12] = {0.,},New_Q4J_R[6] = {0.,},New_Q4J_L[6] = {0.,}, ERR = 0., sum = 0.,
			New_PR[12][12] = {{0.,},{0.,},{0.,},{0.,},{0.,},{0.,}},
			J[12][12] = {{0.,},},
			Inv_J[12][12] = {{0.,},};

	MatrixXd eigen_J = MatrixXd::Zero(12,12);
	MatrixXd eigen_J_Inv = MatrixXd::Zero(12,12);
	MatrixXd eigen_Inv_J = MatrixXd::Zero(12,12);
	MatrixXd PC2RF_RPY = MatrixXd::Zero(3,3);
	MatrixXd RF2PC_RPY = MatrixXd::Zero(3,3);
	MatrixXd PC2LF_RPY = MatrixXd::Zero(3,3);
	MatrixXd RF2LF_RPY = MatrixXd::Zero(3,3);
	VectorXd Ref_PR_eigen = VectorXd::Zero(12);
	VectorXd Temp = VectorXd::Zero(3);

	double RF2PC_PR[6] = {0.,}, RF2LF_PR[6]={0.,};
	double nx,ny,nz,ox,oy,oz,ax,ay,az;

	const double del_Q = 0.0001; // 0.0001
	
  // Initial Joint angles //
	for (int i=0; i < 12; i++) th[i] = Init_th[i]; 
	
	for (int iter = 0; iter < 100; iter++) //iter<100
  {
		//c_cnt++;
		//if(c_cnt > 3) break;
		for (int i=0; i < 12; i++)	old_Q[i] = th[i];
		
    // Forward Kinematics
		//for(int j=0; j<12; j++) std::cout<<"th["<<j<<"] : "<<th[j]<<endl;
		BRP_RL2PC_FK(th,PR);
		//for(int j=0; j<12; j++) std::cout<<"PR["<<j<<"] : "<<PR[j]<<endl;
		
    //오른발 좌표계에서 바라본 골반 좌표계의 참조 롤/피치/요 회전 행렬
    PC2RF_RPY << cos(Ref_RP[5])*cos(Ref_RP[4]), -sin(Ref_RP[5])*cos(Ref_RP[3])+cos(Ref_RP[5])*sin(Ref_RP[4])*sin(Ref_RP[3]), sin(Ref_RP[5])*sin(Ref_RP[3])+cos(Ref_RP[5])*sin(Ref_RP[4])*cos(Ref_RP[3]),
    		sin(Ref_RP[5])*cos(Ref_RP[4]), cos(Ref_RP[5])*cos(Ref_RP[3])+sin(Ref_RP[5])*sin(Ref_RP[4])*sin(Ref_RP[3]), -cos(Ref_RP[5])*sin(Ref_RP[3])+sin(Ref_RP[5])*sin(Ref_RP[4])*cos(Ref_RP[3]),
		    -sin(Ref_RP[4]), cos(Ref_RP[4])*sin(Ref_RP[3]), cos(Ref_RP[4])*cos(Ref_RP[3]); 

		RF2PC_RPY = PC2RF_RPY.inverse();
		
		//std::cout<<"RF2PC_RPY"<<std::endl;
    //std::cout<<RF2PC_RPY<<std::endl;

    //골반 좌표계에서 바라본 왼발 좌표계의 참조 롤/피치/요 회전 행렬
    PC2LF_RPY << cos(Ref_RP[11])*cos(Ref_RP[10]), -sin(Ref_RP[11])*cos(Ref_RP[9])+cos(Ref_RP[11])*sin(Ref_RP[10])*sin(Ref_RP[9]), sin(Ref_RP[11])*sin(Ref_RP[9])+cos(Ref_RP[11])*sin(Ref_RP[10])*cos(Ref_RP[9]),
		    sin(Ref_RP[11])*cos(Ref_RP[10]), cos(Ref_RP[11])*cos(Ref_RP[9])+sin(Ref_RP[11])*sin(Ref_RP[10])*sin(Ref_RP[9]), -cos(Ref_RP[11])*sin(Ref_RP[9])+sin(Ref_RP[11])*sin(Ref_RP[10])*cos(Ref_RP[9]),
		    -sin(Ref_RP[10]), cos(Ref_RP[10])*sin(Ref_RP[9]), cos(Ref_RP[10])*cos(Ref_RP[9]);
	
		//std::cout<<"PC2LF_RPY"<<std::endl;
    //std::cout<<PC2LF_RPY<<std::endl;
	
		//오른발 좌표계에서 바라본 골반 좌표계의 참조 롤/피치/요 오일러 각도 계산
    nx = RF2PC_RPY(0,0); ny = RF2PC_RPY(1,0); nz = RF2PC_RPY(2,0); 
		ox = RF2PC_RPY(0,1); oy = RF2PC_RPY(1,1); oz = RF2PC_RPY(2,1);
    ax = RF2PC_RPY(0,2); ay = RF2PC_RPY(1,2); az = RF2PC_RPY(2,2);
		
    RF2PC_PR[5] = atan2(ny,nx);  // FI : Roll about z0 axis
    RF2PC_PR[4] = atan2(-nz,cos(RF2PC_PR[5])*nx + sin(RF2PC_PR[5])*ny);  // theta : Pitch about y0 axis
    RF2PC_PR[3] = atan2(sin(RF2PC_PR[5])*ax - cos(RF2PC_PR[5])*ay, -sin(RF2PC_PR[5])*ox + cos(RF2PC_PR[5])*oy) ; // csi : Yaw about x0 axis

		//오른발 좌표계에서 바라본 왼발 좌표계의 참조 롤/피치/요 회전 행렬
    RF2LF_RPY =  RF2PC_RPY*PC2LF_RPY;

    //오른발 좌표계에서 바라본 왼발 좌표계의 참조 롤/피치/요 오일러 각도 계산
    nx = RF2LF_RPY(0,0); ny = RF2LF_RPY(1,0); nz = RF2LF_RPY(2,0); 
		ox = RF2LF_RPY(0,1); oy = RF2LF_RPY(1,1); oz = RF2LF_RPY(2,1);
    ax = RF2LF_RPY(0,2); ay = RF2LF_RPY(1,2); az = RF2LF_RPY(2,2);
	
		RF2LF_PR[5] = atan2(ny,nx);  // FI : Roll about z0 axis
    RF2LF_PR[4] = atan2(-nz,cos(RF2LF_PR[5])*nx + sin(RF2LF_PR[5])*ny);  // theta : Pitch about y0 axis
    RF2LF_PR[3] = atan2(sin(RF2LF_PR[5])*ax - cos(RF2LF_PR[5])*ay, -sin(RF2LF_PR[5])*ox + cos(RF2LF_PR[5])*oy) ; // csi : Yaw about x0 axis

		//Desired X 와의 Error Vector 구하기 

		//오른발 좌표계에서 바라본 골반의 위치/방위 오차 벡터 계산
		for(int i =0; i<12; i++) Ref_PR_eigen(i) = Ref_RP[i];
		//std::cout<<"Ref_PR_eigen"<<std::endl;
		//std::cout<<Ref_PR_eigen<<std::endl;

    Temp(0) = -RL_T_80[0][0]*Ref_PR_eigen(0)-RL_T_80[0][1]*Ref_PR_eigen(1)-RL_T_80[0][2]*Ref_PR_eigen(2);
		Temp(1) = -RL_T_80[1][0]*Ref_PR_eigen(0)-RL_T_80[1][1]*Ref_PR_eigen(1)-RL_T_80[1][2]*Ref_PR_eigen(2);
		Temp(2) = -RL_T_80[2][0]*Ref_PR_eigen(0)-RL_T_80[2][1]*Ref_PR_eigen(1)-RL_T_80[2][2]*Ref_PR_eigen(2);

    F[0] = Temp(0) -  PR[0];
    F[1] = Temp(1) -  PR[1];
    F[2] = Temp(2) -  PR[2];
    F[3] = RF2PC_PR[3] -  PR[3];
    F[4] = RF2PC_PR[4] -  PR[4];
    F[5] = RF2PC_PR[5] -  PR[5];
    
    //오른발 좌표계에서 바라본 왼발의 위치/방위 오차 벡터 계산

    Temp(0) = RL_T_80[0][0]*(Ref_PR_eigen(6)-Ref_PR_eigen(0))+RL_T_80[0][1]*(Ref_PR_eigen(7)-Ref_PR_eigen(1))+RL_T_80[0][2]*(Ref_PR_eigen(8)-Ref_PR_eigen(2));
		Temp(1) = RL_T_80[1][0]*(Ref_PR_eigen(6)-Ref_PR_eigen(0))+RL_T_80[1][1]*(Ref_PR_eigen(7)-Ref_PR_eigen(1))+RL_T_80[1][2]*(Ref_PR_eigen(8)-Ref_PR_eigen(2));
		Temp(2) = RL_T_80[2][0]*(Ref_PR_eigen(6)-Ref_PR_eigen(0))+RL_T_80[2][1]*(Ref_PR_eigen(7)-Ref_PR_eigen(1))+RL_T_80[2][2]*(Ref_PR_eigen(8)-Ref_PR_eigen(2));
		
    F[6] = Temp(0) - PR[6];
    F[7] = Temp(1) - PR[7];
    F[8] = Temp(2) - PR[8];
    F[9] = RF2LF_PR[3] - PR[9];
    F[10] = RF2LF_PR[4] - PR[10];
    F[11] = RF2LF_PR[5] - PR[11];
	
		//전체 오차 크기 계산
    ERR = sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8] + F[9]*F[9] + F[10]*F[10] + F[11]*F[11]);
		//std::cout<<"ERR : "<<ERR<<endl;
	
		if (ERR < 0.00000015) //0.00000015
    {
			for(int i=0; i < 12; i++)	IK_th[i] = th[i];
			std::cout << "iter = " << iter << std::endl;
			break;
		}
		else if(iter == 99)
    {
			for(int i=0; i < 12; i++)	IK_th[i] = Init_th[i];
			std::cout << "iter = 99" << std::endl;
			break;
		}
	
		// Jacobian Cacluation using perturbation //
		for(int i=0; i<12; i++) old_PR[i] = PR[i];
		for(int i=0; i<12; i++)
    {
			for(int j=0; j<12; j++) New_Q4J[j] = old_Q[j];  // Reset
						
			New_Q4J[i] = old_Q[i] + del_Q; // Perturb
			//std::cout << "New_Q4J[" <<i<<"] = " <<New_Q4J[i] << std::endl;
			
			// Forward Kinematics again //
			BRP_RL2PC_FK(New_Q4J,PR);

			//for(int j=0; j<12; j++) std::cout<<"PR["<<j<<"] : "<<PR[j]<<endl;

			for(int j = 0; j<12; j++)
			{
				New_PR[j][i] = PR[j];
			}
		} // End of for(i=0; i<12; i++){	

		/*
		std::cout << "New_PR:" << std::endl;
		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				std::cout << "\t" << New_PR[i][j];
			}
		std::cout << std::endl; // 엔터
		}			
		*/

		for(int i = 0; i<12; i++)
    {
			for(int j = 0; j<12; j++)
      {
				J[i][j] = (New_PR[i][j] - old_PR[i])*10000; //del_Q = 0.0001;
			}
		}

		/*
		std::cout << "old_PR:" << std::endl;
		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				std::cout << "\t" << old_PR[i];
			}
		std::cout << std::endl; // 엔터
		}
		*/
		/*
		std::cout << "J:" << std::endl;
		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				std::cout << "\t" << J[i][j];
				eigen_J(i,j)= J[i][j];
			}
		//std::cout << std::endl; // 엔터
		}
		*/
		
		inv_mat12(0,12,J,Inv_J);

		/*
		std::cout << "Inv_J:" << std::endl;
		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{	
				std::cout << "\t" << Inv_J[i][j];
				eigen_Inv_J(i,j) = Inv_J[i][j];
			}
		std::cout << std::endl; // 엔터
		}
		std::cout<<"eigen_J*eigen_Inv_J : "<<std::endl;
		std::cout<<eigen_J*eigen_Inv_J<<std::endl;
		*/
  
		for(int k=0; k < 12; k++)
		{
	  	sum = 0.;
			for(int j=0; j < 12;j++)
			{
				sum = sum + Inv_J[k][j]*F[j];
			}
			th[k] = old_Q[k] + sum;
			//std::cout<<"sum["<<k<<"] : "<<sum<<std::endl;
			//std::cout << "th["<<k<<"] : " <<th[k]<<std::endl;
		}
		
		if (th[3] < 0)
    {
			th[3] = -th[3];
		}
		if (th[9] < 0)
    {
			th[9] = -th[9];
		}
	} // End of for (iter = 0; iter <= 100; iter++) {
}

void BRP_12DOF_IK(double Ref_RP[12], double Init_th[12], double IK_th[12])  // 12DOF IK푸는식
{
	int iter,i,j,k;
	double th[12] = {0.,}, th_R[6] = {0.,}, th_L[6] = {0.,}, PR[12] = {0.,}, old_PR[12] = {0.,}, F[12] = {0.,}, old_Q[12] = {0.,}, RL_PR[6] = {0.,},
      LL_PR[6] = {0.,},	New_Q4J[12] = {0.,}, New_Q4J_R[6] = {0.,}, New_Q4J_L[6] = {0.,}, ERR = 0., sum = 0., New_PR[12][12] = {{0.,},{0.,},{0.,},{0.,},{0.,},{0.,}},
			J[12][12] = {{0.,},}, Inv_J[12][12] = {{0.,},};
	MatrixXd eigen_J = MatrixXd::Zero(12,12);
	MatrixXd eigen_J_Inv = MatrixXd::Zero(12,12);
	MatrixXd eigen_Inv_J = MatrixXd::Zero(12,12);
	const double del_Q = 0.0001; // 0.0001
	
  // Initial Joint angles //
	for (int i=0; i < 12; i++) th[i] = Init_th[i]; 
	
	for (int iter = 0; iter < 100; iter++) //iter<100
  {
    //c_cnt++;
    //if(c_cnt > 3) break;
    for (int i=0; i < 12; i++)	old_Q[i] = th[i];
    for (int i = 0; i < 6; i++){th_R[i] = th[i]; th_L[i] = th[i+6];}
    
    // Forward Kinematics
    BRP_RL_FK(th_R, RL_PR);
    BRP_LL_FK(th_L, LL_PR);

    for(int i = 0; i<6; i++)
    {
      PR[i] = RL_PR[i]; PR[i+6] = LL_PR[i];
      //std::cout<<"RL_PR["<<i<<"] = "<<RL_PR[i]<<std::endl;
      //std::cout<<"LL_PR["<<i<<"] = "<<LL_PR[i]<<std::endl;
    }
    
    // Error calculation
    for (int i=0; i < 12; i++) F[i] = Ref_RP[i] - PR[i];
    //for (int i=0; i < 12; i++) std::cout<<"Ref_RP["<<i<<"] = "<<Ref_RP[i]<<std::endl;
    //for (int i=0; i < 12; i++) std::cout<<"PR["<<i<<"] = "<<PR[i]<<std::endl;
    //for (int i=0; i < 12; i++) std::cout<<"F["<<i<<"] = "<<F[i]<<std::endl;
    ERR = sqrt(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8] + F[9]*F[9] + F[10]*F[10] + F[11]*F[11]);
    //std::cout << "ERR = " << ERR << std::endl;

    if (ERR < 0.00000015) //0.00000015
    {
      for(int i=0; i < 12; i++)	IK_th[i] = th[i];
      std::cout << "iter = " << iter << std::endl;
      break;
    }
    else if(iter == 99)
    {
      for(int i=0; i < 12; i++)	IK_th[i] = Init_th[i];
      std::cout << "iter = 99" << std::endl;
      break;
    }
    
    // Jacobian Cacluation using perturbation //
    for(int i=0; i<12; i++) old_PR[i] = PR[i];

    for(int i=0; i<12; i++)
    {
      for(int j=0; j<12; j++) New_Q4J[j] = old_Q[j];  // Reset
            
      New_Q4J[i] = old_Q[i] + del_Q; // Perturb
      //std::cout << "New_Q4J[" <<i<<"] = " <<New_Q4J[i] << std::endl;
      for(int j=0; j<6; j++) {New_Q4J_R[j] = New_Q4J[j]; New_Q4J_L[j] = New_Q4J[j+6];}
      
      // Forward Kinematics again //
      BRP_RL_FK(New_Q4J_R, RL_PR);
      BRP_LL_FK(New_Q4J_L, LL_PR);
      
      for(int j = 0; j<6;j++)
      {
        PR[j] = RL_PR[j]; PR[j+6] = LL_PR[j];
      }
      for(int j = 0; j<12; j++)
      {
        New_PR[j][i] = PR[j];
      }
    } // End of for(i=0; i<12; i++){

    /*
    std::cout << "New_PR:" << std::endl;
    for (int i = 0; i < 12; i++)
    {
      for (int j = 0; j < 12; j++)
      {
        std::cout << "\t" << New_PR[i][j];
      }
    std::cout << std::endl; // 엔터
    }			
    */

    for(int i = 0; i<12; i++)
    {
      for(int j = 0; j<12; j++)
      {
        J[i][j] = (New_PR[i][j] - old_PR[i])*10000; //del_Q = 0.0001;
      }
    }

    /*
    std::cout << "old_PR:" << std::endl;
    for (int i = 0; i < 12; i++)
    {
      for (int j = 0; j < 12; j++)
      {
        std::cout << "\t" << old_PR[i];
      }
    std::cout << std::endl; // 엔터
    }
    */
    //std::cout << "J:" << std::endl;

    for (int i = 0; i < 12; i++)
    {
      for (int j = 0; j < 12; j++)
      {
        //std::cout << "\t" << J[i][j];
        eigen_J(i,j)= J[i][j];
      }
    //std::cout << std::endl; // 엔터
    }

    inv_mat12(0,12,J,Inv_J);
    //eigen_J_Inv = eigen_J.inverse();
    /*
    std::cout << "Inv_J:" << std::endl;
    for (int i = 0; i < 12; i++)
    {
      for (int j = 0; j < 12; j++)
      {	
        std::cout << "\t" << Inv_J[i][j];
        eigen_Inv_J(i,j) = Inv_J[i][j];
      }
    std::cout << std::endl; // 엔터
    }
    */		
    //std::cout<<"eigen_J_Inv : "<<std::endl;
    //std::cout<<eigen_J_Inv<<std::endl;
    //std::cout<<"eigen_J*eigen_J_Inv : "<<std::endl;
    //std::cout<<eigen_J*eigen_J_Inv<<std::endl;	
    //std::cout<<"eigen_J*eigen_Inv_J : "<<std::endl;
    //std::cout<<eigen_J*eigen_Inv_J<<std::endl;			
    
    for(int k=0; k < 12; k++)
    {
      sum = 0.;
      for(int j=0; j < 12;j++)
      {
        sum = sum + Inv_J[k][j]*F[j];
      }
      th[k] = old_Q[k] + sum;
      //std::cout<<"sum["<<k<<"] : "<<sum<<std::endl;
      //std::cout << "th["<<k<<"] : " <<th[k]<<std::endl;
    }
    
    if (th[3] < 0)
    {  
      th[3] = -th[3];
    }
    if (th[9] < 0)
    {  
      th[9] = -th[9];
    }
	} // End of for (iter = 0; iter <= 100; iter++)
}

void BRP_RL2PC_FK(double th[12], double PR[12])  // (2015_07_29)
{
	double  c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6, c34, s34, s345, c345, c12345, s1345, c1345, nx, ny, nz, ox, oy, oz, ax, ay, az;
	c1 = cos(th[0]); c2 = cos(th[1]); c3 = cos(th[2]); c4 = cos(th[3]); c5 = cos(th[4]); c6 = cos(th[5]); 
	s1 = sin(th[0]); s2 = sin(th[1]); s3 = sin(th[2]); s4 = sin(th[3]); s5 = sin(th[4]); s6 = sin(th[5]);
	c34 = cos(th[2] + th[3]); s34 = sin(th[2] + th[3]); s345 = sin(th[2] + th[3] + th[4]); c345 = cos(th[2] + th[3] + th[4]);
	c12345 = cos(th[0] + th[1] + th[2] + th[3] + th[4]); s1345 = sin(th[0] + th[2] +th[3] + th[4]); c1345 = cos(th[0] + th[2] +th[3] + th[4]);
	
	//double RL_T_80[4][4] = {{0.,},};
	double RL_T_08[4][4] = {{0.5*(cos(th[0] - th[2] - th[3] - th[4]) + cos(th[0] + th[2] + th[3] + th[4]) - 2.*s1*s2*s345), -c2*c6*s1 + 0.25*(cos(th[0] - th[1] - th[2] - th[3] - th[4]) - cos(th[0] + th[1] - th[2] - th[3] - th[4]) + cos(th[0] - th[1] + th[2] + th[3] +th[4]) - c12345 - 2.*sin(th[0] - th[2] - th[3] - th[4]) + 2.*s1345)*s6, 0.25*c6*(cos(th[0] - th[1] - th[2] - th[3] - th[4]) - cos(th[0] + th[1] - th[2] - th[3] - th[4]) + cos(th[0] - th[1] + th[2] + th[3] + th[4]) - c12345 - 2.*sin(th[0] - th[2] - th[3] - th[4]) + 2.*s1345) + c2*s1*s6, -c3*((L3 + c4*(L4 + L6*c5*c6))*s1*s2 + c1*(L4 + L6*c5*c6)*s4) - c1*((L3 + c4*(L4 + L6*c5*c6))*s3 + L6*c34*c6*s5) + s1*(s2*((L4 + L6*c5*c6)*s3*s4 + L6*c6*s34*s5) - L6*c2*s6)},{0.25*(-cos(th[0] - th[1] - th[2] - th[3] - th[4]) + cos(th[0] + th[1] - th[2] - th[3] - th[4]) + cos(th[0] - th[1] + th[2] + th[3] + th[4]) - c12345 + 2.*sin(th[0] - th[2] - th[3] - th[4]) + 2.*s1345),s1*s345*s6 + c1*(c2*c6 - c345*s2*s6),0.5*c6*(cos(th[0] - th[2] - th[3] - th[4]) - c1345 - 2.*c1*c345*s2) - c1*c2*s6,-L0 - s1*((L3 + L4*c4)*s3 + L4*c3*s4 + L6*c6*s345) + c1*(c3*(L3 + L4*c4)*s2 + L6*c345*c6*s2 - L4*s2*s3*s4 + L6*c2*s6)},{-c2*s345,c6*s2 + c2*c345*s6,c2*c345*c6 - s2*s6,-c2*(c3*(L3 + L4*c4) + L6*c345*c6 - L4*s3*s4) + L6*s2*s6},{0,0,0,1}};

	inv_mat6(0, 4, RL_T_08, 0, RL_T_80, 0);
	
	// 8번 좌표계(=오른발 좌표계)에서 바라본 0번 좌표게(=골반 좌표계)의 위치 (pelvis center position w.r.t right foot fixed frame)
	PR[0] = RL_T_80[0][3];
	PR[1] = RL_T_80[1][3];
	PR[2] = RL_T_80[2][3];
	//std::cout<<PR[0]<<std::endl;
	//std::cout<<PR[1]<<std::endl;
	//std::cout<<PR[2]<<std::endl;
	
  // 오른발 좌표계에서 바라본 골반의 오일러 각도 계산, o0,x0,y0,z0  (Euler order : z -> y -> x)
  nx = RL_T_80[0][0]; ny = RL_T_80[1][0]; nz = RL_T_80[2][0]; ox = RL_T_80[0][1]; oy = RL_T_80[1][1]; oz = RL_T_80[2][1];
  ax = RL_T_80[0][2]; ay = RL_T_80[1][2]; az = RL_T_80[2][2];

  PR[5] = atan2(ny,nx) ; // FI : Roll about z0 axis
  PR[4] = atan2(-nz,cos(PR[5])*nx + sin(PR[5])*ny);  // theta : Pitch about y0 axis
  PR[3] = atan2(sin(PR[5])*ax - cos(PR[5])*ay, -sin(PR[5])*ox + cos(PR[5])*oy) ; // csi : Yaw about x0 axis

	double  c7, c8, c9, c10, c11, c12, s7, s8, s9, s10, s11, s12, c910, s910, s91011, c91011, c7891011, s791011, c791011;
	c7 = cos(th[6]); c8 = cos(th[7]); c9 = cos(th[8]); c10 = cos(th[9]); c11 = cos(th[10]); c12 = cos(th[11]); 
	s7 = sin(th[6]); s8 = sin(th[7]); s9 = sin(th[8]); s10 = sin(th[9]); s11 = sin(th[10]); s12 = sin(th[11]);
	c910 = cos(th[8] + th[9]); s910 = sin(th[8] + th[9]); s91011 = sin(th[8] + th[9] + th[10]); c91011 = cos(th[8] + th[9] + th[10]);
	c7891011 = cos(th[6] + th[7] + th[8] + th[9] + th[10]); s791011 = sin(th[6] + th[8] +th[9] + th[10]); c791011 = cos(th[6] + th[8] +th[9] + th[10]);

	double LL_T_08[4][4] = {{0.5*(cos(th[6] - th[8] - th[9] - th[10]) + cos(th[6] + th[8] + th[9] + th[10]) - 2*s7*s8*s91011),-c8*c12*s7 + 0.25*(cos(th[6] - th[7] - th[8] - th[9] - th[10]) - cos(th[6] + th[7] - th[8] - th[9] - th[10]) + cos(th[6] - th[7] + th[8] + th[9] +th[10]) - c7891011 - 2.*sin(th[6] - th[8] - th[9] - th[10]) + 2.*s791011)*s12,0.25*c12*(cos(th[6] - th[7] - th[8] - th[9] - th[10]) - cos(th[6] + th[7] - th[8] - th[9] - th[10]) + cos(th[6] - th[7] + th[8] + th[9] + th[10]) - c7891011 - 2.*sin(th[6] - th[8] - th[9] - th[10]) + 2.*s791011) + c8*s7*s12,-c9*((L3 + c10*(L4 + L6*c11*c12))*s7*s8 + c7*(L4 + L6*c11*c12)*s10) - c7*((L3 + c10*(L4 + L6*c11*c12))*s9 + L6*c910*c12*s11) + s7*(s8*((L4 + L6*c11*c12)*s9*s10 + L6*c12*s910*s11) - L6*c8*s12)},{0.25*(-cos(th[6] - th[7] - th[8] - th[9] - th[10]) + cos(th[6] + th[7] - th[8] - th[9] - th[10]) + cos(th[6] - th[7] + th[8] + th[9] + th[10]) - c7891011 + 2.*sin(th[6] - th[8] - th[9] - th[10]) + 2.*s791011),s7*s91011*s12 + c7*(c8*c12 - c91011*s8*s12),0.5*c12*(cos(th[6] - th[8] - th[9] - th[10]) - c791011 - 2.*c7*c91011*s8) - c7*c8*s12,L0 - s7*((L3 + L4*c10)*s9 + L4*c9*s10 + L6*c12*s91011) + c7*(c9*(L3 + L4*c10)*s8 + L6*c91011*c12*s8 - L4*s8*s9*s10 + L6*c8*s12)},{-c8*s91011,c12*s8 + c8*c91011*s12,c8*c91011*c12 - s8*s12,-c8*(c9*(L3 + L4*c10) + L6*c91011*c12 - L4*s9*s10) + L6*s8*s12},{0,0,0,1}};
	
	MatrixXd LLT08 = MatrixXd::Zero(4,4);
	MatrixXd RLT80 = MatrixXd::Zero(4,4);
	
  for(int i = 0; i< 4; i++)
  {
		for(int j =0; j < 4; j++)
    {
			LLT08(i,j) = LL_T_08[i][j];
			RLT80(i,j) = RL_T_80[i][j];
		}
	}

	//오른발 좌표계에서 바라본 왼 다리 관절들의 Homogeneous Transformation Matrix
	LLT08 = RLT80*LLT08;

	for(int i = 0; i< 4; i++)
  {
		for(int j =0; j < 4; j++)
    {
			LL_T_08[i][j] = LLT08(i,j);
		}
	}

	// 오른발 좌표계에서 바라본 왼 발의 위치
	PR[6] = LL_T_08[0][3];
	PR[7] = LL_T_08[1][3];
	PR[8] = LL_T_08[2][3];	

	//오른발 좌표계에서 바라본 왼발의 오일러 각도 계산, o0,x0,y0,z0  (Euler order : z -> y -> x)
  nx = LL_T_08[0][0]; ny = LL_T_08[1][0]; nz = LL_T_08[2][0]; ox = LL_T_08[0][1]; oy = LL_T_08[1][1]; oz = LL_T_08[2][1];
  ax = LL_T_08[0][2]; ay = LL_T_08[1][2]; az = LL_T_08[2][2]; 	

	PR[11] = atan2(ny,nx);  // FI : Roll about z0 axis
	PR[10] = atan2(-nz,cos(PR[11])*nx + sin(PR[11])*ny);  // theta : Pitch about y0 axis
	PR[9] = atan2(sin(PR[11])*ax - cos(PR[11])*ay, -sin(PR[11])*ox + cos(PR[11])*oy);  // csi : Yaw about x0 axis   	
}

void BRP_LL_FK(double th[6], double PR[6])  // (2015_07_29)
{
	double  c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6, c34, s34, s345, c345, c12345, s1345, c1345, nx, ny, nz, ox, oy, oz, ax, ay, az;
	c1 = cos(th[0]); c2 = cos(th[1]); c3 = cos(th[2]); c4 = cos(th[3]); c5 = cos(th[4]); c6 = cos(th[5]); 
	s1 = sin(th[0]); s2 = sin(th[1]); s3 = sin(th[2]); s4 = sin(th[3]); s5 = sin(th[4]); s6 = sin(th[5]);
	c34 = cos(th[2] + th[3]); s34 = sin(th[2] + th[3]); s345 = sin(th[2] + th[3] + th[4]); c345 = cos(th[2] + th[3] + th[4]);
	c12345 = cos(th[0] + th[1] + th[2] + th[3] + th[4]); s1345 = sin(th[0] + th[2] +th[3] + th[4]); c1345 = cos(th[0] + th[2] +th[3] + th[4]);

	// Endeffector position
	PR[0] = -c3*((L3 + c4*(L4 + L6*c5*c6))*s1*s2 + c1*(L4 + L6*c5*c6)*s4) - c1*((L3 + c4*(L4 + L6*c5*c6))*s3 + L6*c34*c6*s5) + s1*(s2*((L4 + L6*c5*c6)*s3*s4 + L6*c6*s34*s5) - L6*c2*s6);
	PR[1] = L0 - s1*((L3 + L4*c4)*s3 + L4*c3*s4 + L6*c6*s345) + c1*(c3*(L3 + L4*c4)*s2 + L6*c345*c6*s2 - L4*s2*s3*s4 + L6*c2*s6);
	PR[2] = -c2*(c3*(L3 + L4*c4) + L6*c345*c6 - L4*s3*s4) + L6*s2*s6;

	// Endeffector orientation
	nx = 0.5*(cos(th[0] - th[2] - th[3] - th[4]) + cos(th[0] + th[2] + th[3] + th[4]) - 2.*s1*s2*s345);
	ny = 0.25*(-cos(th[0] - th[1] - th[2] - th[3] - th[4]) + cos(th[0] + th[1] - th[2] - th[3] - th[4]) + cos(th[0] - th[1] + th[2] + th[3] + th[4]) - c12345 + 2.*sin(th[0] - th[2] - th[3] - th[4]) + 2.*s1345);
	nz = -c2*s345;

	//printf("nx = %f, ny = %f, nz = %f \n",nx,ny,nz);

	ox = -c2*c6*s1 + 0.25*(cos(th[0] - th[1] - th[2] - th[3] - th[4]) - cos(th[0] + th[1] - th[2] - th[3] - th[4]) + cos(th[0] - th[1] + th[2] + th[3] +th[4]) - c12345 - 2.*sin(th[0] - th[2] - th[3] - th[4]) + 2.*s1345)*s6;
	oy = s1*s345*s6 + c1*(c2*c6 - c345*s2*s6);
	oz = c6*s2 + c2*c345*s6;

	//printf("ox = %f, oy = %f, oz = %f \n",ox,oy,oz);

	ax = 0.25*c6*(cos(th[0] - th[1] - th[2] - th[3] - th[4]) - cos(th[0] + th[1] - th[2] - th[3] - th[4]) + cos(th[0] - th[1] + th[2] + th[3] + th[4]) - c12345 - 2.*sin(th[0] - th[2] - th[3] - th[4]) + 2.*s1345) + c2*s1*s6;
	ay = 0.5*c6*(cos(th[0] - th[2] - th[3] - th[4]) - c1345 - 2.*c1*c345*s2) - c1*c2*s6;
	az = c2*c345*c6 - s2*s6;

	//printf("ax = %f, ay = %f, az = %f \n",ax,ay,az);

	PR[5] = atan2(ny,nx);  // FI : Roll about z0 axis
	PR[4] = atan2(-nz,cos(PR[5])*nx + sin(PR[5])*ny);  // theta : Pitch about y0 axis
	PR[3] = atan2(sin(PR[5])*ax - cos(PR[5])*ay, -sin(PR[5])*ox + cos(PR[5])*oy);  // csi : Yaw about x0 axis
}

void BRP_RL_FK(double th[6], double PR[6])  // (2015_07_29)
{
	double  c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6, c34, s34, s345, c345, c12345, s1345, c1345, nx, ny, nz, ox, oy, oz, ax, ay, az;

	c1 = cos(th[0]); c2 = cos(th[1]); c3 = cos(th[2]); c4 = cos(th[3]); c5 = cos(th[4]); c6 = cos(th[5]); 
	s1 = sin(th[0]); s2 = sin(th[1]); s3 = sin(th[2]); s4 = sin(th[3]); s5 = sin(th[4]); s6 = sin(th[5]);
	c34 = cos(th[2] + th[3]); s34 = sin(th[2] + th[3]); s345 = sin(th[2] + th[3] + th[4]); c345 = cos(th[2] + th[3] + th[4]);
	c12345 = cos(th[0] + th[1] + th[2] + th[3] + th[4]); s1345 = sin(th[0] + th[2] +th[3] + th[4]); c1345 = cos(th[0] + th[2] +th[3] + th[4]);

	// Endeffector position
	PR[0] = -c3*((L3 + c4*(L4 + L6*c5*c6))*s1*s2 + c1*(L4 + L6*c5*c6)*s4) - c1*((L3 + c4*(L4 + L6*c5*c6))*s3 + L6*c34*c6*s5) + s1*(s2*((L4 + L6*c5*c6)*s3*s4 + L6*c6*s34*s5) - L6*c2*s6);
	PR[1] = -L0 - s1*((L3 + L4*c4)*s3 + L4*c3*s4 + L6*c6*s345) + c1*(c3*(L3 + L4*c4)*s2 + L6*c345*c6*s2 - L4*s2*s3*s4 + L6*c2*s6);
	PR[2] = -c2*(c3*(L3 + L4*c4) + L6*c345*c6 - L4*s3*s4) + L6*s2*s6;

	// Endeffector orientation
	nx = 0.5*(cos(th[0] - th[2] - th[3] - th[4]) + cos(th[0] + th[2] + th[3] + th[4]) - 2*s1*s2*s345);
	ny = 0.25*(-cos(th[0] - th[1] - th[2] - th[3] - th[4]) + cos(th[0] + th[1] - th[2] - th[3] - th[4]) + cos(th[0] - th[1] + th[2] + th[3] + th[4]) - c12345 + 2.*sin(th[0] - th[2] - th[3] - th[4]) + 2.*s1345);
	nz = -c2*s345;

	ox = -c2*c6*s1 + 0.25*(cos(th[0] - th[1] - th[2] - th[3] - th[4]) - cos(th[0] + th[1] - th[2] - th[3] - th[4]) + cos(th[0] - th[1] + th[2] + th[3] +th[4]) - c12345 - 2.*sin(th[0] - th[2] - th[3] - th[4]) + 2.*s1345)*s6;
	oy = s1*s345*s6 + c1*(c2*c6 - c345*s2*s6);
	oz = c6*s2 + c2*c345*s6;

	ax = 0.25*c6*(cos(th[0] - th[1] - th[2] - th[3] - th[4]) - cos(th[0] + th[1] - th[2] - th[3] - th[4]) + cos(th[0] - th[1] + th[2] + th[3] + th[4]) - c12345 - 2.*sin(th[0] - th[2] - th[3] - th[4]) + 2.*s1345) + c2*s1*s6;
	ay = 0.5*c6*(cos(th[0] - th[2] - th[3] - th[4]) - c1345 - 2.*c1*c345*s2) - c1*c2*s6;
	az = c2*c345*c6 - s2*s6;

	PR[5] = atan2(ny,nx);  // FI : Roll about z0 axis
	PR[4] = atan2(-nz,cos(PR[5])*nx + sin(PR[5])*ny);  // theta : Pitch about y0 axis
	PR[3] = atan2(sin(PR[5])*ax - cos(PR[5])*ay, -sin(PR[5])*ox + cos(PR[5])*oy);  // csi : Yaw about x0 axis
}

void BCHT(double Ref_Body_Roll_deg, double Ref_Body_Pitch_deg, double Ref_Body_Yaw_deg, double Ref_RL_BC[3], double Ref_RL_BC_TF[3], double O[3])
{
	double  c_roll, c_pitch, c_yaw, s_roll, s_pitch, s_yaw;

	c_roll = cos(-Ref_Body_Roll_deg*deg2rad);
	c_pitch = cos(-Ref_Body_Pitch_deg*deg2rad);
	c_yaw = cos(-Ref_Body_Yaw_deg*deg2rad);
	s_roll = sin(-Ref_Body_Roll_deg*deg2rad);
	s_pitch = sin(-Ref_Body_Pitch_deg*deg2rad);
	s_yaw = sin(-Ref_Body_Yaw_deg*deg2rad);

	Ref_RL_BC_TF[0] = c_pitch*c_yaw*(Ref_RL_BC[0] - O[0]) + (-s_yaw*c_roll+s_roll*s_pitch*c_yaw)*(Ref_RL_BC[1] - O[1]) + (s_yaw*s_roll+s_pitch*c_yaw*c_roll)*(Ref_RL_BC[2] - O[2]);
	Ref_RL_BC_TF[1] = s_yaw*c_pitch*(Ref_RL_BC[0] - O[0]) + (c_yaw*c_roll+s_roll*s_pitch*s_yaw)*(Ref_RL_BC[1] - O[1]) + (-c_yaw*s_roll+s_yaw*s_pitch*c_roll)*(Ref_RL_BC[2] - O[2]);
	Ref_RL_BC_TF[2] = -s_pitch*(Ref_RL_BC[0] - O[0]) + (c_pitch*s_roll)*(Ref_RL_BC[1] - O[1]) + (c_pitch*c_roll)*(Ref_RL_BC[2] - O[2]);
}

void inv_mat6(int m, int n, double Mat4[][4], double Mat6[][6], double c_inv4[4][4], double c_inv[6][6])   // (2015_07_29)
{
	double big, size, abig, cbig, ratio; 
	int i, k, j, ibig;
	double jcob[6][6];
	
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
		{
			if(n == 4)
				jcob[i][j]=Mat4[i][j];
			else if(n==6)
				jcob[i][j]=Mat6[i][j];
		}
	}

	if(n==6)
  {
    for(i=m;i<n;i++)
    {
      for(j=m;j<n;j++)
        {
        c_inv[i][j]=0.;
        if(i==j) c_inv[i][i]=1.;
        }
    }

    for (k=m; k<n; k++)
    {
      big=fabs(jcob[k][k]);
      ibig=k;

      for (i=k;i<n;i++)
      {
        size=fabs(jcob[i][k]);
        if(size < big) goto next;
        big=size;
        ibig=i;
        next:;			
      }
	
      if(k==ibig) goto next2;
      for(j=m;j<n;j++)
			{
			  if(j>=k)
				{
          abig=jcob[ibig][j];
          jcob[ibig][j]=jcob[k][j];
          jcob[k][j]=abig;
        }
        cbig=c_inv[ibig][j];
        c_inv[ibig][j]=c_inv[k][j];
        c_inv[k][j]=cbig;
 			}

      next2:;

		  if(jcob[k][k] == 0.) {/*lcd_putch('S'); halt_e();return(1);*/}
		  for(i=m;i<n;i++)
			{
        if(i==k) goto next3;
        ratio=jcob[i][k]/jcob[k][k];
        for (j=m;j <n;j++)
				{
          if(j>=k) jcob[i][j]=jcob[i][j] - ratio*jcob[k][j];
          
          c_inv[i][j]=c_inv[i][j] - ratio*c_inv[k][j];
				}
        next3:;
			}
		}

  	for (k=m; k<n; k++)
		{
	  	for(j=m;j<n;j++)
			{
		  	c_inv[k][j]=c_inv[k][j]/jcob[k][k];
			}
		}
	}
	else if(n==4)
  {
    for(i=m;i<n;i++)
    {
      for(j=m;j<n;j++)
      {
        c_inv4[i][j]=0.;
        if(i==j) c_inv4[i][i]=1.;
      }
    }

    for (k=m; k<n; k++)
    {
      big=fabs(jcob[k][k]);
      ibig=k;

      for (i=k;i<n;i++)
      {
        size=fabs(jcob[i][k]);
        if(size < big) goto next1_1;
        big=size;
        ibig=i;
        next1_1:;			
      }
	
		  if(k==ibig) goto next2_1;
		  for(j=m;j<n;j++)
			{
			  if(j>=k)
				{
          abig=jcob[ibig][j];
          jcob[ibig][j]=jcob[k][j];
          jcob[k][j]=abig;
				}
        cbig=c_inv4[ibig][j];
        c_inv4[ibig][j]=c_inv4[k][j];
        c_inv4[k][j]=cbig;
 			}
		  
      next2_1:;

		  if(jcob[k][k] == 0.) {/*lcd_putch('S'); halt_e();return(1);*/}
		  for(i=m;i<n;i++)
			{
			  if(i==k) goto next3_1;
        ratio=jcob[i][k]/jcob[k][k];
        for (j=m;j <n;j++)
				{
          if(j>=k) jcob[i][j]=jcob[i][j] - ratio*jcob[k][j];
          c_inv4[i][j]=c_inv4[i][j] - ratio*c_inv4[k][j];
				}
        next3_1:;
			}
		}

  	for (k=m; k<n; k++)
		{
		  for(j=m;j<n;j++)
			{
			  c_inv4[k][j]=c_inv4[k][j]/jcob[k][k];
			}
		}
	}
}

void inv_mat12(int m, int n, double Mat12[][12], double c_inv[12][12])   // (2015_07_29)
{
	double big,size,abig, cbig, ratio; 
	int i, k, j, ibig;
	double jcob[12][12];
	
	for(i=0;i<n;i++)
	{
		for(j=0;j<n;j++)
		{
			if(n==12)
				jcob[i][j]=Mat12[i][j];
		}
	}

	if(n==12)
  {
    for(i=m;i<n;i++)
    {
      for(j=m;j<n;j++)
        {
        c_inv[i][j]=0.;
        if(i==j) c_inv[i][i]=1.;
        }
    }

  	for (k=m; k<n; k++)
		{
      big=fabs(jcob[k][k]);
      ibig=k;

	  	for (i=k;i<n;i++)
			{
        size=fabs(jcob[i][k]);
        if(size < big) goto next;
        big=size;
        ibig=i;
        next:;			
			}
	
  		if(k==ibig) goto next2;
  		
      for(j=m;j<n;j++)
			{
		  	if(j>=k)
				{
          abig=jcob[ibig][j];
          jcob[ibig][j]=jcob[k][j];
          jcob[k][j]=abig;
				}
        cbig=c_inv[ibig][j];
        c_inv[ibig][j]=c_inv[k][j];
        c_inv[k][j]=cbig;
 			}
      next2:;

	  	if(jcob[k][k] == 0.) {/*lcd_putch('S'); halt_e();return(1);*/}
	  	for(i=m;i<n;i++)
			{
        if(i==k) goto next3;
        ratio=jcob[i][k]/jcob[k][k];
        for (j=m;j <n;j++)
				{
          if(j>=k) jcob[i][j]=jcob[i][j] - ratio*jcob[k][j];
          c_inv[i][j]=c_inv[i][j] - ratio*c_inv[k][j];
				}
        next3:;
			}
		}

  	for (k=m; k<n; k++)
		{
	  	for(j=m;j<n;j++)
			{
	  		c_inv[k][j]=c_inv[k][j]/jcob[k][k];
			}
		}
	}
}

void BodyRotation(double Trans_Ref_PR[6], double Ref_PR[6], double o_fcp[3], double alpha, double beta, double gamma)
{
  //////////// Foot position & orientation change w.r.t pelvis center (Euler angle : Z(gamma) -> Y(beta) -> X(alpha)) //////////////
  
  double c_a = cos(alpha), c_b = cos(beta), c_g = cos(gamma), s_a = sin(alpha), s_b = sin(beta), s_g = sin(gamma);

  
  Trans_Ref_PR[0] = (Ref_PR[0]-o_fcp[0])*(c_b*c_g)+(Ref_PR[1]-o_fcp[1])*(-s_g*c_a + s_a*s_b*c_g)+(Ref_PR[2]-o_fcp[2])*(s_g*s_a + s_b*c_g*c_a); // 연산결과를 사용 행렬연산과정있으면 느려짐.
  Trans_Ref_PR[1] = (Ref_PR[0]-o_fcp[0])*(s_g*c_b)+(Ref_PR[1]-o_fcp[1])*(c_g*c_a + s_a*s_b*s_g)+(Ref_PR[2]-o_fcp[2])*(-c_g*s_a + s_g*s_b*c_a);
  Trans_Ref_PR[2] = (Ref_PR[0]-o_fcp[0])*(-s_b)+(Ref_PR[1]-o_fcp[1])*(c_b*s_a)+(Ref_PR[2]-o_fcp[2])*(c_b*c_a);
  
  Trans_Ref_PR[0] = Trans_Ref_PR[0] + o_fcp[0];
  Trans_Ref_PR[1] = Trans_Ref_PR[1] + o_fcp[1];
  Trans_Ref_PR[2] = Trans_Ref_PR[2] + o_fcp[2];
  
  Trans_Ref_PR[3] = Ref_PR[3] + alpha;
  Trans_Ref_PR[4] = Ref_PR[4] + beta;
  Trans_Ref_PR[5] = Ref_PR[5] + gamma;
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

void gazebo::SUBO3_plugin::Calc_Feedback_Pos()
{
  Math::Vector3d Foot_Pos;
  Math::Matrix3d R, R_Tmp;
  VectorNd QDot;
  MatrixNd L_Jacobian8, L_Jacobian, L_Jacobian_tmp, L_Bmatrix;
  MatrixNd R_Jacobian8, R_Jacobian, R_Jacobian_tmp, R_Bmatrix;

  QDot = VectorNd::Zero(6);

  L_Jacobian8 = MatrixNd::Zero(L_rbdl_model->dof_count,L_rbdl_model->dof_count);
  L_Jacobian = MatrixNd::Zero(6,6);
  L_Jacobian_tmp = MatrixNd::Zero(6,6);
  L_Bmatrix = MatrixNd::Zero(6,6);

  R_Jacobian8 = MatrixNd::Zero(R_rbdl_model->dof_count,R_rbdl_model->dof_count);
  R_Jacobian = MatrixNd::Zero(6,6);
  R_Jacobian_tmp = MatrixNd::Zero(6,6);
  R_Bmatrix = MatrixNd::Zero(6,6);
  
  double pi = 0, theta = 0, psi = 0;

  //*********************Left Leg**********************//
  // Get the End Effector's Aixs
  Foot_Pos = CalcBodyToBaseCoordinates(*L_rbdl_model, L_Q, L_body_FOOT_id, Math::Vector3d(0, 0, 0), true);
  
  // Get the End Effector's Rotation Matrix
  R_Tmp = CalcBodyWorldOrientation(*L_rbdl_model, L_Q, L_body_FOOT_id, true);
  R = R_Tmp.transpose();

  // Get the Euler Angle - pi, theta, psi
  pi = atan2(R(1,0),R(0,0));
  theta = atan2(-R(2,0), cos(pi)*R(0,0) + sin(pi)*R(1,0));
  psi = atan2(sin(pi)*R(0,2) - cos(pi)*R(1,2), -sin(pi)*R(0,1) + cos(pi)*R(1,1));

  cout << "End: " << pi << ' ' << theta << ' ' << psi << endl << endl;;

  // Get the Jacobian
  CalcPointJacobian6D(*L_rbdl_model, L_Q, L_body_FOOT_id, Math::Vector3d(0, 0, 0), L_Jacobian8, true);
  
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
  L_A_Jacobian = L_Bmatrix*L_Jacobian;
  Inv_L_A_Jacobian = L_A_Jacobian.inverse();

  // Calculate the Jacobian dot
  for(int i = 0; i < 6; i++)
  {
    for(int j = 0 ; j< 6; j++)
    {
      L_A_Jacobian_dot(i,j) = (L_A_Jacobian(i,j) - L_prev_A_Jacobian(i,j)) / dt;
      L_prev_A_Jacobian(i,j) = L_A_Jacobian(i,j);
    }
    QDot(i) = L_QDot(i+3);
  }

  // Current Pos & Pos_dot
  L_Foot_Pos(0) = Foot_Pos(0);
  L_Foot_Pos(1) = Foot_Pos(1);
  L_Foot_Pos(2) = Foot_Pos(2);
  L_Foot_Pos(3) = psi;
  L_Foot_Pos(4) = theta;
  L_Foot_Pos(5) = pi;
  L_Foot_Pos_dot = L_A_Jacobian*QDot;

  //*********************Right Leg**********************//
  // Get the End Effector's Aixs
  Foot_Pos = CalcBodyToBaseCoordinates(*R_rbdl_model, R_Q, R_body_FOOT_id, Math::Vector3d(0, 0, 0), true);
  
  // Get the End Effector's Rotation Matrix
  R_Tmp = CalcBodyWorldOrientation(*R_rbdl_model, R_Q, R_body_FOOT_id, true);
  R = R_Tmp.transpose();

  // Get the Euler Angle - pi, theta, psi
  pi = atan2(R(1,0),R(0,0));
  theta = atan2(-R(2,0), cos(pi)*R(0,0) + sin(pi)*R(1,0));
  psi = atan2(sin(pi)*R(0,2) - cos(pi)*R(1,2), -sin(pi)*R(0,1) + cos(pi)*R(1,1));

  // Get the Jacobian
  CalcPointJacobian6D(*R_rbdl_model, R_Q, R_body_FOOT_id, Math::Vector3d(0, 0, 0), R_Jacobian8, true);
  
  // Get the Jacobian for 6 by 6
  for(int i = 0; i < 6; i++)
  {
    for(int j = 0; j < 6; j++)
    {
      R_Jacobian_tmp(i, j) = R_Jacobian8(i, j+3);
    }
  }
  
  // Chage the Row
  for(int j = 0; j < 6; j++)
  {
    for(int i = 0; i < 3; i++)
    {
      R_Jacobian(i,j) = R_Jacobian_tmp(i+3,j);  // linear
    }
    for(int i = 3; i < 6; i++)
    {
      R_Jacobian(i,j) = R_Jacobian_tmp(i-3,j);  // angular
    }
  }

  // Calculate the Analytical Jacobian & Inverse of Analytical Jacobian
  R_Bmatrix << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, cos(pi)/cos(theta), sin(pi)/cos(theta), 0, 0, 0, 0, -sin(pi), cos(pi), 0, 0, 0, 0, cos(pi)*tan(theta), sin(pi)*tan(theta), 1;
  R_A_Jacobian = R_Bmatrix*R_Jacobian;
  Inv_R_A_Jacobian = R_A_Jacobian.inverse();

  // Calculate the Jacobian dot
  for(int i = 0; i < 6; i++)
  {
    for(int j = 0 ; j< 6; j++)
    {
      R_A_Jacobian_dot(i,j) = (R_A_Jacobian(i,j) - R_prev_A_Jacobian(i,j)) / dt;
      R_prev_A_Jacobian(i,j) = R_A_Jacobian(i,j);
    }
    QDot(i) = R_QDot(i+3);
  }

  // Current Pos & Pos_dot
  R_Foot_Pos(0) = Foot_Pos(0);
  R_Foot_Pos(1) = Foot_Pos(1);
  R_Foot_Pos(2) = Foot_Pos(2);
  R_Foot_Pos(3) = psi;
  R_Foot_Pos(4) = theta;
  R_Foot_Pos(5) = pi;
  R_Foot_Pos_dot = R_A_Jacobian*QDot;
}

void gazebo::SUBO3_plugin::Calc_CTC_Torque()
{
  VectorNd QDot;
  QDot = VectorNd::Zero(6);

  if(start_flag == 0)
  {
    Kp << 10000, 10000, 60000, 10000, 60000, 10000;
    Kv << 200, 200, 500, 200, 500, 200;
  }

  //*********************Left Leg**********************//
  VectorNd L_X_CTC;
  L_X_CTC = VectorNd::Zero(6);

  VectorNd L_q_CTC;
  L_q_CTC = VectorNd::Zero(6);

  VectorNd L_NE_Tau;
  L_NE_Tau = VectorNd::Zero(6);

  MatrixNd L_I_Matrix_tmp, L_I_Matrix;
  L_I_Matrix = MatrixNd::Zero(6,6);
  L_I_Matrix_tmp = MatrixNd::Zero(L_rbdl_model->dof_count,L_rbdl_model->dof_count);

  for(int i = 0; i < 6; i++)
  {
    L_X_CTC(i) = L_Des_XDDot(i) + Kp(i) * (L_Des_X(i) - L_Foot_Pos(i)) + Kv(i) * (L_Des_XDot(i) - L_Foot_Pos_dot(i));
    QDot(i) = L_QDot(i+3);
  }

  L_q_CTC = Inv_L_A_Jacobian * (L_X_CTC - L_A_Jacobian_dot*QDot);

  NonlinearEffects(*L_rbdl_model, L_Q, L_QDot, L_Tau, NULL);
  CompositeRigidBodyAlgorithm(*L_rbdl_model, L_Q, L_I_Matrix_tmp, true);

  for(int i = 0; i < 6; i++)
  {
    L_NE_Tau(i) = L_Tau(i+1);
    for(int j = 0; j < 6; j++)
    {
      L_I_Matrix(i,j) = L_I_Matrix_tmp(i+3,j+3);
    }
  }

  L_torque_CTC = L_I_Matrix * L_q_CTC + L_NE_Tau;

  //*********************Right Leg**********************//
  VectorNd R_X_CTC;
  R_X_CTC = VectorNd::Zero(6);

  VectorNd R_q_CTC;
  R_q_CTC = VectorNd::Zero(6);

  VectorNd R_NE_Tau;
  R_NE_Tau = VectorNd::Zero(6);

  MatrixNd R_I_Matrix_tmp, R_I_Matrix;
  R_I_Matrix = MatrixNd::Zero(6,6);
  R_I_Matrix_tmp = MatrixNd::Zero(L_rbdl_model->dof_count,L_rbdl_model->dof_count);

  for(int i = 0; i < 6; i++)
  {
    R_X_CTC(i) = R_Des_XDDot(i) + Kp(i) * (R_Des_X(i) - R_Foot_Pos(i)) + Kv(i) * (R_Des_XDot(i) - R_Foot_Pos_dot(i));
    QDot(i) = R_QDot(i+3);
  }

  R_q_CTC = Inv_R_A_Jacobian * (R_X_CTC - R_A_Jacobian_dot*QDot);

  NonlinearEffects(*R_rbdl_model, R_Q, R_QDot, R_Tau, NULL);
  CompositeRigidBodyAlgorithm(*R_rbdl_model, R_Q, R_I_Matrix_tmp, true);

  for(int i = 0; i < 6; i++)
  {
    R_NE_Tau(i) = R_Tau(i+1);
    for(int j = 0; j < 6; j++)
    {
      R_I_Matrix(i,j) = R_I_Matrix_tmp(i+3,j+3);
    }
  }

  R_torque_CTC = R_I_Matrix * R_q_CTC + R_NE_Tau;

  fprintf(tmpdata0, "%f,%f,%f,%f,%f,%f\n", L_Des_X(0),L_Des_X(1),L_Des_X(2),L_Des_X(3),L_Des_X(4),L_Des_X(5));
  fprintf(tmpdata1, "%f,%f,%f,%f,%f,%f\n", L_Foot_Pos(0),L_Foot_Pos(1),L_Foot_Pos(2),L_Foot_Pos(3),L_Foot_Pos(4),L_Foot_Pos(5));
  fprintf(tmpdata2, "%f,%f,%f,%f,%f,%f\n", L_torque_CTC(0),L_torque_CTC(1),L_torque_CTC(2),L_torque_CTC(3),L_torque_CTC(4),L_torque_CTC(5));
}

void gazebo::SUBO3_plugin::CalcBodyAngle()
{
  double calBuff;

	// 57.29578 = 180 / pi : radian to deg
	calBuff = BODY_ImuAcc(1) * BODY_ImuAcc(1) + BODY_ImuAcc(2) * BODY_ImuAcc(2);
	accel(0) = atan2(BODY_ImuAcc(0), sqrt(calBuff)) * rad2deg;
	accel(1) = atan2(BODY_ImuAcc(1), BODY_ImuAcc(2)) * rad2deg;

	// raw data * (real scale) / (data scale) -> 250 / 32768 = 0.007629394
	gyro_rate(0) = (double)BODY_ImuGyro(0) * deg2rad;
	gyro_rate(1) = (double)BODY_ImuGyro(1) * deg2rad;
	gyro_rate(2) = (double)BODY_ImuGyro(2) * deg2rad;

	// 1st Low pass filter
	LPF_BODY_ImuAcc(0) = LPF_1st(accel(0), &preLPF_BODY_ImuAcc(0), 20, dt);
	LPF_BODY_ImuAcc(1) = LPF_1st(accel(1), &preLPF_BODY_ImuAcc(1), 20, dt);

  LPF_BODY_ImuGyro(0) = LPF_1st(gyro_rate(0), &preLPF_BODY_ImuGyro(0), 20, dt);
  LPF_BODY_ImuGyro(1) = LPF_1st(gyro_rate(1), &preLPF_BODY_ImuGyro(1), 20, dt);

	imu_rate(0) = LPF_BODY_ImuGyro(0);
	imu_rate(1) = LPF_BODY_ImuGyro(1);

	LPF_BODY_ImuGyro(2) = LPF_1st(gyro_rate(2), &preLPF_BODY_ImuGyro(2), 20, dt);
	imu_rate(2) = LPF_BODY_ImuGyro(2);

  body_EAngle(0) = 0.997 * (body_EAngle(0) + imu_rate(0) * dt) + 0.003 * LPF_BODY_ImuAcc(0);
	body_EAngle(1) = 0.997 * (body_EAngle(1) + imu_rate(1) * dt) + 0.003 * LPF_BODY_ImuAcc(1);
	body_EAngle(2) = body_EAngle(2) + imu_rate(2) * dt;

  L_Q(0) = body_EAngle(1)*deg2rad;
  L_Q(1) = -body_EAngle(0)*deg2rad;
  R_Q(0) = body_EAngle(1)*deg2rad;
  R_Q(1) = -body_EAngle(0)*deg2rad;
  cout << "Body: " << L_Q(0) << ' ' << L_Q(1) << endl;
}

void gazebo::SUBO3_plugin::jointController()
{
  //* Torque Limit 감속기 정격토크참조함.
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
    New_L_Des_X(0) = msg->L_Des_X_x;New_L_Des_X(1) = msg->L_Des_X_y;New_L_Des_X(2) = msg->L_Des_X_z;
    New_L_Des_X(3) = msg->L_Des_X_rll;New_L_Des_X(4) = msg->L_Des_X_pit;New_L_Des_X(5) = msg->L_Des_X_yaw;
    New_L_Des_XDot(0) = msg->L_Des_XDot_x;New_L_Des_XDot(1) = msg->L_Des_XDot_y;New_L_Des_XDot(2) = msg->L_Des_XDot_z;
    New_L_Des_XDot(3) = msg->L_Des_XDot_rll;New_L_Des_XDot(4) = msg->L_Des_XDot_pit;New_L_Des_XDot(5) = msg->L_Des_XDot_yaw;
    New_L_Des_XDDot(0) = msg->L_Des_XDDot_x;New_L_Des_XDDot(1) = msg->L_Des_XDDot_y;New_L_Des_XDDot(2) = msg->L_Des_XDDot_z;
    New_L_Des_XDDot(3) = msg->L_Des_XDDot_rll;New_L_Des_XDDot(4) = msg->L_Des_XDDot_pit;New_L_Des_XDDot(5) = msg->L_Des_XDDot_yaw;

    New_R_Des_X(0) = msg->R_Des_X_x;New_R_Des_X(1) = msg->R_Des_X_y;New_R_Des_X(2) = msg->R_Des_X_z;
    New_R_Des_X(3) = msg->R_Des_X_rll;New_R_Des_X(4) = msg->R_Des_X_pit;New_R_Des_X(5) = msg->R_Des_X_yaw;
    New_R_Des_XDot(0) = msg->R_Des_XDot_x;New_R_Des_XDot(1) = msg->R_Des_XDot_y;New_R_Des_XDot(2) = msg->R_Des_XDot_z;
    New_R_Des_XDot(3) = msg->R_Des_XDot_rll;New_R_Des_XDot(4) = msg->R_Des_XDot_pit;New_R_Des_XDot(5) = msg->R_Des_XDot_yaw;
    New_R_Des_XDDot(0) = msg->R_Des_XDDot_x;New_R_Des_XDDot(1) = msg->R_Des_XDDot_y;New_R_Des_XDDot(2) = msg->R_Des_XDDot_z;
    New_R_Des_XDDot(3) = msg->R_Des_XDDot_rll;New_R_Des_XDDot(4) = msg->R_Des_XDDot_pit;New_R_Des_XDDot(5) = msg->R_Des_XDDot_yaw;
    
    Kp(0) = msg->Kp0;
    Kp(1) = msg->Kp1;
    Kp(2) = msg->Kp2;
    Kp(3) = msg->Kp3;
    Kp(4) = msg->Kp4;
    Kp(5) = msg->Kp5;
    Kv(0) = msg->Kv0;
    Kv(1) = msg->Kv1;
    Kv(2) = msg->Kv2;
    Kv(3) = msg->Kv3;
    Kv(4) = msg->Kv4;
    Kv(5) = msg->Kv5;
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
}

void gazebo::SUBO3_plugin::Init_Pos_Traj()
{
  Kp_q << 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500;
  Kd_q << 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5;
  step_time = 2; //주기설정 (초) 변수
  cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double periodic_function_sin = sin(2*PI/step_time*cnt_time);
  double periodic_function_cos = cos(2*PI/step_time*cnt_time);
  double Init_trajectory = (1-cos((0.5*PI)*(cnt_time/step_time)));

  CalcBodyAngle();

  if(cnt_time <= step_time)
  {
    Theo_RL_th[0] = 0*deg2rad;
    Theo_RL_th[1] = 0*deg2rad;
    Theo_RL_th[2] = -30*Init_trajectory*deg2rad;
    Theo_RL_th[3] = 60*Init_trajectory*deg2rad;
    Theo_RL_th[4] = -30*Init_trajectory*deg2rad;
    Theo_RL_th[5] = 0*deg2rad;
    Theo_LL_th[0] = 0*deg2rad;
    Theo_LL_th[1] = 0*deg2rad;
    Theo_LL_th[2] = -30*Init_trajectory*deg2rad;
    Theo_LL_th[3] = 60*Init_trajectory*deg2rad;
    Theo_LL_th[4] = -30*Init_trajectory*deg2rad;
    Theo_LL_th[5] = 0*deg2rad;
  }
  
  ///////////////토크 입력////////////////
  for (int i = 0; i < 6; i++) {
    joint[i].torque = Kp_q[i]*(Theo_RL_th[i] - actual_joint_pos[i]) + Kd_q[i] * (0 - actual_joint_vel[i]); // 기본 PV제어 코드
    joint[i+6].torque = Kp_q[i]*(Theo_LL_th[i] - actual_joint_pos[i+6]) + Kd_q[i] * (0 - actual_joint_vel[i+6]); // 기본 PV제어 코드
    old_joint[i].torque = joint[i].torque;
    old_joint[i+6].torque = joint[i+6].torque;
  }

  L_Q(3) = actual_joint_pos[0];
  L_Q(4) = actual_joint_pos[1];
  L_Q(5) = actual_joint_pos[2];
  L_Q(6) = actual_joint_pos[3];
  L_Q(7) = actual_joint_pos[4];
  L_Q(8) = actual_joint_pos[5];
  
  R_Q(3) = actual_joint_pos[6];
  R_Q(4) = actual_joint_pos[7];
  R_Q(5) = actual_joint_pos[8];
  R_Q(6) = actual_joint_pos[9];
  R_Q(7) = actual_joint_pos[10];
  R_Q(8) = actual_joint_pos[11];

  L_QDot = (L_Q - L_prevQ) / dt;
  L_QDDot = (L_QDot - L_prevQDot) / dt;
  R_QDot = (R_Q - R_prevQ) / dt;
  R_QDDot = (R_QDot - R_prevQDot) / dt;

  L_prevQ = L_Q;
  L_prevQDot = L_QDot;
  R_prevQ = R_Q;
  R_prevQDot = R_QDot;
}

void gazebo::SUBO3_plugin::Gravity_Cont()
{
  step_time = 2; //주기설정 (초) 변수
  cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = (cos((0.5*PI)*(cnt_time/step_time)));
  double new_trajectory = (1-cos((0.5*PI)*(cnt_time/step_time)));

  CalcBodyAngle();

  L_Q(3) = actual_joint_pos[0];
  L_Q(4) = actual_joint_pos[1];
  L_Q(5) = actual_joint_pos[2];
  L_Q(6) = actual_joint_pos[3];
  L_Q(7) = actual_joint_pos[4];
  L_Q(8) = actual_joint_pos[5];

  R_Q(3) = actual_joint_pos[6];
  R_Q(4) = actual_joint_pos[7];
  R_Q(5) = actual_joint_pos[8];
  R_Q(6) = actual_joint_pos[9];
  R_Q(7) = actual_joint_pos[10];
  R_Q(8) = actual_joint_pos[11];

  L_QDot = (L_Q - L_prevQ) / dt;
  L_QDDot = (L_QDot - L_prevQDot) / dt;
  R_QDot = (R_Q - R_prevQ) / dt;
  R_QDDot = (R_QDot - R_prevQDot) / dt;

  L_prevQ = L_Q;
  L_prevQDot = L_QDot;
  R_prevQ = R_Q;
  R_prevQDot = R_QDot;

  InverseDynamics(*L_rbdl_model, L_Q, VectorNd::Zero(9), VectorNd::Zero(9), L_Tau, NULL);
  InverseDynamics(*R_rbdl_model, R_Q, VectorNd::Zero(9), VectorNd::Zero(9), R_Tau, NULL);

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = old_joint[i].torque*old_trajectory + L_Tau(i+3)*new_trajectory;
      joint[i+6].torque = old_joint[i+6].torque*old_trajectory + R_Tau(i+3)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = L_Tau(i+3);
      joint[i+6].torque = R_Tau(i+3);
      old_joint[i].torque = joint[i].torque;
      old_joint[i+6].torque = joint[i+6].torque;
    }
  }
}

void gazebo::SUBO3_plugin::CTC_Control()
{
  step_time = 2; //주기설정 (초) 변수
  cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;
  
  double old_trajectory = (cos((0.5*PI)*(cnt_time/step_time)));
  double new_trajectory = (1-cos((0.5*PI)*(cnt_time/step_time)));

  CalcBodyAngle();

  L_Q(3) = actual_joint_pos[0];
  L_Q(4) = actual_joint_pos[1];
  L_Q(5) = actual_joint_pos[2];
  L_Q(6) = actual_joint_pos[3];
  L_Q(7) = actual_joint_pos[4];
  L_Q(8) = actual_joint_pos[5];

  R_Q(3) = actual_joint_pos[6];
  R_Q(4) = actual_joint_pos[7];
  R_Q(5) = actual_joint_pos[8];
  R_Q(6) = actual_joint_pos[9];
  R_Q(7) = actual_joint_pos[10];
  R_Q(8) = actual_joint_pos[11];

  L_QDot = (L_Q - L_prevQ) / dt;
  L_QDDot = (L_QDot - L_prevQDot) / dt;
  R_QDot = (R_Q - R_prevQ) / dt;
  R_QDDot = (R_QDot - R_prevQDot) / dt;

  L_prevQ = L_Q;
  L_prevQDot = L_QDot;
  R_prevQ = R_Q;
  R_prevQDot = R_QDot;

  // Target Pos, Pos Dot, Pos DDot
  L_Des_X(0) = 0;  L_Des_X(1) = 0.07;  L_Des_X(2) = -0.507;  L_Des_X(3) = 0;  L_Des_X(4) = 0;  L_Des_X(5) = 0;
  L_Des_XDot(0) = 0;  L_Des_XDot(1) = 0;  L_Des_XDot(2) = 0;  L_Des_XDot(3) = 0;  L_Des_XDot(4) = 0;  L_Des_XDot(5) = 0;
  L_Des_XDDot(0) = 0;  L_Des_XDDot(1) = 0;  L_Des_XDDot(2) = 0;  L_Des_XDDot(3) = 0;  L_Des_XDDot(4) = 0;  L_Des_XDDot(5) = 0;

  R_Des_X(0) = 0;  R_Des_X(1) = -0.07;  R_Des_X(2) = -0.507;  R_Des_X(3) = 0;  R_Des_X(4) = 0;  R_Des_X(5) = 0;
  R_Des_XDot(0) = 0;  R_Des_XDot(1) = 0;  R_Des_XDot(2) = 0;  R_Des_XDot(3) = 0;  R_Des_XDot(4) = 0;  R_Des_XDot(5) = 0;
  R_Des_XDDot(0) = 0;  R_Des_XDDot(1) = 0;  R_Des_XDDot(2) = 0;  R_Des_XDDot(3) = 0;  R_Des_XDDot(4) = 0;  R_Des_XDDot(5) = 0;

  Calc_Feedback_Pos();  // calculate the feedback
  Calc_CTC_Torque();    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = old_joint[i].torque*old_trajectory + L_torque_CTC(i)*new_trajectory;
      joint[i+6].torque = old_joint[i+6].torque*old_trajectory + R_torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = L_torque_CTC(i);
      joint[i+6].torque = R_torque_CTC(i);
      old_joint[i].torque = joint[i].torque;
      old_joint[i+6].torque = joint[i+6].torque;    
    }
  }
}

void gazebo::SUBO3_plugin::CTC_Control_Pos()
{
  step_time = 2; //주기설정 (초) 변수
  cnt_time = cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
  cnt++;

  double old_trajectory = (cos((0.5*PI)*(cnt_time/step_time)));
  double new_trajectory = (1-cos((0.5*PI)*(cnt_time/step_time)));

  CalcBodyAngle();

  L_Q(3) = actual_joint_pos[0];
  L_Q(4) = actual_joint_pos[1];
  L_Q(5) = actual_joint_pos[2];
  L_Q(6) = actual_joint_pos[3];
  L_Q(7) = actual_joint_pos[4];
  L_Q(8) = actual_joint_pos[5];

  R_Q(3) = actual_joint_pos[6];
  R_Q(4) = actual_joint_pos[7];
  R_Q(5) = actual_joint_pos[8];
  R_Q(6) = actual_joint_pos[9];
  R_Q(7) = actual_joint_pos[10];
  R_Q(8) = actual_joint_pos[11];

  L_QDot = (L_Q - L_prevQ) / dt;
  L_QDDot = (L_QDot - L_prevQDot) / dt;
  R_QDot = (R_Q - R_prevQ) / dt;
  R_QDDot = (R_QDot - R_prevQDot) / dt;

  L_prevQ = L_Q;
  L_prevQDot = L_QDot;
  R_prevQ = R_Q;
  R_prevQDot = R_QDot;

  // Target Pos, Pos Dot, Pos DDot
  if(start_flag == 0)
  {
    L_Des_X(0) = 0;  L_Des_X(1) = 0.07;  L_Des_X(2) = -0.507;  L_Des_X(3) = 0;  L_Des_X(4) = 0;  L_Des_X(5) = 0;
    L_Des_XDot(0) = 0;  L_Des_XDot(1) = 0;  L_Des_XDot(2) = 0;  L_Des_XDot(3) = 0;  L_Des_XDot(4) = 0;  L_Des_XDot(5) = 0;
    L_Des_XDDot(0) = 0;  L_Des_XDDot(1) = 0;  L_Des_XDDot(2) = 0;  L_Des_XDDot(3) = 0;  L_Des_XDDot(4) = 0;  L_Des_XDDot(5) = 0;

    R_Des_X(0) = 0;  R_Des_X(1) = -0.07;  R_Des_X(2) = -0.507;  R_Des_X(3) = 0;  R_Des_X(4) = 0;  R_Des_X(5) = 0;
    R_Des_XDot(0) = 0;  R_Des_XDot(1) = 0;  R_Des_XDot(2) = 0;  R_Des_XDot(3) = 0;  R_Des_XDot(4) = 0;  R_Des_XDot(5) = 0;
    R_Des_XDDot(0) = 0;  R_Des_XDDot(1) = 0;  R_Des_XDDot(2) = 0;  R_Des_XDDot(3) = 0;  R_Des_XDDot(4) = 0;  R_Des_XDDot(5) = 0;

    Old_L_Des_X = L_Des_X; Old_L_Des_XDot = L_Des_XDot; Old_L_Des_XDDot = L_Des_XDDot;
    Old_R_Des_X = R_Des_X; Old_R_Des_XDot = R_Des_XDot; Old_R_Des_XDDot = R_Des_XDDot;
  }
  else if(start_flag == 1)
  {
    chg_step_time = 2;
    chg_cnt_time = chg_cnt*dt; // 한스텝의 시간 설정 dt = 0.001초 고정값
    chg_cnt++;
    double change_trajectory = (1-cos((0.5*PI)*(chg_cnt_time/chg_step_time)));
    if(chg_cnt_time <= chg_step_time)
    {
      L_Des_X = Old_L_Des_X + (New_L_Des_X - Old_L_Des_X)*change_trajectory;
      L_Des_XDot = Old_L_Des_XDot + (New_L_Des_XDot - Old_L_Des_XDot)*change_trajectory;
      L_Des_XDDot = Old_L_Des_XDDot + (New_L_Des_XDDot - Old_L_Des_XDDot)*change_trajectory;

      R_Des_X = Old_R_Des_X + (New_R_Des_X - Old_R_Des_X)*change_trajectory;
      R_Des_XDot = Old_R_Des_XDot + (New_R_Des_XDot - Old_R_Des_XDot)*change_trajectory;
      R_Des_XDDot = Old_R_Des_XDDot + (New_R_Des_XDDot - Old_R_Des_XDDot)*change_trajectory;
    }
    else
    {
      L_Des_X = New_L_Des_X; L_Des_XDot = New_L_Des_XDot; L_Des_XDDot = New_L_Des_XDDot;
      R_Des_X = New_R_Des_X; R_Des_XDot = New_R_Des_XDot; R_Des_XDDot = New_R_Des_XDDot;

      Old_L_Des_X = L_Des_X; Old_L_Des_XDot = L_Des_XDot; Old_L_Des_XDDot = L_Des_XDDot;
      Old_R_Des_X = R_Des_X; Old_R_Des_XDot = R_Des_XDot; Old_R_Des_XDDot = R_Des_XDDot;

      start_flag = 2;
      chg_cnt = 0;
    }
  }

  Calc_Feedback_Pos();  // calculate the feedback
  Calc_CTC_Torque();    // calculate the CTC torque

  if(cnt_time <= step_time)
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = old_joint[i].torque*old_trajectory + L_torque_CTC(i)*new_trajectory;
      joint[i+6].torque = old_joint[i+6].torque*old_trajectory + R_torque_CTC(i)*new_trajectory;
    }
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      joint[i].torque = L_torque_CTC(i);
      joint[i+6].torque = R_torque_CTC(i);
      old_joint[i].torque = joint[i].torque;
      old_joint[i+6].torque = joint[i+6].torque;
    }
  }
}

void gazebo::SUBO3_plugin::Print() // 한 싸이클 돌때마다 데이터 플로팅
{

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
