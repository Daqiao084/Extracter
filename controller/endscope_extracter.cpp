#include <cnoid/SimpleController>
//#include <cnoid/SharedJoystick>
#include <mutex>
#include <iostream>
#include <vector>
#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include <ros/ros.h>
//#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>

//Camera
#include <cnoid/Camera>
#include <cnoid/Joystick>

//SpotLight
#include <cnoid/SpotLight>

using namespace std;
using namespace cnoid;

class endscope_sample : public SimpleController
{
public:
  ros::NodeHandle nh; //ノードを操るための変数を初期化
  ros::Subscriber joystickSubscriber;

  sensor_msgs::Joy latestJoystickState;
  // geometry_msgs::Pose svd_transform;
  // std::mutex svdMutex;
  std::mutex joystickMutex;
  sensor_msgs::Joy joystick;
  // geometry_msgs::Pose svd;

  enum TrackType { NO_TRACKS = 0, CONTINOUS_TRACKS, PSEUDO_TRACKS };
  int trackType;
  Link* trackL;
  Link* trackR;
  double trackgain;

  // vector<int> armJointIdMap;
  // vector<Link*> armJoints;
  vector<double> q_ref;
  vector<double> q_prev;
  // vector<double> pgain;
  // vector<double> dgain;
  // double* q_tip1;
  // double* q_tip2;
  // double* q_tip3;
  // double* q_tip4;

  Link::ActuationMode mainActuationMode;
  Body* body;
  Body* ikBody;
  Link* ikWrist;
  Link* ikWrist2;
  Link* ikWrist3;
  Link* ikWrist4;    //pipe
  std::shared_ptr<cnoid::JointPath> baseToWrist;
  std::shared_ptr<cnoid::JointPath> baseToWrist2;
  std::shared_ptr<cnoid::JointPath> baseToWrist3;
  std::shared_ptr<cnoid::JointPath> baseToWrist4;    //pipe

  double dt;
  double time;
  double waitTime;

  // int arm1Mode;
  // int arm2Mode;
  // int arm3Mode;
  int currentJoystickMode;
  //const int SHIFT_BUTTON = Joystick::L_BUTTON;
  int shiftState;


  Camera* camera;     //Camera: Endscope
  Camera* camera2;     //Camera2: Extracter

  Joystick Cjoystick;
  bool prevButtonState;       //Camera
  bool prevButtonState2;     //SpotLight
  bool prevButtonState3;
  bool prevButtonState4;

  std::ostream* os;

  //SpotLight
  SpotLight* light;
  SpotLight* light2;
  SpotLight* light3;
  SpotLight* light4;


  enum AxisType { STICK, BUTTON };
  enum AxisID {
    L_STICK_H_AXIS,                   //0
    L_STICK_V_AXIS,                   //1
    R_STICK_H_AXIS,                   //2
    R_STICK_V_AXIS,                   //3
    DIRECTIONAL_PAD_H_AXIS,           //4
    DIRECTIONAL_PAD_V_AXIS,           //5
    L_TRIGGER_AXIS,                   //6
    R_TRIGGER_AXIS,                   //7
    NUM_STD_AXES                      //8
  };

  enum ButtonID {
    A_BUTTON, // Cross    0
    B_BUTTON, // Circle   1
    X_BUTTON, // Square   2
    Y_BUTTON, // Triangle 3
    L_BUTTON, //          4
    R_BUTTON, //          5
    SELECT_BUTTON, //     6
    START_BUTTON,//       7
    L_STICK_BUTTON,//     8
    R_STICK_BUTTON,//     9
    LOGO_BUTTON, //       10
    NUM_STD_BUTTONS//     11
  };

  struct OperationAxis {
    Link* joint;
    AxisType type;
    int id;
    double ratio;
    int shift;
    OperationAxis(Link* joint, AxisType type, int id, double ratio, int shift = 0)
    : joint(joint), type(type), id(id), ratio(ratio), shift(shift) {
    }
  };


  vector<vector<OperationAxis> > operationAxes;
  int operationSetIndex;


  struct JointInfo {
    Link* joint;
    double q_ref;
    double q_old;
    double kp;
    double kd;
  };

  vector<JointInfo> jointInfos;
  //vector<JointInfo> jointInfos2;
  struct JointSpec {
    string name;
    double kp_torque;
    double kd_torque;
    double kp_velocity;
  };

  enum {
    MFRAME,
    BLOCK,
    BOOM,
    ARM,
    TOHKU_PITCH,
    TOHKU_ROLL,
    TOHKU_TIP_01,
    TOHKU_TIP_02,
    MNP_SWING,
    MANIBOOM,
    MANIARM,
    MANIELBOW,
    YAWJOINT,
    HANDBASE,
    PUSHROD,
    CAM_ARM2,
    CAM_ARM3,
    CAM_ARM4,
    A,
    B,
    C,
    D,
    E,
    EX_ARM2,//Pipe
    EX_A,
    JOINT1,
    JOINT3,
    EX_ARM5,
    EX_ARM6,
    ROTATION,
    SMALLEXT1,
    SMALLEXT2,
    SMALLEXT3,
    SMALLEXT4,//33
    NUM_JOINTS//必ず一番下に書く
  };


public:
  endscope_sample();

  virtual bool initialize(SimpleControllerIO* io) override;
  bool initContinuousTracks(SimpleControllerIO* io);
  bool initPseudoContinuousTracks(SimpleControllerIO* io);
  bool initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix);
  void initArms(SimpleControllerIO* io);
  void initPDGain();
  void initJoystickKeyBind();
  bool start();
  void controlTracks();
  void setTargetArmPositions();
  void controlArms();
  void controlArmsWithTorque();
  void controlArmsWithVelocity();
  void controlArmsWithPosition();

  double deg2rad(double degree);
  double rad2deg(double radian);
  Matrix3d RotateX(double radian);
  Matrix3d RotateY(double radian);
  Matrix3d RotateZ(double radian);

  void setArmIK();

  virtual bool control() override;
  void joystickCallback(const sensor_msgs::Joy& msg);
  //void svdCallback(const geometry_msgs::Pose& msg);

  Link* link(const char* name) {
    return body->link(name);
  }

};

double endscope_sample::deg2rad(double degree)
{
  return (double) (degree * M_PI / 180.0);
}

double endscope_sample::rad2deg(double radian)
{
  return (double) (radian * 180.0/ M_PI);
}
Matrix3d endscope_sample::RotateX(double radian)
{
  Matrix3d rotX = MatrixXd::Zero(3,3);
  rotX(0,0) = 1;
  rotX(1,1) = cos(radian);
  rotX(1,2) = -sin(radian);
  rotX(2,1) = sin(radian);
  rotX(2,2) = cos(radian);
  return rotX;
}

Matrix3d endscope_sample::RotateY(double radian)
{
  Matrix3d rotY = MatrixXd::Zero(3,3);
  rotY(0,0) = cos(radian);
  rotY(0,2) = sin(radian);
  rotY(1,1) = 1;
  rotY(2,0) = -sin(radian);
  rotY(2,2) = cos(radian);
  return rotY;
}

Matrix3d endscope_sample::RotateZ(double radian)
{
  Matrix3d rotZ = MatrixXd::Zero(3,3);
  rotZ(0,0) = cos(radian);
  rotZ(0,1) = -sin(radian);
  rotZ(1,0) = sin(radian);
  rotZ(1,1) = cos(radian);
  rotZ(2,2) = 1;
  return rotZ;
}

endscope_sample::endscope_sample()
{
  mainActuationMode = Link::ActuationMode::JOINT_VELOCITY;
  trackType = NO_TRACKS;
}


bool endscope_sample::initialize(SimpleControllerIO* io)
{
  body = io->body();
  ikBody = body->clone();
  ikWrist = ikBody->link("TOHKU_ROLL");
  ikWrist2 = ikBody->link("HANDBASE");
  ikWrist3 = ikBody->link("E");
  ikWrist4 = ikBody->link("JOINT3");//pipe
  Link* base = ikBody->link("MFRAME");
  Link* base2 = ikBody->link("A");
  Link* base3 = ikBody->link("EX_A");//pipe
  base->p().setZero();
  base->R().setIdentity();
  base2->p().setZero();
  base2->R().setIdentity();
  base3->p().setZero();//pipe
  base3->R().setIdentity();
  baseToWrist  = getCustomJointPath(ikBody, base, ikWrist);
  baseToWrist2 = getCustomJointPath(ikBody, base, ikWrist2);
  baseToWrist3 = getCustomJointPath(ikBody, base2, ikWrist3);
  baseToWrist4 = getCustomJointPath(ikBody, base3, ikWrist4);//pipe
  dt = io->timeStep();
  waitTime = io->currentTime();


  //Camera
  camera = io->body()->findDevice<Camera>("Camera");
  io->enableInput(camera);
  prevButtonState = false;
  os = &io->os();

  //Endscope_SpotLight
  light = io->body()->findDevice<SpotLight>("Endscope_Light");
  prevButtonState2 = false;

  //Camera2
  camera2 = io->body()->findDevice<Camera>("EX_Camera");
  io->enableInput(camera2);
  prevButtonState3 = false;

  light2 = io->body()->findDevice<SpotLight>("Left_Light");
  prevButtonState4 = false;

  light3 = io->body()->findDevice<SpotLight>("Right_Light");
  prevButtonState4 = false;

  light4 = io->body()->findDevice<SpotLight>("Front_Light");
  prevButtonState4 = false;

  // 全関節のinitialize
  string option = io->optionString();
  mainActuationMode = Link::ActuationMode::JOINT_VELOCITY;
  string prefix;
  option = "velocity";
  if(option == "velocity") {
    mainActuationMode = Link::ActuationMode::JOINT_VELOCITY;
  } else if(option  == "position") {
    mainActuationMode = Link::ActuationMode::JOINT_DISPLACEMENT;
  } else {
    mainActuationMode = Link::ActuationMode::JOINT_EFFORT;
    prefix = option;
  }


  jointInfos.clear();
  const double P_GAIN_VELOCITY = 0.3;
  vector<JointSpec> specs(NUM_JOINTS);//bodyのjointId([0]~[NUM_JOINTS - 1])動く

  //--------------------PD control--------------------//

  if(io->timeStep() < 0.02) {
    //                                          P          D           P (vel)
    specs[MFRAME      ] = { "MFRAME",         200000.0,  20000.0,   P_GAIN_VELOCITY };
    specs[BLOCK       ] = { "BLOCK",          150000.0,  15000.0,   P_GAIN_VELOCITY };
    specs[BOOM        ] = { "BOOM",           150000.0,  15000.0,   P_GAIN_VELOCITY };
    specs[ARM         ] = { "ARM",            100000.0,  10000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_PITCH ] = { "TOHKU_PITCH",     30000.0,   3000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_ROLL  ] = { "TOHKU_ROLL",      20000.0,   2000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_01] = { "TOHKU_TIP_01",      500.0,     50.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_02] = { "TOHKU_TIP_02",      500.0,     50.0,   P_GAIN_VELOCITY };
    specs[MNP_SWING   ] = { "MNP_SWING",       50000.0,   5000.0,   P_GAIN_VELOCITY };
    specs[MANIBOOM    ] = { "MANIBOOM",       100000.0,  10000.0,   P_GAIN_VELOCITY };
    specs[MANIARM     ] = { "MANIARM",        100000.0,  10000.0,   P_GAIN_VELOCITY };
    specs[MANIELBOW   ] = { "MANIELBOW",       30000.0,   3000.0,   P_GAIN_VELOCITY };
    specs[YAWJOINT    ] = { "YAWJOINT",        20000.0,   2000.0,   P_GAIN_VELOCITY };
    specs[HANDBASE    ] = { "HANDBASE",          500.0,     50.0,   P_GAIN_VELOCITY };
    specs[PUSHROD     ] = { "PUSHROD",         50000.0,   5000.0,   P_GAIN_VELOCITY };
    specs[CAM_ARM2    ] = { "CAM_ARM2",         1000.0,      100,   P_GAIN_VELOCITY };
    specs[CAM_ARM3    ] = { "CAM_ARM3",         1000.0,      100,   P_GAIN_VELOCITY };
    specs[CAM_ARM4    ] = { "CAM_ARM4",          600.0,       60,   P_GAIN_VELOCITY };
    specs[A           ] = { "A",                   0.6,     0.07,   P_GAIN_VELOCITY };
    specs[B           ] = { "B",                   3.8,     0.08,   P_GAIN_VELOCITY };
    specs[C           ] = { "C",                   2.4,     0.03,   P_GAIN_VELOCITY };
    specs[D           ] = { "D",                   2.2,     0.03,   P_GAIN_VELOCITY };
    specs[E           ] = { "E",                   2.0,    0.005,   P_GAIN_VELOCITY };
    specs[EX_ARM2     ] = { "EX_ARM2",          1000.0,      100,   P_GAIN_VELOCITY };//pipe
    specs[EX_A        ] = { "EX_A",              500.0,     50.0,   P_GAIN_VELOCITY };
    specs[JOINT1      ] = { "JOINT1",            400.0,     40.0,   P_GAIN_VELOCITY };
    specs[JOINT3      ] = { "JOINT3",            400.0,     40.0,   P_GAIN_VELOCITY };
    specs[EX_ARM5     ] = { "EX_ARM5",           300.0,       30,   P_GAIN_VELOCITY };
    specs[EX_ARM6     ] = { "EX_ARM6",           300.0,       30,   P_GAIN_VELOCITY };
    specs[ROTATION    ] = { "ROTATION",          600.0,     60.0,   P_GAIN_VELOCITY };
    specs[SMALLEXT1   ] = { "SMALLEXT1",        1000.0,      100,   P_GAIN_VELOCITY };
    specs[SMALLEXT2   ] = { "SMALLEXT2",        1000.0,      100,   P_GAIN_VELOCITY };
    specs[SMALLEXT3   ] = { "SMALLEXT3",        1000.0,      100,   P_GAIN_VELOCITY };
    specs[SMALLEXT4   ] = { "SMALLEXT4",        1000.0,      100,   P_GAIN_VELOCITY };


  } else {
    //                                          P      D      P (vel)
    specs[MFRAME      ] = { "MFRAME",         20000.0,  2000.0,   P_GAIN_VELOCITY };
    specs[BLOCK       ] = { "BLOCK",          15000.0,  1500.0,   P_GAIN_VELOCITY };
    specs[BOOM        ] = { "BOOM",           15000.0,  1500.0,   P_GAIN_VELOCITY };
    specs[ARM         ] = { "ARM",            10000.0,  1000.0,   P_GAIN_VELOCITY };
    specs[TOHKU_PITCH ] = { "TOHKU_PITCH",     3000.0,   300.0,   P_GAIN_VELOCITY };
    specs[TOHKU_ROLL  ] = { "TOHKU_ROLL",      2000.0,   200.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_01] = { "TOHKU_TIP_01",      50.0,     5.0,   P_GAIN_VELOCITY };
    specs[TOHKU_TIP_02] = { "TOHKU_TIP_02",      50.0,     5.0,   P_GAIN_VELOCITY };
    specs[MNP_SWING   ] = { "MNP_SWING",       5000.0,   500.0,   P_GAIN_VELOCITY };
    specs[MANIBOOM    ] = { "MANIBOOM",       10000.0,  1000.0,   P_GAIN_VELOCITY };
    specs[MANIARM     ] = { "MANIARM",        10000.0,  1000.0,   P_GAIN_VELOCITY };
    specs[MANIELBOW   ] = { "MANIELBOW",       3000.0,   300.0,   P_GAIN_VELOCITY };
    specs[YAWJOINT    ] = { "YAWJOINT",        2000.0,   200.0,   P_GAIN_VELOCITY };
    specs[HANDBASE    ] = { "HANDBASE",          50.0,     5.0,   P_GAIN_VELOCITY };
    specs[PUSHROD     ] = { "PUSHROD",         5000.0,   500.0,   P_GAIN_VELOCITY };
    specs[CAM_ARM2    ] = { "CAM_ARM2",         100.0,      10,   P_GAIN_VELOCITY };
    specs[CAM_ARM3    ] = { "CAM_ARM3",         100.0,      10,   P_GAIN_VELOCITY };
    specs[CAM_ARM4    ] = { "CAM_ARM4",          60.0,       6,   P_GAIN_VELOCITY };
    specs[A           ] = { "A",                  0.6,    0.07,   P_GAIN_VELOCITY };
    specs[B           ] = { "B",                  3.8,    0.08,   P_GAIN_VELOCITY };
    specs[C           ] = { "C",                  2.4,    0.03,   P_GAIN_VELOCITY };
    specs[D           ] = { "D",                  2.2,    0.03,   P_GAIN_VELOCITY };
    specs[E           ] = { "E",                  2.0,   0.005,   P_GAIN_VELOCITY };
    specs[EX_ARM2     ] = { "EX_ARM2",          100.0,      10,   P_GAIN_VELOCITY };//pipe
    specs[EX_A        ] = { "EX_A",              50.0,     4.0,   P_GAIN_VELOCITY };
    specs[JOINT1      ] = { "JOINT1",            40.0,     4.0,   P_GAIN_VELOCITY };
    specs[JOINT3      ] = { "JOINT3",            40.0,     4.0,   P_GAIN_VELOCITY };
    specs[EX_ARM5     ] = { "EX_ARM5",           30.0,     3.0,   P_GAIN_VELOCITY };//pipe
    specs[EX_ARM6     ] = { "EX_ARM6",           30.0,     3.0,   P_GAIN_VELOCITY };
    specs[ROTATION    ] = { "ROTATION",          60.0,     6.0,   P_GAIN_VELOCITY };
    specs[SMALLEXT1   ] = { "SMALLEXT1",        100.0,    10.0,   P_GAIN_VELOCITY };
    specs[SMALLEXT2   ] = { "SMALLEXT2",        100.0,    10.0,   P_GAIN_VELOCITY };
    specs[SMALLEXT3   ] = { "SMALLEXT3",        100.0,    10.0,   P_GAIN_VELOCITY };
    specs[SMALLEXT4   ] = { "SMALLEXT4",        100.0,    10.0,   P_GAIN_VELOCITY };
  }

  //--------------------PD control--------------------//

  if(!initializeJoints(io, specs, prefix)) {
    return false;
  }

  initContinuousTracks(io) || initPseudoContinuousTracks(io);
  initJoystickKeyBind();
  shiftState = 0;

  joystickSubscriber = nh.subscribe("enryu_joy",1, &endscope_sample::joystickCallback,this);


  return true;
}

bool endscope_sample::start()
{
  return true;
}


bool endscope_sample::initContinuousTracks(SimpleControllerIO* io)
{
  trackL = link("WHEEL_L0");
  trackR = link("WHEEL_R0");

  if(!trackL || !trackR) {
    return false;
  }
  trackL->setActuationMode(Link::ActuationMode::JOINT_VELOCITY);
  trackR->setActuationMode(Link::ActuationMode::JOINT_VELOCITY);

  io->enableOutput(trackL);
  io->enableOutput(trackR);
  trackType = CONTINOUS_TRACKS;
  io->os() << "Continuous tracks of " << body->name() << " are detected." << endl;

  return true;
}

bool endscope_sample::initPseudoContinuousTracks(SimpleControllerIO* io)
{
  trackL = link("TRACK_L");
  trackR = link("TRACK_R");
  if(!trackL || !trackR) {
    return false;
  }

  if(trackL->actuationMode() == Link::JOINT_SURFACE_VELOCITY && trackR->actuationMode() == Link::JOINT_SURFACE_VELOCITY) {
    io->enableOutput(trackL);
    io->enableOutput(trackR);
    trackType = PSEUDO_TRACKS;
    io->os() << "Pseudo continuous tracks of " << body->name() << " are detected." << endl;
  }

  return (trackType == PSEUDO_TRACKS);
}


bool endscope_sample::initializeJoints(SimpleControllerIO* io, vector<JointSpec>& specs, const string& prefix)
{
  for(auto& spec : specs) {
    string name = prefix + spec.name;
    auto joint = body->link(name);
    if(!joint) {
      io->os() << "test2 : " << name << endl;
      return false;
    }
    else{
      joint->setActuationMode(mainActuationMode);
      io->enableIO(joint);

      JointInfo info;
      info.joint = joint;
      info.q_ref = info.q_old = joint->q();
      if(mainActuationMode == Link::JOINT_VELOCITY) {
        info.kp = spec.kp_velocity;
      }
      else if(mainActuationMode == Link::JOINT_TORQUE) {
        info.kp = spec.kp_torque;
        info.kd = spec.kd_torque;
      }
      jointInfos.push_back(info);
    }
  }

  return true;
}

//--------------------Set operation--------------------//

void endscope_sample::initJoystickKeyBind()
{
  operationAxes = {
    {
      { link("MFRAME"),       STICK,  L_STICK_H_AXIS, -0.6 },
      { link("BLOCK"),        STICK,  R_STICK_H_AXIS, -0.6 },
      { link("BOOM"),         STICK,  L_STICK_V_AXIS, -0.6 },
      { link("ARM"),          STICK,  R_STICK_V_AXIS,  0.6 },
      { link("TOHKU_PITCH"),  BUTTON, A_BUTTON,        0.6 },
      { link("TOHKU_PITCH"),  BUTTON, Y_BUTTON,       -0.6 },
      { link("TOHKU_ROLL"),   BUTTON, X_BUTTON,        1.0 },
      { link("TOHKU_ROLL"),   BUTTON, B_BUTTON,       -1.0 },
      { link("TOHKU_TIP_01"), STICK,  R_TRIGGER_AXIS, -0.6 },
      { link("TOHKU_TIP_02"), STICK,  R_TRIGGER_AXIS, -0.6 },
      { link("TOHKU_TIP_01"), BUTTON, R_BUTTON,        0.5 },
      { link("TOHKU_TIP_02"), BUTTON, R_BUTTON,        0.5 }
    },
    {
      { link("MFRAME"),       STICK,  L_STICK_H_AXIS, -0.6    },
      { link("MNP_SWING"),    STICK,  R_STICK_H_AXIS, -0.6    },
      { link("MANIBOOM"),     STICK,  L_STICK_V_AXIS, -0.6    },
      { link("MANIARM"),      STICK,  R_STICK_V_AXIS,  0.6    },
      { link("MANIELBOW"),   BUTTON,       A_BUTTON,   0.6    },
      { link("MANIELBOW"),   BUTTON,       Y_BUTTON,  -0.6    },
      { link("YAWJOINT"),    BUTTON,       X_BUTTON,   1.0, 1 },
      { link("YAWJOINT"),    BUTTON,       B_BUTTON,  -1.0, 1 },
      { link("HANDBASE"),    BUTTON,       X_BUTTON,  -1.0, 0 },
      { link("HANDBASE"),    BUTTON,       B_BUTTON,   1.0, 0 },
      { link("PUSHROD"),      STICK, R_TRIGGER_AXIS,  -0.04   },
      { link("PUSHROD"),     BUTTON,       R_BUTTON,   0.04   }
    },
    {
      { link("CAM_ARM2"),     STICK,  R_TRIGGER_AXIS,  -0.09   },//-0.01
      { link("CAM_ARM3"),     STICK,  R_TRIGGER_AXIS,  -0.09   },//-0.01
      { link("CAM_ARM4"),     STICK,  R_TRIGGER_AXIS,  -0.09   },
      { link("CAM_ARM2"),    BUTTON,        R_BUTTON,   0.09   },//0.05
      { link("CAM_ARM3"),    BUTTON,        R_BUTTON,   0.09   },//0.05
      { link("CAM_ARM4"),    BUTTON,        R_BUTTON,   0.09   },//0.05
      { link("A"),            STICK,  L_STICK_H_AXIS,   0.6    },
      { link("B"),            STICK,  L_STICK_V_AXIS,  -0.6    },
      { link("C"),            STICK,  R_STICK_V_AXIS,  -0.6    },//-0.6
      { link("D"),           BUTTON,        Y_BUTTON,   0.6    },//-0.6
      { link("D"),           BUTTON,        A_BUTTON,  -0.6    },//-0.6
      { link("E"),            STICK,  R_STICK_H_AXIS,   1.2    }//0.6
    },
    {//pipe
      { link("MNP_SWING"),    STICK,   R_STICK_H_AXIS,  -0.6    },
      { link("EX_ARM2"),     BUTTON,         R_BUTTON,   0.6    },
      { link("EX_ARM2"),      STICK,   R_TRIGGER_AXIS,  -0.6    },
      { link("EX_A"),         STICK,   L_STICK_H_AXIS,   1.3    },
      { link("JOINT1"),       STICK,   L_STICK_V_AXIS,  -1.0    },
      { link("JOINT3"),       STICK,   R_STICK_V_AXIS,  -1.0    },
      { link("EX_ARM5"),     BUTTON,         Y_BUTTON,   0.3    },
      { link("EX_ARM6"),     BUTTON,         Y_BUTTON,   0.3    },
      { link("EX_ARM5"),     BUTTON,         A_BUTTON,  -0.3    },
      { link("EX_ARM6"),     BUTTON,         A_BUTTON,  -0.3    },
      { link("ROTATION"),    BUTTON,         B_BUTTON,   2.0    },
      { link("ROTATION"),    BUTTON,         X_BUTTON,  -2.0    },
      { link("SMALLEXT1"),   BUTTON,         L_BUTTON,   0.3    },
      { link("SMALLEXT2"),   BUTTON,         L_BUTTON,   0.3    },
      { link("SMALLEXT3"),   BUTTON,         L_BUTTON,   0.3    },
      { link("SMALLEXT4"),   BUTTON,         L_BUTTON,   0.3    },
      { link("SMALLEXT1"),    STICK,   L_TRIGGER_AXIS,  -0.3    },
      { link("SMALLEXT2"),    STICK,   L_TRIGGER_AXIS,  -0.3    },
      { link("SMALLEXT3"),    STICK,   L_TRIGGER_AXIS,  -0.3    },
      { link("SMALLEXT4"),    STICK,   L_TRIGGER_AXIS,  -0.3    }
    }
  };

  operationSetIndex = 0;
}

//--------------------Set operation--------------------//

void endscope_sample::controlTracks()
{
  trackL->u() = 0.0;
  trackL->dq() = 0.0;
  trackR->u() = 0.0;
  trackR->dq() = 0.0;

  const double k1 = 0.2;
  const double k2 = 0.4;

  double pos[2];
  pos[0] =  -joystick.axes[DIRECTIONAL_PAD_H_AXIS];// 十字キー入力
  pos[1] =   joystick.axes[DIRECTIONAL_PAD_V_AXIS];// 十字キー入力

  if(trackType == CONTINOUS_TRACKS && mainActuationMode == Link::ActuationMode::JOINT_EFFORT) {
    trackL->u() = trackgain * (-1.0 * pos[1] + pos[0]);
    trackR->u() = trackgain * (-1.0 * pos[1] - pos[0]);
  } else {
    trackL->dq_target() = (-2.0 * pos[1] - pos[0]);
    trackR->dq_target() = (-2.0 * pos[1] + pos[0]);
    //(*os) << trackR->dq_target() << " , "<<  trackL->dq_target() <<  std::endl;
  }
}

bool endscope_sample::control()
{

  {

    std::lock_guard<std::mutex> lock(joystickMutex);

    joystick = latestJoystickState;
    joystick.axes.resize(10, 0.0f);
    joystick.buttons.resize(13, 0);
  }
  if(joystick.buttons[LOGO_BUTTON] )
  {
    if((time - waitTime) > 100*dt) {
      waitTime = time;
      operationSetIndex++;
      shiftState=0;//切り替えると順運動学になる

      if(operationSetIndex == 1) {
        (*os) << "left arm" <<std::endl;
        printf("Left arm\n");
      }
      else if(operationSetIndex == 2) {
        (*os) << "Endscope" <<std::endl;
        printf("Endscope\n");
      }
      else if(operationSetIndex == 3) {
        (*os) << "Extracter" <<std::endl;
        printf("Extracter\n");
      }
      else{//pipe
        (*os) << "right arm" <<std::endl;
        printf("right arm\n");
      }

      (*os) << "Forward Kinematics" <<std::endl;
      printf("Forward Kinematics\n");

      if(operationSetIndex >= 4) {//pipe
        operationSetIndex = 0;
        shiftState=0;
      }
    }
  }
  if(trackType) {
    controlTracks();
  }
  controlArms();
  time += dt;


  Cjoystick.readCurrentState();

  //--------------------Camera--------------------//

  bool currentState = Cjoystick.getButtonState(1);
  if(currentState && !prevButtonState && operationSetIndex == 2) { //修正 operationSetIndex == 2を追加　しないとどこでも写真ををとってしまう
    const Image& image = camera->constImage();
    if(!image.empty()) {
      std::string filename = camera->name() + ".png";
      camera->constImage().save(filename);
      (*os) << "The image of " << camera->name()
      << " has been saved to \"" << filename << "\"."
      << std::endl;
    }
  }
  prevButtonState = currentState;

  //zoom updown

  bool currentState3 = joystick.buttons[4] /*Cjoystick.getButtonState(4)*/;//Lstick

  double p = 0.0;
  bool changed = false;
  if(!currentState3) {
    p = joystick.axes[6];
  } else  {
    p = -1.0;
  }

  if(fabs(p) < 0.15) {
    p = 0.0;
  }

  if(operationSetIndex == 2) {
    if(camera && fabs(p) > 0.0) {
      double fov = camera->fieldOfView();
      // (*os) << "fov is " << fov
      //       << std::endl;
      fov += radian(1.0) * p * 0.1;
      if((fov > radian(0.0)) && (fov < radian(90.0))) {
        camera->setFieldOfView(fov);
        changed = true;
      }
      if(changed) {
        camera->notifyStateChange();
      }
    }
  }

  prevButtonState3 = currentState3;
  //--------------------Camera--------------------//

  //--------------------SpotLight--------------------//

  bool currentState2 = joystick.buttons[2]; // □ボタン以外false
  if(currentState2 &&!prevButtonState2 && operationSetIndex == 2) {
    light->on(!light->on());
    changed = true;
  }

  prevButtonState2 = currentState2;

  if(changed) {
    light->notifyStateChange();
  }

  bool currentState4 = joystick.buttons[4]; // Lボタン以外false
  if(currentState4 &&!prevButtonState4 && operationSetIndex == 0) {
    light2->on(!light2->on());
    light3->on(!light3->on());
    light4->on(!light4->on());
    changed = true;
  }

  prevButtonState4 = currentState4;

  if(changed) {
    light2->notifyStateChange();
    light3->notifyStateChange();
    light4->notifyStateChange();
  }


  //--------------------SpotLight--------------------//



  return true;
}


void endscope_sample::setTargetArmPositions()
{
  const vector<OperationAxis>& axes = operationAxes[operationSetIndex];

  for(auto& axis : axes) {
    if(axis.shift < 0 || axis.shift == shiftState) {
      auto joint = axis.joint;
      auto& q = jointInfos[joint->jointId()].q_ref;//jointId()はボディファイルで指定した値
      if(axis.type == BUTTON) {
        if(joystick.buttons[axis.id]) {
          q += axis.ratio * dt;
        }
      } else if(axis.type == STICK) {
        auto pos = joystick.axes[axis.id];
        q += axis.ratio * pos * dt;
      }
    }
  }
}

//--------------------Inverse kinematics--------------------//

void endscope_sample::setArmIK()
{
  // 手首座標
  ikBody   = body->clone();
  ikWrist  = ikBody->link("TOHKU_ROLL");
  ikWrist2 = ikBody->link("HANDBASE");
  ikWrist3 = ikBody->link("E");
  ikWrist4 = ikBody->link("JOINT3");//pipe
  Link* base  = ikBody->link("MFRAME");
  Link* base2 = ikBody->link("A");
  Link* base3 = ikBody->link("EX_A");
  base->p().setZero();
  base->R().setIdentity();
  base2->p().setZero();
  base2->R().setIdentity();
  base3->p().setZero();//pipe
  base3->R().setIdentity();
  baseToWrist = getCustomJointPath(ikBody, base, ikWrist);
  baseToWrist2 = getCustomJointPath(ikBody, base, ikWrist2);
  baseToWrist3 = getCustomJointPath(ikBody, base2, ikWrist3);
  baseToWrist4 = getCustomJointPath(ikBody, base3, ikWrist4);//pipe
  baseToWrist->calcForwardKinematics();
  baseToWrist2->calcForwardKinematics();
  baseToWrist3->calcForwardKinematics();
  baseToWrist4->calcForwardKinematics();//pipe

  Vector3d p =   ikWrist->p();
  Matrix3d R =   ikWrist->R();
  Vector3d p2 =  ikWrist2->p();
  Matrix3d R2 =  ikWrist2->R();
  Vector3d p3 =  ikWrist3->p();
  Matrix3d R3 =  ikWrist3->R();
  Vector3d p4 =  ikWrist4->p();//pipe
  Matrix3d R4 =  ikWrist4->R();

  //MatrixXd Jacbian;
  double dq_fingerL = 0.0;
  double ltL = 0.0;
  Link* TohkuTipJoint = body->link("TOHKU_TIP_02");
  Link* Pushrod = body->link("PUSHROD");
  double maxerror;//?
  double q_current;
  double q_lower;
  double q_upper;
  double q_lower2;
  double q_upper2;
  double dAngle;
  double baseDRot;

  switch(operationSetIndex) {
    case 0: //Right arm
    p(1) -= joystick.axes[0]/200; //L_STICK_H_AXIS
    p(0) -= joystick.axes[1]/200; //L_STICK_V_AXIS
    p(2) -= joystick.axes[3]/200; //  R_STICK_V_AXIS
    if(joystick.buttons[1]) { R *= RotateZ(deg2rad(1));  }//B_BUTTON Circle
    if(joystick.buttons[2]) { R *= RotateZ(deg2rad(-1)); } //X_BUTTON  Square
    if(joystick.buttons[0]) { R *= RotateY(deg2rad(1));  }//A_BUTTON  Cross
    if(joystick.buttons[3]) { R *= RotateY(deg2rad(-1)); } // Y_BUTTON   Triangle

    // **Control of TOHKU_TIP
    ltL = joystick.axes[7]; //R_TRIGGER_AXIS
    dq_fingerL -= ltL;
    ltL = joystick.buttons[5]; //R_BUTTON
    dq_fingerL += ltL;
    if(mainActuationMode == Link::ActuationMode::JOINT_EFFORT) {
      maxerror = 20.0;
    } else {
      maxerror = 0.01;
    }
    q_current = TO_DEGREE*TohkuTipJoint->q();
    q_lower = -40.0;
    q_upper = 0.0;
    q_lower2 = std::max(q_current - maxerror, q_lower);
    q_upper2 = std::min(q_current + maxerror, q_upper);
    if(q_current < q_lower2) {
      dq_fingerL = 0.1;
    }
    if(q_current > q_upper2) {
      dq_fingerL = -0.1;
    }
    jointInfos[TohkuTipJoint->jointId()].q_ref += 0.005*dq_fingerL;
    jointInfos[TohkuTipJoint->jointId()].q_ref += 0.005*dq_fingerL;
    // **Control of MFRAME
    dAngle = joystick.axes[6]; //L_TRIGGER_AXIS
    baseDRot += dAngle;
    dAngle = joystick.buttons[4]; //L_BUTTON
    baseDRot -= dAngle;
    jointInfos[base->jointId()].q_ref += 0.01*baseDRot;
    break;

    case 1: //Left arm
    p2(1) -= joystick.axes[0]/200;
    p2(0) -= joystick.axes[1]/200;
    p2(2) -= joystick.axes[3]/200;
    if(joystick.buttons[1]) { R2 *= RotateZ(deg2rad(-1)); }
    if(joystick.buttons[2]) { R2 *= RotateZ(deg2rad(1));  }
    if(joystick.buttons[0]) { R2 *= RotateY(deg2rad(1));  }
    if(joystick.buttons[3]) { R2 *= RotateY(deg2rad(-1)); }
    if(joystick.axes[6])   { R2 *= RotateX(deg2rad(-1)); }
    if(joystick.buttons[4]) { R2 *= RotateX(deg2rad(1)); }


    // **Control of PUSHROD
    ltL = joystick.axes[7];
    dq_fingerL -= ltL;
    ltL = joystick.buttons[5];
    dq_fingerL += ltL;
    if(mainActuationMode == Link::ActuationMode::JOINT_EFFORT) {
      maxerror = 20.0;
    } else {
      maxerror = 0.01;
    }
    q_current = TO_DEGREE*TohkuTipJoint->q();

    q_upper = 0.0;
    q_lower2 = std::max(q_current - maxerror, q_lower);
    q_upper2 = std::min(q_current + maxerror, q_upper);
    if(q_current < q_lower2) {
      dq_fingerL = 0.1;
    }
    if(q_current > q_upper2) {
      dq_fingerL = -0.1;
    }
    jointInfos[Pushrod->jointId()].q_ref += 0.0008*dq_fingerL;
    break;

    case 2://Endscope
    p3(1) -= joystick.axes[0]/200;
    p3(0) -= joystick.axes[1]/200;
    p3(2) -= joystick.axes[3]/200;
    if(joystick.buttons[0]) { R3 *= RotateY(deg2rad(1));  }
    if(joystick.buttons[3]) { R3 *= RotateY(deg2rad(-1)); }

    if(mainActuationMode == Link::ActuationMode::JOINT_EFFORT) {
      maxerror = 20.0;
    } else {
      maxerror = 0.01;
    }
    break;

    case 3://pipe
    p4(1) -= joystick.axes[0]/200;
    p4(0) -= joystick.axes[1]/200;
    p4(2) -= joystick.axes[3]/200;
    //if(joystick.axes[3]) { R4 *= RotateX(deg2rad(1)); }
    //if(joystick.buttons[4]) { R4 *= RotateY(deg2rad(1));  }

    if(mainActuationMode == Link::ActuationMode::JOINT_EFFORT) {
      maxerror = 20.0;
    } else {
      maxerror = 0.01;
    }
    break;

    default:
    break;
  }

  baseToWrist->calcInverseKinematics(p, R);
  for(int i = 0; i <5; i++ )
  {
    Link* joint = baseToWrist->joint(i);
    jointInfos[joint->jointId()].q_ref = joint->q();
  }

  baseToWrist2->calcInverseKinematics(p2, R2);
  for(int i = 0; i <6; i++ )
  {
    Link* joint = baseToWrist2->joint(i);
    jointInfos[joint->jointId()].q_ref = joint->q();
  }

  baseToWrist3->calcInverseKinematics(p3, R3);
  for(int i = 0; i < 4; i++ )
  {
    Link* joint = baseToWrist3->joint(i);
    jointInfos[joint->jointId()].q_ref = joint->q();
  }

  baseToWrist4->calcInverseKinematics(p4, R4);//pipe
  for(int i = 0; i < 2; i++ )
  {
    Link* joint = baseToWrist4->joint(i);
    jointInfos[joint->jointId()].q_ref = joint->q();
  }

}

//--------------------Inverse kinematics--------------------//

//--------------------Pop out forward or inverse kinematics--------------------//

void endscope_sample::controlArms()
{
  if(joystick.buttons[START_BUTTON] ) //START_BUTTON
  {
    if((time - waitTime) > 100*dt) {
      waitTime = time;
      shiftState++;
      if(shiftState == 0) {
        (*os) << "Forward kinematics" <<std::endl;
        printf("Forward kinematics\n");
      }else if(shiftState == 1) {
        (*os) << "Inverse kinematics" <<std::endl;
        printf("Inverse kinematics\n");
      }
      if(shiftState >= 2) {
        shiftState = 0;
        (*os) << "Forward kinematics" <<std::endl;
        printf("Forward kinematics\n");
      }
    }
  }

  //--------------------Pop out forward or inverse kinematics--------------------//

  //--------------------Store joint information--------------------//

  for(auto& info : jointInfos) {
    Link* joint = info.joint;
    info.q_ref = joint->q();
  }

  //--------------------Store joint information--------------------//


  if(shiftState == 1 ) {setArmIK();}
  else{setTargetArmPositions();}

  //--------------------Switch ActuationMode--------------------//

  switch(mainActuationMode) {
    case Link::ActuationMode::JOINT_DISPLACEMENT:
    controlArmsWithPosition();
    break;
    case Link::ActuationMode::JOINT_VELOCITY:
    controlArmsWithVelocity();
    break;
    case Link::ActuationMode::JOINT_EFFORT:
    controlArmsWithTorque();
    break;
    default:
    break;
  }
}

//--------------------Switch ActuationMode--------------------//

void endscope_sample::controlArmsWithPosition()
{
  for(auto& info : jointInfos) {
    info.joint->q_target() = info.q_ref;
  }
}


void endscope_sample::controlArmsWithVelocity()
{
  for(auto& info : jointInfos) {
    auto joint = info.joint;
    double q = joint->q();
    joint->dq_target() = info.kp * (info.q_ref - q) / dt;
  }
}


void endscope_sample::controlArmsWithTorque()
{
  for(auto& info : jointInfos) {
    auto joint = info.joint;
    double q = joint->q();
    double dq = (q - info.q_old) / dt;
    joint->u() = info.kp * (info.q_ref - q) + info.kd * (0.0 - dq);
    info.q_old = q;
  }
}

void endscope_sample::joystickCallback(const sensor_msgs::Joy& msg)
{
  std::lock_guard<std::mutex> lock(mutex);
  latestJoystickState = msg;
}

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(endscope_sample)
