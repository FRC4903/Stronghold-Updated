#include <frc/Timer.h>
#include "frc/TimedRobot.h"
#include "rev/CANSparkMax.h"
#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartDashboard/SendableChooser.h>
#include "ctre/Phoenix.h"
#include <frc/Encoder.h>
#include <frc/Joystick.h>
#include "cameraserver/CameraServer.h"
#include <string_view>
#include <frc/Ultrasonic.h>


using namespace frc;
using namespace std;
using namespace rev;

class Robot: public TimedRobot {
  public:
  //Joystick axes
  int joy_x2 = 0;
  int joy_y1 = 5;
  int joystick_2 = 1;
  int but_lb = 5;
  int but_rb = 6;
  //Speed limitations
  float speedLimit = 0.5;
  float turnLimit = 0.5;
  //Coast control
  int coast = 5;
  bool slowmode=false;
  bool fastmode=false;
  //left
  TalonSRX left_1;
  TalonSRX left_2;
  //right
  TalonSRX right_1;
  TalonSRX right_2;
  //joystick
  Joystick controller0 {0}; //Drive Controller using fs i6
  Joystick controller1 {1}; //Shoot Controller is still the same
  //Old speeds for speed calculation
  float oldLeft=0;
  float oldRight=0;
  //Intake/Conveyer
  TalonSRX intake;
  TalonSRX conveyer;
  //Shooters
  CANSparkMax shoot1{5, CANSparkMax::MotorType::kBrushless};
  CANSparkMax shoot2{6, CANSparkMax::MotorType::kBrushless};
  rev::SparkMaxRelativeEncoder en1 = shoot1.GetEncoder();
  Timer *gameTimer = new Timer();

  Ultrasonic *ultra = new Ultrasonic(0,1); 
  // Ultrasonic ultra{0,1};
  double power = 0.1;
  //construct
  enum AutoOptions { Auto1, Auto2, Auto3, Auto4, Auto5 }; 
  SendableChooser<AutoOptions> m_chooser;
  Robot():
    left_1(1),
    left_2(2),
    right_1(3),
    right_2(4),
    intake(7),
    conveyer(8)
  {}
  void RobotInit() {
    
    //initializing
    left_1.Set(ControlMode::PercentOutput, 0);
    left_2.Set(ControlMode::PercentOutput, 0);
    right_1.Set(ControlMode::PercentOutput, 0);
    right_2.Set(ControlMode::PercentOutput, 0);
    intake.Set(ControlMode::PercentOutput, 0);
    conveyer.Set(ControlMode::PercentOutput, 0);

    CameraServer::GetInstance()->StartAutomaticCapture();

    //chooser stuff
    
    
  }
  //the names explain idk what else you want
  void RobotPeriodic() {}
  void TestInit(){
    
   }
  void TestPeriodic(){
    
   
  }
  void AutonomousInit() override {
    gameTimer -> Start();
    gameTimer -> Reset();
   
    ultra ->SetAutomaticMode(true);
    SmartDashboard::PutNumber("Power",0);
    SmartDashboard::PutBoolean("Switch",false);
    SmartDashboard::PutBoolean("Conveyer",false);
  }

  void AutonomousPeriodic() override {

    float y1 ;
    float y2 ;
    AutoOptions m_Selection = m_chooser.GetSelected();
    if (m_Selection == AutoOptions::Auto1){
      double time1 = SmartDashboard::GetNumber("turn first",0);
    SmartDashboard::PutNumber("turn first periodic copy",time1);
    // double timesCopy[5] = frc::SmartDashboard::GetNumberArray("times",0);
    if(gameTimer ->Get() < units::second_t{time1}){
      y1 = -1.0;
      y2 = 1.0;
      SmartDashboard::PutBoolean("status",true);
    }
    else{
      y1 = 0;
      y2 = 0;
      SmartDashboard::PutBoolean("status",false);
      
      }

    }else if(m_Selection == AutoOptions::Auto2){
      double time1 = SmartDashboard::GetNumber("go straight",0);
      if (gameTimer -> Get() < units::second_t{time1}) {
      // Drive forwards half speed
      y1 = 1.0*0.844;
      y2 = 1.0;
      intake.Set(ControlMode::PercentOutput, -1);
      conveyer.Set(ControlMode::PercentOutput, 1);
    } else if (gameTimer -> Get() < units::second_t{time1+0.7} && gameTimer -> Get() > units::second_t{time1}){
      y1 = 0;
      y2 = 0;
      }
    }else if(m_Selection == AutoOptions::Auto3){
      bool init=true;
      bool f1=false;
      bool t1=false;
      bool t2=false;
      bool f2=false;
      bool shoot=false;
      bool complete=false;
      double dist = ultra -> GetRange().value();
      double target = SmartDashboard::GetNumber("straight meters",0);
      double target2 = SmartDashboard::GetNumber("straight meters 2",0);
      SmartDashboard::PutNumber("ultra test",dist);
      if (dist <= target && init){
        y1 = 1.0*0.844;
        y2 = 1.0;
      }else if(dist < 3.3 && init){
        y1 = 0;
        y2 = 0;
        f1=true;
        init=false;
      }else if (dist >= 2.7 && f1){
        y1 = -1.0;
        y2 = 1.0;
      }else if (dist < 2.7 && f1){
        y1 = 0;
        y2 = 0;
        f1=false;
        t1=true;
      }else if (dist >= target && t1){
        y1 = -1.0*0.844;
        y2 = -1.0;
      }else if(dist > 0.5 && t1){
        y1 = 0;
        y2 = 0;
        t2=true;
        t1=false;
      }
    }
    
    SmartDashboard::PutNumber("Ultrasonic Inches", ultra -> GetRange().value());
    
    double p = frc::SmartDashboard::GetNumber("Power", 0);
    
    shoot1.Set(p);
    shoot2.Set(-p);
    SmartDashboard::PutNumber("shoot1 rpm", en1.GetVelocity());
    SmartDashboard::PutNumber("shoot1 pos", en1.GetPosition());
    

    if(SmartDashboard::GetBoolean("Conveyer",false)){conveyer.Set(ControlMode::PercentOutput, 1);}
    else{conveyer.Set(ControlMode::PercentOutput, 0);}

    // Drive for 2 seconds
    // if (gameTimer -> Get() < units::second_t{time1}) {
    //   // Drive forwards half speed
    //   y1 = 1.0*0.844;
    //   y2 = 1.0;
    //   intake.Set(ControlMode::PercentOutput, -1);
    //   conveyer.Set(ControlMode::PercentOutput, 1);
    // } else if (gameTimer -> Get() < 3.7_s && gameTimer -> Get() > 3_s){
    //   y1 = 0;
    //   y2 = 0;
    // } else if (3.7_s < gameTimer -> Get() && gameTimer -> Get() < 4.37_s){

    //   y1 = -1.0;
    //   y2 = 1.0;
    //   conveyer.Set(ControlMode::PercentOutput, 0);
    // } else if (4.385_s < gameTimer -> Get() && gameTimer -> Get() < 5.585_s){
    //   y1=0;
    //   y2=0;
    //   conveyer.Set(ControlMode::PercentOutput, 1);
    //   shoot1.Set(0.8);
    //   shoot2.Set(-0.8);
    // } else {
    //   // Stop robot
    //   shoot1.Set(0);
    //   shoot2.Set(0);
    //   intake.Set(ControlMode::PercentOutput, 0);
    //   conveyer.Set(ControlMode::PercentOutput, 0);
    // }

    float LeftSpeed = max(-1.0f, min(1.0f, (-y1/40+oldLeft*39/40)));//neo side
    float RightSpeed = max(-1.0f, min(1.0f, (y2/40+oldRight*39/40)));//battery side
    
    //set motor output speeds yay
    left_1.Set(ControlMode::PercentOutput, LeftSpeed*speedLimit);
    right_1.Set(ControlMode::PercentOutput, RightSpeed*speedLimit);
    left_2.Set(ControlMode::PercentOutput, LeftSpeed*speedLimit);
    right_2.Set(ControlMode::PercentOutput, RightSpeed*speedLimit);
    //setting old speeds to current speed for next speed calc
    oldLeft=LeftSpeed;
    oldRight=RightSpeed;
  }


  void TeleopInit() {
   
    SmartDashboard::PutNumber("turn first",0.67);
    SmartDashboard::PutNumber("go straight",3);
    SmartDashboard::PutNumber("straight meters",2.8);
    m_chooser.SetDefaultOption("auto1", AutoOptions::Auto1);
    m_chooser.AddOption("auto2", AutoOptions::Auto2);
    m_chooser.AddOption("auto3", AutoOptions::Auto3);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    
  }

  void TeleopPeriodic() {
    //joystick axes, y1 and x for arcade, y1 and y2 for separate joystick controls
    float y1 = controller0.GetRawAxis(joy_y1);
    float y2 = controller0.GetRawAxis(joystick_2);
    float x = controller0.GetRawAxis(joy_x2);
    //if the joysticks are just barely offset, it wont case another accident
    y1 = (abs(y1)<=0.05)? 0 : -y1;
    y2 = (abs(y2)<=0.05)? 0 : -y2;
    x = (abs(x)<=0.05)? 0 : x;

    //gets input to shoot or intake
    bool doOuttake = controller1.GetRawButton(but_rb);
    bool doIntake = controller1.GetRawButton(but_lb);
    // //Sets shoot and intake to zero before setting with buttons
    conveyer.Set(ControlMode::PercentOutput, 0);
    if (controller1.GetRawAxis(joy_y1)<-0.1){conveyer.Set(ControlMode::PercentOutput, 1);}
    if (controller1.GetRawAxis(joy_y1)>0.1){conveyer.Set(ControlMode::PercentOutput, -1);}
    intake.Set(ControlMode::PercentOutput, 0);
    shoot1.Set(0);
    shoot2.Set(0);
    // //shoots and intakes
    if (doIntake){
      intake.Set(ControlMode::PercentOutput, -1);
    }
    if (doOuttake){
      intake.Set(ControlMode::PercentOutput, 1);
    }
    shoot1.Set((controller1.GetRawAxis(3)+controller1.GetRawAxis(2))/2);
    shoot2.Set(-(controller1.GetRawAxis(3)+controller1.GetRawAxis(2))/2);
    //Determines whether to limit speed and modify coast
    speedLimit=0.5;
    turnLimit=0.5;
    coast=5;
    slowmode=controller0.GetRawButton(5);
    SmartDashboard::PutBoolean("B5", controller0.GetRawButton(5));
    SmartDashboard::PutBoolean("B6", controller0.GetRawButton(6));
    if (controller0.GetRawButton(5)){speedLimit=0.25;
      turnLimit=0.25;}
    //axis(3) is the switch C on fs i6
    if (controller0.GetRawButton(6)){
      speedLimit=1;
      coast=10;}

    //ARCADE DRIVE: alters speeds of motors to constrain within -1 and 1, slowly ramps speeds up and down so it's not jerky


    if(controller0.GetRawButton(4)){
      y1=-y1;
    }

    x=-x;

    float LeftSpeed = max(-1.0f, min(1.0f, ((y1*abs(y1)+x*abs(x)))/coast)+(oldLeft*(coast-1)/coast));
    float RightSpeed = max(-1.0f, min(1.0f, -1*((y1*abs(y1)-x*abs(x))/coast)+(oldRight*(coast-1)/coast)));

    
    
    //SEPARATE JOYSTICK DRIVE: similar idea, now takes separate sticks instead of using x value
    //float LeftSpeed = max(-1.0f, min(1.0f, (-y1/5+oldLeft*4/5)));
    //float RightSpeed = max(-1.0f, min(1.0f, (y2/5+oldRight*4/5)));
    
    //set motor output speeds yay
    left_1.Set(ControlMode::PercentOutput, -1*LeftSpeed*speedLimit);
    right_1.Set(ControlMode::PercentOutput, -1*RightSpeed*speedLimit);
    left_2.Set(ControlMode::PercentOutput, -1*LeftSpeed*speedLimit);
    right_2.Set(ControlMode::PercentOutput, -1*RightSpeed*speedLimit);
    //setting old speeds to current speed for next speed calc
    oldLeft=LeftSpeed;
    oldRight=RightSpeed;
    
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return StartRobot<Robot>();
}
#endif