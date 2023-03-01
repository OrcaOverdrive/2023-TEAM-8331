#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include "frc/DoubleSolenoid.h"
#include "frc/Compressor.h"
#include "rev/CANSparkMax.h"
#include "rev/SparkMaxPIDController.h"
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <iostream>
#include "cameraserver/CameraServer.h"
#include <frc/motorcontrol/Spark.h>

class MotorGroup {
 public:
  MotorGroup(frc::Spark& motor1, frc::Spark& motor2, frc::Spark& motor3, frc::Spark& motor4)
      : motor1_(motor1), motor2_(motor2), motor3_(motor3), motor4_(motor4) {}

  void SetSpeeds(double left_speed, double right_speed) {
    motor1_.Set(left_speed);
    motor2_.Set(left_speed);
    motor3_.Set(right_speed);
    motor4_.Set(right_speed);
  }

 private:
  frc::Spark& motor1_;
  frc::Spark& motor2_;
  frc::Spark& motor3_;
  frc::Spark& motor4_;
};

class Robot : public frc::TimedRobot {
 public:
  Robot() {
    // Drivetrain
    m_right.SetInverted(true);
    m_right2.SetInverted(true);
    m_robotDrive.SetExpiration(100_ms);

    //PNEUMATICS
    pcmCompressor.EnableDigital();
    //Initialize DoubleSolonoid so it knows where to start.
    DoublePCM.Set(frc::DoubleSolenoid::Value::kForward);

    m_timer.Start();
  }

  void RobotInit() override {
    frc::CameraServer::StartAutomaticCapture();
  }
  
  void AutonomousInit() override {
    m_timer.Reset();
    m_timer.Start();
  }
    
    // AUTONOMOUS
  void AutonomousPeriodic() override {
   
    // Drive Backwards and puts arm down
    if (m_timer.Get() < 4.5_s) {
      motor_group.SetSpeeds(0.5, 0.5);
      //m_arm.Set(-0.5);

    } else if (m_timer.Get() > 4.5_s && m_timer.Get() < 5_s){
     
      // Stop robot
      motor_group.SetSpeeds(0.0, 0.0);
      //m_arm.Set(0);
    }

    // Drive forwards and lift claw up.

  }

  void TeleopInit() override {}

  void TeleopPeriodic() override {
    // change speed to 50%
    if(m_controller.GetBButtonPressed()){
      if(speedfactor==1)speedfactor=0.5;
      else speedfactor=1;
    } 

    // Drive with tank style
    double leftspeed = -m_controller.GetLeftY();
    double rightspeed = -m_controller.GetRightY();

    motor_group.SetSpeeds(leftspeed*speedfactor, rightspeed*speedfactor);


    // if x button pressed, toggle doubele solenoid
    if (m_controller.GetXButtonPressed()) {DoublePCM.Toggle();}

 // elevator up
    if (m_controller.GetYButtonPressed() && (elevator_switch_upper.Get())) {m_elevator.Set(1);}
    if (m_controller.GetYButtonReleased()) {m_elevator.Set(0);}

    //stop moving when hits upper switch
    if (!elevator_switch_upper.Get() && (m_elevator.Get() > 0)) {
      m_elevator.Set(0);
      }

    //elevator down
    if (m_controller.GetAButtonPressed() && (!elevator_switch_lower.Get())) {m_elevator.Set(-1);}
    if (m_controller.GetAButtonReleased()) {m_elevator.Set(0);}

    //stop moving when hits lower switch
        if (elevator_switch_lower.Get() && (m_elevator.Get() < 0)) 
        {m_elevator.Set(0);
        } 


    // Arm up and down
    if (m_controller.GetPOV() == 0) {m_arm.Set(0.5);}
    if (m_controller.GetPOV() == 180) {m_arm.Set(-0.5);}
    if (m_controller.GetPOV() == -1) {m_arm.Set(0);}

  }

  void TestInit() override {}

  void TestPeriodic() override {}

 private:
  // Robot drive system
  frc::Spark m_left{1};
  frc::Spark m_right{0};
  frc::Spark m_right2{2};
  frc::Spark m_left2{3};
  MotorGroup motor_group{m_left, m_left2, m_right, m_right2};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};  

  double speedfactor=1;

  // controller
  frc::XboxController m_controller{0};

  //Switch lower 
  frc::DigitalInput elevator_switch_lower{0};
  //Switch upper
  frc::DigitalInput elevator_switch_upper{1};

  //CANbus elevator
  rev::CANSparkMax m_elevator{21, rev::CANSparkMax::MotorType::kBrushless};

  //CANbus arm
  rev::CANSparkMax m_arm{24, rev::CANSparkMax::MotorType::kBrushless};

  //pneumatics
  frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
  bool pressureSwitch = pcmCompressor.GetPressureSwitchValue();
  double current = (double) pcmCompressor.GetCurrent();

  // Double Solenoid, 0, 1 - ports
  frc::DoubleSolenoid DoublePCM{frc::PneumaticsModuleType::CTREPCM, 0, 1};

  //timers
  frc::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
