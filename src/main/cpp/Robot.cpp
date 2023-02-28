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
 
//frc::CameraServer::StartAutomaticCapture{};
//cs::CvSink cvSink = frc::CameraServer::GetVideo();
//cs::CvSource outputStream = frc::CameraServer::PutVideo("Blur",640,480); 

class Robot : public frc::TimedRobot {
 public:
  Robot() {
    // Drivetrain
    m_right.SetInverted(true);
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

  void AutonomousPeriodic() override {
    // Drive for 2 seconds
    if (m_timer.Get() < 2_s) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.ArcadeDrive(0.5, 0.0, false);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    }
  }

  void TeleopInit() override {}

  void TeleopPeriodic() override {
    // Drive with arcade style (use right stick to steer)s
    double leftspeed = -m_controller.GetLeftY();
    double rightspeed = -m_controller.GetRightY();
    if(m_controller.GetBackButtonPressed()){
      if(speedfactor==1)speedfactor=0.5;
      else speedfactor=1;
    }
    m_left.Set(leftspeed*speedfactor);
    m_right.Set(rightspeed*speedfactor);

    // if x button pressed, toggle doubele solenoid
    if (m_controller.GetXButtonPressed()) {DoublePCM.Toggle();}

 // elevator up
    if (m_controller.GetYButtonPressed() && (elevator_switch_upper.Get())) {m_elevator.Set(1);}
    if (m_controller.GetYButtonReleased()) {m_elevator.Set(0);}

    //stop moving when hits upper switch
    if (!elevator_switch_upper.Get() && (m_elevator.Get() > 0)) {
      m_elevator.Set(0);
      m_controller.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
      }

    //elevator down
    if (m_controller.GetAButtonPressed() && (!elevator_switch_lower.Get())) {m_elevator.Set(-1);}
    if (m_controller.GetAButtonReleased()) {m_elevator.Set(0);}

    //stop moving when hits lower switch
        if (elevator_switch_lower.Get() && (m_elevator.Get() < 0)) 
        {m_elevator.Set(0);
        m_controller.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);
        } 


    // Arm up and down
    if (m_controller.GetPOV() == 0) {m_arm.Set(0.5);}
    if (m_controller.GetPOV() == 180) {m_arm.Set(-0.5);}
    if (m_controller.GetPOV() == -1) {m_arm.Set(0);}

    if (m_controller.GetBButtonPressed()) {m_controller.SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 1.0);}

  }

  void TestInit() override {}

  void TestPeriodic() override {}

 private:
  // Robot drive system
  frc::PWMSparkMax m_left{1};
  frc::PWMSparkMax m_right{0};
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
