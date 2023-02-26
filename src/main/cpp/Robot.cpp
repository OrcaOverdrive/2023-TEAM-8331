
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


class Robot : public frc::TimedRobot {
 public:
  Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_left.SetInverted(true);
    m_robotDrive.SetExpiration(100_ms);




    pcmCompressor.EnableDigital();
    //pcmCompressor.Disable();
    //Initialize DoubleSolonoid so it knows where to start.
    DoublePCM.Set(frc::DoubleSolenoid::Value::kForward);

    m_timer.Start();
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
    // Drive with arcade style (use right stick to steer)
    m_robotDrive.ArcadeDrive(-m_controller.GetLeftY()*0.75,
                             m_controller.GetRightX()*0.65);

    // if y button pressed, toggle doubele solenoid
    if (m_controller.GetXButtonPressed()) {DoublePCM.Toggle();}


    // elevator up and down
    if (m_controller.GetYButtonPressed() && (!elevator_switch_upper.Get())) {m_elevator.Set(1);}
    if (m_controller.GetYButtonReleased()) {m_elevator.Set(0);}
    if ((elevator_switch_upper.Get())) {m_elevator.Set(0);}

    if (m_controller.GetAButtonPressed() && (!elevator_switch_lower.Get())) {m_elevator.Set(-1);}
    if (m_controller.GetAButtonReleased()) {m_elevator.Set(0);}
    if ((elevator_switch_lower.Get())) {m_elevator.Set(0);}
    
    // Arm up and down
    if (m_controller.GetPOV() == 0) {m_arm.Set(0.2);}
    if (m_controller.GetPOV() == 180) {m_arm.Set(-0.2);}
    if (m_controller.GetPOV() == -1) {m_arm.Set(0);}

  }

  void TestInit() override {}

  void TestPeriodic() override {}

 private:
  // Robot drive system
  frc::PWMSparkMax m_left{0};
  frc::PWMSparkMax m_right{1};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};  

  frc::XboxController m_controller{0};
  frc::Timer m_timer;

  //Switch lower true if hit false if not
  frc::DigitalInput elevator_switch_lower{0};
  //Switch upper true if hit false if not
  frc::DigitalInput elevator_switch_upper{1};

  //CANbus elevator
  rev::CANSparkMax m_elevator{21, rev::CANSparkMax::MotorType::kBrushless};

  //CANbus arm
  rev::CANSparkMax m_arm{24, rev::CANSparkMax::MotorType::kBrushed};

  //pneumatics
  frc::Compressor pcmCompressor{0, frc::PneumaticsModuleType::CTREPCM};
  bool enabled = pcmCompressor.Enabled();
  bool pressureSwitch = pcmCompressor.GetPressureSwitchValue();
  double current = (double) pcmCompressor.GetCurrent();

  // Double Solenoid, 0, 1 - ports
  frc::DoubleSolenoid DoublePCM{frc::PneumaticsModuleType::CTREPCM, 0, 1};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif