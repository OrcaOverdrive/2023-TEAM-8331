// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/PowerDistribution.h>

class Robot : public frc::TimedRobot
{
public:
  Robot()
  {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.SetInverted(true);
    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
  }

  void AutonomousInit() override
  {
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override
  {
    // Drive for 2 seconds
    if (m_timer.Get() < 2_s)
    {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.ArcadeDrive(0.5, 0.0, false);
    }
    else
    {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0, false);
    }
  }

  void TeleopInit() override {}

  void TeleopPeriodic() override
  {
    // Drive with arcade style (use right stick to steer)
    m_robotDrive.ArcadeDrive(-m_controller.GetLeftY(),
                             m_controller.GetRightX());
  }

  void TestInit() override
  {
  }

  void TestPeriodic() override
  {
    // Read Power Distribution Panel Power
    double pdp_voltage = m_power_distribution_panel.GetVoltage();
    // Read PDP Temperature
    double pdp_temperature = m_power_distribution_panel.GetTemperature();
    // Read PDP Total Current draw from battery
    double pdp_current = m_power_distribution_panel.GetTotalCurrent();
    // Read PDP Power
    double pdp_power = m_power_distribution_panel.GetTotalPower();
    // Read PDP Energy
    double pdp_energy = m_power_distribution_panel.GetTotalEnergy();
    // Switch PDP relay channel on / off
    m_power_distribution_panel.SetSwitchableChannel(true);
    m_power_distribution_panel.SetSwitchableChannel(false);

  }

private:
  // Robot drive system
  frc::PWMSparkMax m_left{0};
  frc::PWMSparkMax m_right{1};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};

  // Power Distribution
  frc::PowerDistribution m_power_distribution_panel{1, frc::PowerDistribution::ModuleType::kRev};

  // Controller
  frc::XboxController m_controller{0};
  frc::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
