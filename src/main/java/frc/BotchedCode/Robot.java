// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.BotchedCode.Utils.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final boolean useLimelight = false;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_robotContainer.drivetrain.getDaqThread().setThreadPriority(99);
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Mod0 Offset", Units.rotationsToDegrees(m_robotContainer.drivetrain.getModule(0).getCANcoder().getAbsolutePosition().getValue()));
    SmartDashboard.putNumber("Mod1 Offset", Units.rotationsToDegrees(m_robotContainer.drivetrain.getModule(1).getCANcoder().getAbsolutePosition().getValue()));
    SmartDashboard.putNumber("Mod2 Offset", Units.rotationsToDegrees(m_robotContainer.drivetrain.getModule(2).getCANcoder().getAbsolutePosition().getValue()));
    SmartDashboard.putNumber("Mod3 Offset", Units.rotationsToDegrees(m_robotContainer.drivetrain.getModule(3).getCANcoder().getAbsolutePosition().getValue()));

    m_robotContainer.drivetrain.updateOdometry();
    if (useLimelight) {
      m_robotContainer.drivetrain.updateVision();
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
