// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.BotchedCode.Utils.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;

  private static NetworkTable network = NetworkTableInstance.getDefault().getTable("limelight");

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //Module Offsets
    SmartDashboard.putNumber("Mod0 Offset", Units.rotationsToDegrees(m_robotContainer.drivetrain.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("Mod1 Offset", Units.rotationsToDegrees(m_robotContainer.drivetrain.getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("Mod2 Offset", Units.rotationsToDegrees(m_robotContainer.drivetrain.getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble()));
    SmartDashboard.putNumber("Mod3 Offset", Units.rotationsToDegrees(m_robotContainer.drivetrain.getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble()));
    
    SmartDashboard.putNumber("PoseX", m_robotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("PoseY", m_robotContainer.drivetrain.getState().Pose.getY());

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     * s
     */


    if (kUseLimelight) {
      //Pose2d result = LimelightHelpers.toPose2D(network.getEntry("botpose_wpiblue").getDoubleArray(new double[0]));
      if (network.getEntry("tv").getDouble(0) >= 1) {

        Pose2d result = LimelightHelpers.toPose2D(network.getEntry("botpose_wpiblue").getDoubleArray(new double[6]));
        // adjusted so limelight is opposite to robot forward
        Pose2d adjusted = new Pose2d(result.getX(), result.getY(), result.getRotation().plus(Rotation2d.fromDegrees(180)));

        SmartDashboard.putNumber("llX", adjusted.getX() * 100);
        SmartDashboard.putNumber("llY", adjusted.getY() * 100);

        m_robotContainer.drivetrain.addVisionMeasurement(adjusted, Utils.fpgaToCurrentTime(Timer.getFPGATimestamp()));

      }
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
  public void teleopExit() {}

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