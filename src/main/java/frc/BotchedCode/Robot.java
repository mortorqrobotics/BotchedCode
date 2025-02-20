// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.Utils.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;

  private static NetworkTable network = NetworkTableInstance.getDefault().getTable("limelight");

  private CANdle candle = new CANdle(0, "1515Canivore");

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
    SmartDashboard.putNumber("Yaw", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());

    //SmartDashboard.putNumber("Angle", Units.radiansToDegrees(RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get().getRotation().getAngle()));

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
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees(); // this is esentually directly from the external IMU since we barely trust vision angle
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      //assuming start facing red wall (MUST KNOW STARTING ANGLE) TODO
      LimelightHelpers.SetRobotOrientation(RobotMap.LIMELIGHT_NAME, headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotMap.LIMELIGHT_NAME);
      if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
      
      //strafe to tag
      // RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(RobotMap.LIMELIGHT_NAME);
      // for (RawFiducial fiducial : fiducials) {
      //   SmartDashboard.putNumber(fiducial.id + " hypotenuse", fiducial.distToRobot);
      //   SmartDashboard.putNumber(fiducial.id + " ground distance", Math.sqrt((Math.pow(fiducial.distToRobot, 2)-Math.pow((RobotMap.TAG_HEIGHTS[fiducial.id-1] - .09), 2))));
      // }
      

      /* 
      if (network.getEntry("tv").getDouble(0) >= 1) {

        Pose2d result = LimelightHelpers.toPose2D(network.getEntry("botpose_wpiblue").getDoubleArray(new double[6]));
        // adjusted so limelight is opposite to robot forward
        Pose2d adjusted = new Pose2d(result.getX(), result.getY(), result.getRotation().plus(Rotation2d.fromDegrees(180)));

        SmartDashboard.putNumber("llX", adjusted.getX() * 100);
        SmartDashboard.putNumber("llY", adjusted.getY() * 100);

        m_robotContainer.drivetrain.addVisionMeasurement(adjusted, Utils.fpgaToCurrentTime(Timer.getFPGATimestamp()));

      }
      */
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
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = true;
    config.stripType = LEDStripType.GRB;
    config.v5Enabled = true;
    config.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
    config.brightnessScalar = 1;
    candle.configAllSettings(config, 100);
    candle.configLEDType(LEDStripType.GRB); //just added this after cd post
    candle.setLEDs(250, 0, 0, 100, 0, 10);
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