// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

    @Override
  public void robotInit(){
    PathfindingCommand.warmupCommand().schedule();
    // tagPose = RobotMap.WELDED_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME)).get();
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(640/2, 360/2);
    camera.setFPS(30);
    camera.setPixelFormat(PixelFormat.kMJPEG);
    LimelightHelpers.SetIMUMode(RobotMap.LIMELIGHT_NAME, 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    //Module Offsets
    // SmartDashboard.putNumber("Mod0 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble()));
    // SmartDashboard.putNumber("Mod1 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble()));
    // SmartDashboard.putNumber("Mod2 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble()));
    // SmartDashboard.putNumber("Mod3 Offset", Units.rotationsToDegrees(RobotContainer.drivetrain.getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble()));
    
    SmartDashboard.putNumber("PoseX", RobotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("PoseY", RobotContainer.drivetrain.getState().Pose.getY());
    SmartDashboard.putNumber("Yaw", RobotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("TreuYaw", RobotContainer.gyro.getYaw().getValueAsDouble());

    SmartDashboard.putNumber("Viewed Tag", LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME));

    if (kUseLimelight) {

      var driveState = RobotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      
      // try {  
      //   headingDeg += DriverStation.getAlliance().get() == Alliance.Blue ? 180: 0; // this is esentually directly from the external IMU since we barely trust vision angle

      // } 
      // catch (Exception e) {
      //   System.out.print(e);
      // }

      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      //assuming limelight starts facing red wall (MUST KNOW STARTING ANGLE) Todo
      LimelightHelpers.SetRobotOrientation(RobotMap.LIMELIGHT_NAME, headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RobotMap.LIMELIGHT_NAME);
      if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
        RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
    }
  }

  @Override
  public void disabledInit() {
    LimelightHelpers.SetIMUMode(RobotMap.LIMELIGHT_NAME, 0);
    LimelightHelpers.SetThrottle(RobotMap.LIMELIGHT_NAME, 200 );
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    //LimelightHelpers.SetIMUMode(RobotMap.LIMELIGHT_NAME, 2);
    LimelightHelpers.SetThrottle(RobotMap.LIMELIGHT_NAME, 0);
  }

  @Override
  public void autonomousInit() {
    try {  
      RobotContainer.gyro.setYaw(DriverStation.getAlliance().get() == Alliance.Blue ? Math.PI: 0); // this is esentually directly from the external IMU since we barely trust vision angle
      RobotContainer.drivetrain.resetRotation(new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? Math.PI: 0));
    } 
    catch (Exception e) {
      System.out.print(e);
    }
    //RobotContainer.drivetrain.runOnce(() -> RobotContainer.drivetrain.seedFieldCentric()).schedule();
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