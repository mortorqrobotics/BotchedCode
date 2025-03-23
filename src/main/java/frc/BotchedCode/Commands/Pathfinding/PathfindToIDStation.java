// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode.Commands.Pathfinding;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.AprilTagPositions;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindToIDStation extends Command {
  private Command fullPath;
  private CommandSwerveDrivetrain drive;
  private boolean isLeftBumper = false;
  private double maxLinAccel = 2.0;
  private double maxLinVel = 2.0;
  private double maxAngAccel = 360.0;
  private double maxAngVel = 180.0;
  private int id;
  private int timer;

  /** Creates a new PathfindToNearest. */
  public PathfindToIDStation(CommandSwerveDrivetrain drive, boolean isLeftBumper, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.isLeftBumper = isLeftBumper;
    this.id = id;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    Pose2d closestAprilTagPose = getClosestReefAprilTagPose();
    Command pathfindPath = AutoBuilder.pathfindToPose(
        translateCoord(closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5),
        new PathConstraints(maxLinVel, maxLinAccel, Units.degreesToRadians(maxAngVel),
            Units.degreesToRadians(maxAngAccel)));

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath pathToFront = new PathPlannerPath(
          PathPlannerPath.waypointsFromPoses(
              translateCoord(closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5),
              closestAprilTagPose),
          new PathConstraints(maxLinVel, maxLinAccel, Units.degreesToRadians(maxAngVel),
              Units.degreesToRadians(maxAngAccel)),
          null,
          new GoalEndState(0.0, closestAprilTagPose.getRotation()));
      pathToFront.preventFlipping = true;
      fullPath = pathfindPath.andThen(AutoBuilder.followPath(pathToFront));
      fullPath.schedule();
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer ++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (fullPath != null) {
      fullPath.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Path Finished", fullPath.isFinished());
    return fullPath.isFinished() || timer > 80;
  }

  private Pose2d getClosestReefAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo = AprilTagPositions.WELDED_BLUE_STATION_APRIL_TAG_POSITIONS;
    Integer aprilTagNum = id;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_STATION_APRIL_TAG_POSITIONS;
      }
    }
    Pose2d closestPose = aprilTagsToAlignTo.get(id);

    Pose2d inFrontOfAprilTag = translateCoord(closestPose, closestPose.getRotation().getDegrees(),
        -0.45);

    Pose2d leftOrRightOfAprilTag;
    if (isLeftBumper) {
      leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.0);
    } else {
      leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.0);
    }

    if (List.of(11, 10, 9, 22, 21, 20).contains(aprilTagNum)) {
      if (isLeftBumper) {
        leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.0);
      } else {
        leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 90, 0.0);
      }
    }

    return leftOrRightOfAprilTag;
  }

  private Pose2d translateCoord(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }
}