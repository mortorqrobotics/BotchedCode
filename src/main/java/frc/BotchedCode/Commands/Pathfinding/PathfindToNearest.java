// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode.Commands.Pathfinding;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.AprilTagPositions;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathfindToNearest extends Command {
  private Command fullPath;
  private CommandSwerveDrivetrain drive;
  private boolean isLeftBumper = false;
  private double maxLinAccel = 2.0;
  private double maxLinVel = 2.0;
  private double maxAngAccel = 360.0;
  private double maxAngVel = 180.0;

  /** Creates a new PathfindToNearest. */
  public PathfindToNearest(CommandSwerveDrivetrain drive, boolean isLeftBumper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.isLeftBumper = isLeftBumper;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d closestAprilTagPose = getClosestReefAprilTagPose();
    Command pathfindPath = AutoBuilder.pathfindToPose(
      translateCoord(closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5),
        new PathConstraints(maxLinVel, maxLinAccel, Units.degreesToRadians(maxAngVel), Units.degreesToRadians(maxAngAccel)));

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath pathToFront = new PathPlannerPath(
          PathPlannerPath.waypointsFromPoses(
            translateCoord(closestAprilTagPose, closestAprilTagPose.getRotation().getDegrees(), -0.5),
              closestAprilTagPose),
          new PathConstraints(maxLinVel, maxLinAccel, Units.degreesToRadians(maxAngVel), Units.degreesToRadians(maxAngAccel)),
          null, 
          new GoalEndState(0.0, closestAprilTagPose.getRotation())
      );
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
    return false;
  }

  private Pose2d getClosestReefAprilTagPose() {
    HashMap<Integer, Pose2d> aprilTagsToAlignTo = AprilTagPositions.WELDED_BLUE_CORAL_APRIL_TAG_POSITIONS;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        aprilTagsToAlignTo = AprilTagPositions.WELDED_RED_CORAL_APRIL_TAG_POSITIONS;
      }
    }

    Pose2d currentPose = drive.getState().Pose;
    Pose2d closestPose = new Pose2d();
    double closestDistance = Double.MAX_VALUE;
    Integer aprilTagNum = -1;

    for (Map.Entry<Integer, Pose2d> entry : aprilTagsToAlignTo.entrySet()) {
      Pose2d pose = entry.getValue();
      double distance = findDistanceBetween(currentPose, pose);
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = pose;
        aprilTagNum = entry.getKey();
      }
    }

    double xOffset = 0.67;
    double yOffset = isLeftBumper ? 0.2 : -0.1;

    Pose2d inFrontOfAprilTag = translateCoord(closestPose, closestPose.getRotation().getDegrees(),
        -xOffset);
    Pose2d leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 180, yOffset);
    if (List.of(11, 10, 9, 22, 21, 20).contains(aprilTagNum)) {
      leftOrRightOfAprilTag = translateCoord(inFrontOfAprilTag, closestPose.getRotation().getDegrees() + 180, -yOffset);
    }

    return leftOrRightOfAprilTag;
  }

  private Pose2d translateCoord(Pose2d originalPose, double degreesRotate, double distance) {
    double newXCoord = originalPose.getX() + (Math.cos(Math.toRadians(degreesRotate)) * distance);
    double newYCoord = originalPose.getY() + (Math.sin(Math.toRadians(degreesRotate)) * distance);

    return new Pose2d(newXCoord, newYCoord, originalPose.getRotation());
  }

  private double findDistanceBetween(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(Math.pow((pose2.getX() - pose1.getX()), 2) + Math.pow((pose2.getY() - pose1.getY()), 2));
  }
}