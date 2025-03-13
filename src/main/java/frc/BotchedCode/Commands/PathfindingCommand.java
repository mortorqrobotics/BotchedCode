package frc.BotchedCode.Commands;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Robot;
import frc.BotchedCode.RobotContainer;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;
import frc.BotchedCode.Utils.LimelightHelpers;

public class PathfindingCommand extends Command {

    private BooleanSupplier offCenter;    

    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public PathfindingCommand(BooleanSupplier offCenter) {
        this.offCenter = offCenter;
    }

    @Override
    public void initialize(){
        //double farX = 1;
        double xOffset = 0.67;
        double yOffset = offCenter.getAsBoolean() ? -0.2 : 0.1;

        if (LimelightHelpers.getTV(RobotMap.LIMELIGHT_NAME)){
            var tagPose = RobotMap.WELDED_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME)).get();
            double tagRotation = tagPose.getRotation().getAngle();
            double angleOffset = Math.atan(yOffset/xOffset);
            double offset = Math.sqrt(Math.pow(xOffset,2) + Math.pow(yOffset,2));

            //double startX = tagPose.getX() + farX*Math.cos(tagRotation);
            //double startY = tagPose.getY() + farX*Math.sin(tagRotation);
            double endX = tagPose.getX() + offset*Math.cos(tagRotation+angleOffset);
            double endY = tagPose.getY() + offset*Math.sin(tagRotation+angleOffset);

            PathConstraints contraints = new PathConstraints(2, 1, Math.PI, Math.PI*2);
            // List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            //     new Pose2d(startX, startY, Rotation2d.fromRadians(angleOffset+tagRotation)),
            //     new Pose2d(endX, endY, Rotation2d.fromRadians(angleOffset+tagRotation))
            // );

            AutoBuilder.pathfindToPose(
                new Pose2d(endX, endY, Rotation2d.fromRadians(tagRotation+Math.PI)),
                contraints
            ).until(RobotContainer.controller1.start()).schedule();
        }
    }
}