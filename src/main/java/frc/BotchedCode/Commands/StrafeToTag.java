package frc.BotchedCode.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.RobotContainer;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;
import frc.BotchedCode.Utils.LimelightHelpers;

public class StrafeToTag extends Command {
    private CommandSwerveDrivetrain drivetrainSubsystem;
    private PIDController xController;
    private PIDController yController;
    private double xSetpoint; 
    private double ySetpoint;
    private double xOffset;
    private double yOffset;
    //private boolean centered;
    private double xDist;
    private double yDist;
    

    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public StrafeToTag(CommandSwerveDrivetrain drivetrainSubsystem, boolean centered) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        xOffset = 0.5;
        yOffset = centered ? 0 : 0.2;
        double kp = 1, ki = 3, kd = 0, tolerance = 0.02;

        xController = new PIDController(kp, ki, kd);
        // TODO tune PID and tolerance
        xController.setTolerance(tolerance);
        xController.setSetpoint(xSetpoint);

        yController = new PIDController(kp, ki, kd);
        // TODO tune PID and tolerance
        yController.setTolerance(tolerance);
        yController.setSetpoint(ySetpoint);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        xController.reset();
        yController.reset();

        var tagPose = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get();
        double tagHeight = tagPose.getZ();
        double ty = LimelightHelpers.getTY(RobotMap.LIMELIGHT_NAME) + LimelightHelpers.getCameraPose3d_RobotSpace(RobotMap.LIMELIGHT_NAME).getRotation().getY();
        double tx = LimelightHelpers.getTX(RobotMap.LIMELIGHT_NAME);
        xDist = tagHeight/Math.tan(Units.degreesToRadians(ty));
        yDist = xDist*Math.tan(Units.degreesToRadians(tx));
        System.out.println("XDist: " + xDist + "   YDist: " + yDist);

        //double angle = RobotContainer.drivetrain.getState().Pose.getRotation().getRadians();
        //xSetpoint = tagPose.getX() - offset*Math.cos(angle);
        //ySetpoint = tagPose.getY() - offset*Math.sin(angle);
        xSetpoint = xOffset;
        ySetpoint = yOffset;
    }

    @Override
    public void execute() {
        double xSpeed = xController.calculate(xDist, xOffset);
        double ySpeed = yController.calculate(yDist, yOffset);

        //var robotPos = RobotContainer.drivetrain.getState().Pose;

        // double xSpeed = xController.calculate(robotPos.getX(), xSetpoint);
        // double ySpeed = yController.calculate(robotPos.getY(), ySetpoint);

        // System.out.println("XPose: " + robotPos.getX() + "   YPose: " + robotPos.getY());
        // System.out.println("FXPose: " + xSetpoint + "   FYPose: " + ySetpoint);
        // System.out.println("xSpeed: " + xSpeed + "   ySpeed: " + ySpeed);
                
        drivetrainSubsystem.setControl(
            RobotContainer.forwardStraight.withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint();
    }
}