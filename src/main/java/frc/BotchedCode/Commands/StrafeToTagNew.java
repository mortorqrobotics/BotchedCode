package frc.BotchedCode.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.Robot;
import frc.BotchedCode.RobotContainer;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;
import frc.BotchedCode.Utils.LimelightHelpers;
import frc.BotchedCode.Utils.LimelightHelpers.RawDetection;

public class StrafeToTagNew extends Command {
    private CommandSwerveDrivetrain drivetrainSubsystem;
    private PIDController xController;
    private PIDController yController;
    private PIDController angleController;
    private double xSetpoint; 
    private double ySetpoint;
    private double angleSetpoint;
    private double xOffset;
    private double yOffset;
    //private boolean centered;
    

    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public StrafeToTagNew(CommandSwerveDrivetrain drivetrainSubsystem, boolean centered) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        xOffset = 0.5;
        yOffset = centered ? 0 : 0.2;
        double kp = 1, ki = 0, kd = 0, tolerance = 0.02;

        xController = new PIDController(kp, ki, kd);
        // TODO tune PID and tolerance
        xController.setTolerance(tolerance);

        yController = new PIDController(kp, ki, kd);
        // TODO tune PID and tolerance
        yController.setTolerance(tolerance);

        angleController = new PIDController(10, 0, 0);
        // TODO tune PID and tolerance
        angleController.setTolerance(0.025);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        xController.reset();
        yController.reset();
        angleController.reset();

        var tagPose = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME)).get();
        double tagRotation = tagPose.getRotation().getAngle();
        xSetpoint = tagPose.getX()+xOffset*Math.cos(tagRotation)+yOffset*Math.sin(tagRotation);
        ySetpoint = tagPose.getY()+xOffset*Math.sin(tagRotation)+yOffset*Math.cos(tagRotation);
        angleSetpoint = tagRotation+Math.PI;

        xController.setSetpoint(xSetpoint);
        yController.setSetpoint(ySetpoint);
        angleController.setSetpoint(angleSetpoint);
    }

    @Override
    public void execute() {
        var currentPose = drivetrainSubsystem.getState().Pose;

        double xSpeed = xController.calculate(currentPose.getX(), xSetpoint);
        double ySpeed = yController.calculate(currentPose.getY(), ySetpoint);
        double rotationSpeed = angleController.calculate(currentPose.getRotation().getRadians(), angleSetpoint);

        // System.out.println("XPose: " + robotPos.getX() + "   YPose: " + robotPos.getY());
        // System.out.println("FXPose: " + xSetpoint + "   FYPose: " + ySetpoint);
        // System.out.println("xSpeed: " + xSpeed + "   ySpeed: " + ySpeed);
                
        drivetrainSubsystem.setControl(
            RobotContainer.drive.withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rotationSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint();
    }
}