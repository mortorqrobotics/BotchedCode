package frc.BotchedCode.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.RobotContainer;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;
import frc.BotchedCode.Utils.LimelightHelpers;

public class StrafeToTag extends Command {
    private CommandSwerveDrivetrain drivetrainSubsystem;
    // l
    private PIDController angleController;
    private PIDController xController;
    private PIDController yController;
    private double angleSetpoint; 
    private double xSetpoint; 
    private double ySetpoint; 
    private double angleOffset; 
    private double xOffset; 
    private double yOffset; 

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(RobotContainer.MaxSpeed * 0.1).withRotationalDeadband(RobotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    

    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public StrafeToTag(CommandSwerveDrivetrain drivetrainSubsystem, double angleOffset, double xOffset, double yOffset) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.angleOffset = angleOffset;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        angleController = new PIDController(10, 0, 0);
        // TODO tune PID and tolerance
        angleController.setTolerance(0.025);
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setSetpoint(angleSetpoint);

        xController = new PIDController(1, 0, 0);
        // TODO tune PID and tolerance
        xController.setTolerance(0.05);
        xController.setSetpoint(xSetpoint);

        yController = new PIDController(1, 0, 0);
        // TODO tune PID and tolerance
        yController.setTolerance(0.05);
        yController.setSetpoint(ySetpoint);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        angleSetpoint = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get().getRotation().getAngle() + angleOffset;
        xSetpoint = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get().getX() + xOffset;
        ySetpoint = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get().getRotation().getAngle() + yOffset;
        
    }

    @Override
    public void execute() {
        // System.out.print("angle: " + drivetrainSubsystem.getState().Pose.getRotation().getRadians());
        double rotation = (angleController.calculate(drivetrainSubsystem.getState().Pose.getRotation().getRadians(), angleSetpoint));
        // System.out.println( "   rot: " + rotation);
        double xDist = (xController.calculate(drivetrainSubsystem.getState().Pose.getX(), xSetpoint));
        double yDist = (yController.calculate(drivetrainSubsystem.getState().Pose.getY(), ySetpoint));
                
        drivetrainSubsystem.setControl(
            drive.withVelocityX(xDist) // Drive forward with negative Y (forward)
            .withVelocityY(yDist) // Drive left with negative X (left)
            .withRotationalRate(rotation)
        ); // Drive counterclockwise with negative X (left)
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint() && xController.atSetpoint() && yController.atSetpoint();
    }
}