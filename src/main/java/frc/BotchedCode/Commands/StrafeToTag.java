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
    private double tagHeight;
    

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
        xController.setTolerance(0.01);
        xController.setSetpoint(xSetpoint);

        yController = new PIDController(0.01, 0, 0);
        // TODO tune PID and tolerance
        yController.setTolerance(0.01);
        yController.setSetpoint(ySetpoint);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize(){
        angleSetpoint = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get().getRotation().getAngle() + angleOffset;
        // xSetpoint = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get().getX() + xOffset;
        // ySetpoint = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get().getRotation().getAngle() + yOffset;
        tagHeight = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID("limelight")).get().getZ();
        // System.out.println("XTag: " + xSetpoint + "   YTag: " + ySetpoint);
        // System.out.println("XPose: " + drivetrainSubsystem.getState().Pose.getX() + "   YPose: " + drivetrainSubsystem.getState().Pose.getY() + "\n");
    }

    @Override
    public void execute() {
        // System.out.print("angle: " + drivetrainSubsystem.getState().Pose.getRotation().getRadians());
        double rotation = angleController.calculate(drivetrainSubsystem.getState().Pose.getRotation().getRadians(), angleSetpoint);
        double floorDist = tagHeight/Math.tan(Units.degreesToRadians(LimelightHelpers.getTY(RobotMap.LIMELIGHT_NAME) + 30));
        double tx = LimelightHelpers.getTX(RobotMap.LIMELIGHT_NAME);
        //System.out.println("FloorDist: " + floorDist + "   TX: " + tx);
        // System.out.println("   rot: " + rotation);
        double xSpeed = xController.calculate(floorDist, xOffset);
        double ySpeed = yController.calculate(tx, 0);
                
        drivetrainSubsystem.setControl(
            RobotContainer.drive.withVelocityX(xSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(ySpeed) // Drive left with negative X (left)
            .withRotationalRate(0)
        ); // Drive counterclockwise with negative X (left)
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint() && xController.atSetpoint() && yController.atSetpoint();
    }
}