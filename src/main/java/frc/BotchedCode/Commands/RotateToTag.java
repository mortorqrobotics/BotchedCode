package frc.BotchedCode.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.RobotContainer;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;

public class RotateToTag extends Command {
    private CommandSwerveDrivetrain drivetrainSubsystem;
    // l
    private PIDController angleController;
    private double maxRotate;
    private double setpoint;
    

    /**
     * Align robot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public RotateToTag(CommandSwerveDrivetrain drivetrainSubsystem, double setpoint) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.maxRotate = RobotMap.ALIGN_ANGLE_LIMIT * RobotContainer.MaxAngularRate;
        this.setpoint = setpoint;
        
        angleController = new PIDController(RobotMap.ALIGN_ANGLE_KP, RobotMap.ALIGN_ANGLE_KI, RobotMap.ALIGN_ANGLE_KD);
        // TODO retune PID
        angleController.setTolerance(0.025);
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setSetpoint(setpoint);


        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double error = MathUtil.angleModulus(drivetrainSubsystem.getState().Pose.getRotation().getDegrees()
                - setpoint);
        double rotation = (MathUtil.clamp(angleController.calculate(error, 0.0),
                -maxRotate, maxRotate));
        
                
        drivetrainSubsystem.applyRequest(() ->
            RobotContainer.drive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(rotation)
        ); // Drive counterclockwise with negative X (left)
            
    }

    @Override
    public boolean isFinished() {
        return angleController.atSetpoint();
    }
}