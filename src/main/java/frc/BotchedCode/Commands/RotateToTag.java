// package frc.BotchedCode.Commands;

// import frc.BotchedCode.RobotContainer;
// import frc.BotchedCode.Constants.RobotMap;
// import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;

// import com.team364.swervelib.util.SwerveConstants;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import frc.BotchedCode.Constants.RobotMap;
// import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;

// public class RotateToTag extends Command {
//     private CommandSwerveDrivetrain drivetrainSubsystem;
//     // l
//     private PIDController angleController;
//     private double maxRotate;

//     /**
//      * Align robot with the target using the limelight
//      * 
//      * @param drivetrainSubsystem
//      * @param limelight
//      */
//     public RotateToTag(CommandSwerveDrivetrain drivetrainSubsystem) {
//         this.drivetrainSubsystem = drivetrainSubsystem;
//         this.maxRotate = RobotMap.ALIGN_ANGLE_LIMIT * RobotContainer.MaxAngularRate;

//         angleController = new PIDController(RobotMap.ALIGN_ANGLE_KP, RobotMap.ALIGN_ANGLE_KI, RobotMap.ALIGN_ANGLE_KD);
//         // TODO retune PID
//         angleController.setTolerance(0.025);
//         angleController.enableContinuousInput(-Math.PI, Math.PI);
//         angleController.setSetpoint(0.0);

//         addRequirements(drivetrainSubsystem);
//     }

//     @Override
//     public void execute() {
//         double error = MathUtil.angleModulus(RobotContainer.gyro.getGyroscopeRotation().getRadians())
//                 - drivetrainSubsystem.setPoint;
//         double rotation = (MathUtil.clamp(angleController.calculate(error, 0.0),
//                 -maxRotate, maxRotate));
//         drivetrainSubsystem.drive(new Translation2d(0.0, 0.0), rotation, true, true);
//     }

//     @Override
//     public boolean isFinished() {
//         return angleController.atSetpoint();
//     }
// }