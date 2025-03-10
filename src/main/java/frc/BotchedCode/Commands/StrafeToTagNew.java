package frc.BotchedCode.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.RobotContainer;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.BotchedCode.Utils.LimelightHelpers;

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
    private int timer;
    public static boolean running;
    private double speed;
    //private boolean centered;
    

    /**
     * Align eobot with the target using the limelight
     * 
     * @param drivetrainSubsystem
     * @param limelight
     */
    public StrafeToTagNew(CommandSwerveDrivetrain drivetrainSubsystem, boolean centered) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        running = false;
        xOffset = 0.73;
        yOffset = centered ? 0.1 : -0.2;

        SmartDashboard.putNumber("kp", 3);
        SmartDashboard.putNumber("ki", 0.75);
        SmartDashboard.putNumber("kd", 0);
        SmartDashboard.putNumber("tolerance", 0.02);
        SmartDashboard.putNumber("speed", 0.0);

        double kp = 0, ki = 0, kd = 0, tolerance = 0;

        xController = new PIDController(kp, ki, kd);
        // TODO tune PID and tolerance
        xController.setTolerance(tolerance);

        yController = new PIDController(kp, ki, kd);
        // TODO tune PID and tolerance
        yController.setTolerance(tolerance);

        angleController = new PIDController(10, 0, 0);
        // TODO tune PID and tolerance
        angleController.setTolerance(0.015);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrainSubsystem);
        
    }

    @Override
    public void initialize(){
        xController.reset();
        yController.reset();
        angleController.reset();
        speed = 0;

        timer = 0;

        xController.setP(SmartDashboard.getNumber("kp", 0));
        xController.setI(SmartDashboard.getNumber("ki", 0));
        xController.setD(SmartDashboard.getNumber("kd", 0));
        xController.setTolerance(SmartDashboard.getNumber("tolerance", 0));

        yController.setP(SmartDashboard.getNumber("kp", 0));
        yController.setI(SmartDashboard.getNumber("ki", 0));
        yController.setD(SmartDashboard.getNumber("kd", 0));
        yController.setTolerance(SmartDashboard.getNumber("tolerance", 0));

        if (LimelightHelpers.getTV(RobotMap.LIMELIGHT_NAME)){
            var tagPose = RobotMap.ANDYMARK_FIELD2025.getTagPose((int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME)).get();
            double tagRotation = tagPose.getRotation().getAngle();
            xSetpoint = tagPose.getX()+xOffset*Math.cos(tagRotation)+yOffset*Math.sin(tagRotation);
            ySetpoint = tagPose.getY()+xOffset*Math.sin(tagRotation)+yOffset*Math.cos(tagRotation);

            angleSetpoint = tagRotation+Math.PI;

            speed = SmartDashboard.getNumber("speed", 0.0);

            xController.setSetpoint(xSetpoint);
            yController.setSetpoint(ySetpoint);
            angleController.setSetpoint(angleSetpoint);
        }
        else{
            var currentPose = drivetrainSubsystem.getState().Pose;

            xController.setSetpoint(currentPose.getX());
            yController.setSetpoint(currentPose.getY());
            angleController.setSetpoint(currentPose.getRotation().getRadians());
        }
    }

    @Override
    public void execute() {

        //System.out.println("XSet: " + xSetpoint + "   YSet: " + ySetpoint);
        if (xController.atSetpoint() && yController.atSetpoint()){
            timer++;
        }
        else{
            timer=0;
        }
        var currentPose = drivetrainSubsystem.getState().Pose;

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double rotationSpeed = angleController.calculate(currentPose.getRotation().getRadians());
        SmartDashboard.putNumber("X Error", xController.getError());
        SmartDashboard.putNumber("Y Error", yController.getError());

        // System.out.println("XPose: " + robotPos.getX() + "   YPose: " + robotPos.getY());
        // System.out.println("FXPose: " + xSetpoint + "   FYPose: " + ySetpoint);
        // System.out.println("xSpeed: " + xSpeed + "   ySpeed: " + ySpeed);

        double xFF = Math.abs(speed)*Math.signum(xSpeed), yFF = Math.abs(speed)*Math.signum(ySpeed);
                
        drivetrainSubsystem.setControl(
            RobotContainer.drive.withVelocityX(-xSpeed-xFF)
            .withVelocityY(-ySpeed-yFF)
            .withRotationalRate(rotationSpeed)
        );

        //timer++;
        
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint()) && timer>10;
    }

    @Override
    public void end(boolean interupted){
        RobotContainer.drive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0.001);
    }
}