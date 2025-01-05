package frc.BotchedCode.Constants;

public class RobotMap {
    //Limelight 
    // Increase these numbers to trust your model's state estimates less.
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;

    // Increase these numbers to trust global measurements from vision less.
    public static final double kVisionStdDevX = 5;
    public static final double kVisionStdDevY = 5;
    public static final double kVisionStdDevTheta = 500;

    public static final String LIMELIGHT_NAME = "limelight";
    public static final double DIFFERENCE_CUTOFF_THRESHOLD = 1.5; // Max difference between vision and odometry pose

    //Gyro
    public static final int PIGEON_ID = 30;
}
