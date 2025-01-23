package frc.BotchedCode.Constants;

public class RobotMap {
    //Limelight 
    // Increase these numbers to trust your model's state estimates less.
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;

    // Increase these numbers to trust global measurements from vision less.
    public static final double kVisionStdDevX = 15;
    public static final double kVisionStdDevY = 15;
    public static final double kVisionStdDevTheta = 99999;

    public static final String LIMELIGHT_NAME = "limelight";
    public static final double DIFFERENCE_CUTOFF_THRESHOLD = 1.5; // Max difference between vision and odometry pose

    public static final int ELEVATOR_ID = -1; //TODO

    public static final String SUBSYSTEM_BUS = "";

    //Gyro
    public static final int PIGEON_ID = 30;

    public static final double ELEVATOR_SPEED = 0; //TODO
    public static final double L2_HEIGHT = 0; //TODO
    public static final double L3_HEIGHT = 0; //TODO
    public static final double L4_HEIGHT = 0; //TODO
    public static final double CORAL_STATION_HEIGHT = 0; //TODO
    public static final double MAX_ELEVATOR_EXTENSION = 0; //TODO
    public static final double MANUAL_ELEVATOR_INCREMENTATION = 1; //TODO
    public static final double ELEVATOR_MAX_SPEED = 0; //TODO
    public static final double ELEVATOR_MAX_ACCELERATION = 0; //TODO
    public static final double ELEVATOR_KP = 0; //TODO
    public static final double ELEVATOR_KI = 0; //TODO
    public static final double ELEVATOR_KD = 0; //TODO
    public static final int ELEVATOR_LIMIT_SWITCH_CHANNEL = -1; //TODO
}
