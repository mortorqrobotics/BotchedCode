package frc.BotchedCode.Constants;

public class RobotMap {

    public static final String SUBSYSTEM_BUS = "";


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


    //Gyro
    public static final int PIGEON_ID = 30;


    //Elevator
    public static final double L2_HEIGHT = 0; //TODO
    public static final double L3_HEIGHT = 0; //TODO
    public static final double L4_HEIGHT = 0; //TODO
    public static final double CORAL_STATION_HEIGHT = 0; //TODO

    public static final int ELEVATOR_ID = 21; //TODO
    public static final int ELEVATOR2_ID = 22; //TODO
    public static final double MANUAL_ELEVATOR_INCREMENTATION = 0; //TODO
    public static final double ELEVATOR_MAX_SPEED = 0.05; //TODO
    public static final double ELEVATOR_MAX_ACCELERATION = 0; //TODO
    public static final double ELEVATOR_KP = 0; //TODO
    public static final double ELEVATOR_KI = 0; //TODO
    public static final double ELEVATOR_KD = 0; //TODO
    public static final double ELEVATOR_UPPER_LIMIT = 0; //TODO
    public static final int ELEVATOR_LIMIT_SWITCH_CHANNEL = -1; //TODO
    public static final double MANUAL_ELEVATOR_SPEED = .2; //TODO

    //Pivot
    public static final double L23_ANGLE = 0.56; //TODO
    public static final double L4_ANGLE = 0.42; //TODO
    public static final double CORAL_STATION_ANGLE = 0.77; //TODO

    public static final int PIVOT_ID = 15; //TODO
    public static final int PIVOT_CANCODER_ID = 16; //TODO
    public static final double MANUAL_PIVOT_INCREMENTATION = .005; //TODO  .005
    public static final double PIVOT_MAX_SPEED = 0.2; //TODO  .2
    public static final double PIVOT_MAX_ACCELERATION = 1; //TODO 1
    public static final double PIVOT_KP = 0.1; //TODO .2
    public static final double PIVOT_KI = 0; //TODO
    public static final double PIVOT_KD = 0; //TODO
    public static final double PIVOT_LOWER_LIMIT = 0.38; //TODO
    public static final double PIVOT_UPPER_LIMIT = 0.80; //TODO


    public static final int LEFT_INTAKEALGAE_ID = -1; //TODO
    public static final int RIGHT_INTAKEALGAE_ID = -1; //TODO
    public static final double  LEFT_INTAKEALGAE_SPEED = .25; //TODO
    public static final double  RIGHT_INTAKEALGAE_SPEED = -.25; //TODO


    public static final int LEFT_INTAKECORAL_ID = -1; //TODO
    public static final int RIGHT_INTAKECORAL_ID = -1; //TODO
    public static final double INTAKECORAL_SPEED = 0.2; //TODO

    public static final double BARB_SPEED = 0;//TODO 
    
    public static final int BARB_ID = -1;//TODO 



    public static final double ALGAE_INTAKE_CURRENT_PICKUP = 10; //TODO
    public static final double CORAL_INTAKE_CURRENT_PICKUP = 30; //TODO

}
