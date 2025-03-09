package frc.BotchedCode.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class RobotMap {


    //Limelight 
    // Increase these numbers to trust your model's state estimates less.
    public static final double kPositionStdDevX = 0.1;
    public static final double kPositionStdDevY = 0.1;
    public static final double kPositionStdDevTheta = 10;
    // Increase these numbers to trust global measurements from vision less.
    public static final double kVisionStdDevX = 1;
    public static final double kVisionStdDevY = 1;
    public static final double kVisionStdDevTheta = 99999;

    public static final String LIMELIGHT_NAME = "limelight";
    public static final double DIFFERENCE_CUTOFF_THRESHOLD = 1.5; // Max difference between vision and odometry pose


    public static final AprilTagFieldLayout ANDYMARK_FIELD2025 = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final AprilTagFieldLayout WELDED_FIELD2025 = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    

    //Gyro
    public static final int PIGEON_ID = 30;


    //Elevator
    public static final double L2_HEIGHT = 5.5; //TODO
    public static final double L3_HEIGHT = L2_HEIGHT+10; //TODO
    public static final double L4_HEIGHT = 33.75; //TODO
    public static final double CORAL_STATION_HEIGHT = 0; //TODO
    public static final double REST_HEIGHT = 1.4; //TODO
    public static final double PROCESSOR_HEIGHT = 4; //TODO
    public static final double REST_HEIGHT_LIMIT = 1.57; //TODO

    public static final int ELEVATOR_ID = 21; //TODO
    public static final int ELEVATOR2_ID = 22; //TODO
    public static final double MANUAL_ELEVATOR_INCREMENTATION = 0; //TODO
    public static final double ELEVATOR_UPPER_LIMIT = 0; //TODO
    public static final int ELEVATOR_LIMIT_SWITCH_CHANNEL = -1; //TODO
    public static final double ELEVATOR_SPEED = 0.1; //TODO

    //Pivot
    public static final double L23_ANGLE = 13; //TODO
    public static final double L4_ANGLE =10; //TODO
    public static final double CORAL_STATION_ANGLE = 25; //TODO
    public static final double REST_ANGLE = 25; //TODO
    public static final double PROCESSOR_ANGLE = 27; //TODO
    public static final double REST_ANGLE_LIMIT = 34; //TODO

    public static final int PIVOT_ID = 23; //TODO
    public static final int PIVOT_CANCODER_ID = 24; //TODO
    public static final double PIVOT_SPEED = 0.1; //TODO  .2
    public static final double PIVOT_LOWER_LIMIT = 0.38; //TODO
    public static final double PIVOT_UPPER_LIMIT = 0.80; //TODO


    public static final int LEFT_INTAKEALGAE_ID = 25; //TODO
    public static final int RIGHT_INTAKEALGAE_ID = 26; //TODO
    public static final double INTAKEALGAE_SPEED = .3; //TODO


    public static final int LEFT_INTAKECORAL_ID = 27; //TODO
    public static final int RIGHT_INTAKECORAL_ID = 28; //TODO
    public static final double INTAKECORAL_SPEED = 0.3;//TODO


    public static final double BARB_SPEED = 0.4;//TODO 
    public static final int BARB_ID = 29;//TODO 


    public static final double ALGAE_INTAKE_CURRENT_PICKUP = 7; //TODO
    public static final double CORAL_INTAKE_CURRENT_PICKUP = 30; //TODO
    public static final double PIVOT_CURRENT_LIMIT = 3; //TODO
    
    public static final int CANDLE_ID = 0;

}
