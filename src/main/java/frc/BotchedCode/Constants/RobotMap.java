package frc.BotchedCode.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class RobotMap {

    public static final String SUBSYSTEM_BUS = "";


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
    public static final double[] TAG_HEIGHTS = {
    1.4859, 1.4859, /*ID: 1 - 2*/
    1.30175, /*ID: 3*/
    1.8679160000000001, 1.8679160000000001, /*ID: 4 - 5*/
    0.308102, 0.308102, 0.308102, 0.308102, 0.308102, 0.308102, /*ID: 6 - 11*/
    1.4859, 1.4859, /*ID: 12 - 13*/
    1.8679160000000001, 1.8679160000000001, /*ID: 14 - 15*/
    1.30175, /*ID: 16*/
    0.308102, 0.308102, 0.308102, 0.308102, 0.308102, 0.308102 /*ID: 17 - 22*/};

    public static final AprilTagFieldLayout ANDYMARK_FIELD2025 = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final AprilTagFieldLayout WELDED_FIELD2025 = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);


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
    public static final double ELEVATOR_UPPER_LIMIT = 0; //TODO
    public static final int ELEVATOR_LIMIT_SWITCH_CHANNEL = -1; //TODO
    public static final double ELEVATOR_SPEED = .2; //TODO

    //Pivot
    public static final double L23_ANGLE = 0.56; //TODO
    public static final double L4_ANGLE = 0.42; //TODO
    public static final double CORAL_STATION_ANGLE = 0.77; //TODO

    public static final int PIVOT_ID = 15; //TODO
    public static final int PIVOT_CANCODER_ID = 16; //TODO
    public static final double PIVOT_SPEED = 0.1; //TODO  .2
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
    public static final double PIVOT_CURRENT_LIMIT = 3; //TODO

}
