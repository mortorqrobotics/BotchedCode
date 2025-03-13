// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode;

import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.BotchedCode.Commands.PathfindingCommand;
import frc.BotchedCode.Commands.Barb.BarbIn;
import frc.BotchedCode.Commands.Barb.BarbOut;
import frc.BotchedCode.Commands.Intakes.IntakeAlgaeIn;
import frc.BotchedCode.Commands.Intakes.IntakeAlgaeOut;
import frc.BotchedCode.Commands.Intakes.IntakeCoralIn;
import frc.BotchedCode.Commands.Intakes.IntakeCoralOut;
import frc.BotchedCode.Commands.ManualElevatorPivot.ManualElevatorDown;
import frc.BotchedCode.Commands.ManualElevatorPivot.ManualElevatorUp;
import frc.BotchedCode.Commands.ManualElevatorPivot.ManualPivotDown;
import frc.BotchedCode.Commands.ManualElevatorPivot.ManualPivotUp;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.Constants.TunerConstants;
import frc.BotchedCode.Subsystems.Barb;
import frc.BotchedCode.Subsystems.Candle;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;
import frc.BotchedCode.Subsystems.Elevator;
import frc.BotchedCode.Subsystems.IntakeAlgae;
import frc.BotchedCode.Subsystems.IntakeCoral;
import frc.BotchedCode.Subsystems.Pivot;
import frc.BotchedCode.Utils.LimelightHelpers;



public class RobotContainer {
    public static Elevator elevator;
    public static Pivot pivot;
    public static IntakeCoral intakeCoral;
    public static IntakeAlgae intakeAlgae;
    public static Barb barb;
    public static Candle candle;

    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final static SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final static CommandXboxController controller1 = new CommandXboxController(0);
    private final static CommandXboxController controller2 = new CommandXboxController(1);
    private final static CommandXboxController controller3 = new CommandXboxController(2);
    
    public final static CommandSwerveDrivetrain drivetrain = createDrivetrain();
    public static Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_ID);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    
    

    public RobotContainer() {

        elevator = new Elevator();
        pivot = new Pivot();
        intakeAlgae = new IntakeAlgae();
        intakeCoral = new IntakeCoral();
        barb = new Barb();
        candle = new Candle(()->intakeCoral.getLeds(), ()->intakeAlgae.getLeds());

        Command L2Position = Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L2_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        );
        Command L3Position = Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L3_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        );
        Command L4Position = Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L4_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L4_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        );
        Command RestPosition = Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.REST_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        );
        Command ProcessorPosition = Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L2_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        );

        Command IntakeAlgae = Commands.sequence(new IntakeAlgaeIn(intakeAlgae),new InstantCommand(()->candle.algaeOn()));
        Command OuttakeAlgae = Commands.sequence(new IntakeAlgaeOut(intakeAlgae),new InstantCommand(()->candle.algaeOff()));
        Command IntakeCoral = Commands.sequence(new IntakeCoralIn(intakeCoral),new InstantCommand(()->candle.coralOn()));
        Command OuttakeCoral = Commands.sequence(new IntakeCoralOut(intakeCoral),new InstantCommand(()->candle.coralOff()));

        NamedCommands.registerCommand("L2Routine", Commands.sequence(L2Position, OuttakeCoral, RestPosition));
        NamedCommands.registerCommand("L3Routine", Commands.sequence(L3Position, OuttakeCoral, RestPosition));
        NamedCommands.registerCommand("L4Routine", Commands.sequence(L4Position, OuttakeCoral, RestPosition));
        NamedCommands.registerCommand("L3RoutineAlgae", Commands.sequence(L3Position, Commands.parallel(OuttakeCoral, IntakeAlgae), RestPosition));

        NamedCommands.registerCommand("L2Position", L2Position);
        NamedCommands.registerCommand("L3Position", L3Position);
        NamedCommands.registerCommand("L4Position", L4Position);
        NamedCommands.registerCommand("RestPosition", RestPosition);
        NamedCommands.registerCommand("ProcessorPosition", ProcessorPosition);

        NamedCommands.registerCommand("IntakeAlgae", IntakeAlgae);
        NamedCommands.registerCommand("OuttakeAlgae", OuttakeAlgae);
        NamedCommands.registerCommand("IntakeCoral", IntakeCoral);
        NamedCommands.registerCommand("OuttakeCoral", OuttakeCoral);

        NamedCommands.registerCommand("Startup", Commands.parallel(IntakeCoral, RestPosition));

        autoChooser = AutoBuilder.buildAutoChooser("0 Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention

        HashMap<Integer, Command> strafeCommands = new HashMap<Integer, Command>();
        strafeCommands.put(17, defineEndPos(false, 17));
        strafeCommands.put(18, defineEndPos(false, 18));
        strafeCommands.put(19, defineEndPos(false, 19));
        strafeCommands.put(20, defineEndPos(false, 20));
        strafeCommands.put(21, defineEndPos(false, 21));
        strafeCommands.put(22, defineEndPos(false, 22));
        strafeCommands.put(6, defineEndPos(false, 6));
        strafeCommands.put(7, defineEndPos(false, 7));
        strafeCommands.put(8, defineEndPos(false, 8));
        strafeCommands.put(9, defineEndPos(false, 9));
        strafeCommands.put(10, defineEndPos(false, 10));
        strafeCommands.put(11, defineEndPos(false, 11));

        HashMap<Integer, Command> altStrafeCommands = new HashMap<Integer, Command>();
        altStrafeCommands.put(17, defineEndPos(true, 17));
        altStrafeCommands.put(18, defineEndPos(true, 18));
        altStrafeCommands.put(19, defineEndPos(true, 19));
        altStrafeCommands.put(20, defineEndPos(true, 20));
        altStrafeCommands.put(21, defineEndPos(true, 21));
        altStrafeCommands.put(22, defineEndPos(true, 22));
        altStrafeCommands.put(6, defineEndPos(true, 6));
        altStrafeCommands.put(7, defineEndPos(true, 7));
        altStrafeCommands.put(8, defineEndPos(true, 8));
        altStrafeCommands.put(9, defineEndPos(true, 9));
        altStrafeCommands.put(10, defineEndPos(true, 10));
        altStrafeCommands.put(11, defineEndPos(true, 11));

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller1.getLeftY() * MaxSpeed * getRobotSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller1.getLeftX() * MaxSpeed * getRobotSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-controller1.getRightX() * MaxAngularRate * getRobotYawSpeed()) // Drive counterclockwise with negative X (left)
            )
        );

        controller1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller1.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller1.getLeftY(), -controller1.getLeftX()))
        ));
        
        controller1.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        controller1.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        controller1.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );
        controller1.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controller1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        controller2.a().onTrue(Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L2_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE)))); //TODO
        controller2.b().onTrue(Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L3_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))));
        controller2.y().onTrue(Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L4_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L4_ANGLE))));
        //controller2.x().onTrue(Positions[3]);
        controller2.start().onTrue(Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.REST_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE)))); //TODO

        //controller2.x().whileTrue(new InstantCommand(()->elevator.down()));
        //controller2.y().whileTrue(new InstantCommand(()-> elevator.up()));


        //Controls for intakes without candle
        // controller2.leftBumper().onTrue(new IntakeAlgaeIn(intakeAlgae)); 
        // controller2.rightBumper().onTrue(new IntakeAlgaeOut(intakeAlgae));

        // controller2.leftTrigger().onTrue(new IntakeCoralIn(intakeCoral)); 
        // controller2.rightTrigger().onTrue(new IntakeCoralOut(intakeCoral));

        controller2.povUp().whileTrue(new ManualElevatorUp(elevator));
        controller2.povDown().whileTrue(new ManualElevatorDown(elevator));
        controller2.povRight().whileTrue(new ManualPivotUp(pivot));
        controller2.povLeft().whileTrue(new ManualPivotDown(pivot));

        //Controls with candle
        controller2.leftBumper().toggleOnTrue(Commands.sequence(new IntakeAlgaeIn(intakeAlgae),new InstantCommand(()->candle.algaeOn()))); 
        controller2.rightBumper().toggleOnTrue(Commands.sequence(new IntakeAlgaeOut(intakeAlgae),new InstantCommand(()->candle.algaeOff())));
        controller2.leftTrigger().toggleOnTrue(Commands.sequence(new IntakeCoralIn(intakeCoral),new InstantCommand(()->candle.coralOn())));
        controller2.rightTrigger().toggleOnTrue(Commands.sequence(new IntakeCoralOut(intakeCoral),new InstantCommand(()->candle.coralOff())));

        controller1.x().whileTrue(new BarbIn(barb));
        controller3.y().whileTrue(new BarbOut(barb));

        controller1.y().onTrue(new PathfindingCommand(controller1.rightBumper()));
        //controller1.y().onTrue(new InstantCommand(()->getStrafeCommand(controller1.rightBumper(), strafeCommands, altStrafeCommands).schedule()).until(controller1.start()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    
    public static double getRobotSpeed() {
        
        return controller1.getLeftTriggerAxis() >= 0.25 ? 0.3 : 1.0;
    // return 0.7;
    }

    public static double getRobotYawSpeed() {
        
        return controller1.getLeftTriggerAxis() >= 0.25 ? 0.3 : 1;
    // return 0.7;
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        System.out.println(autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }

    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            TunerConstants.DrivetrainConstants, 0,
            VecBuilder.fill(RobotMap.kPositionStdDevX, RobotMap.kPositionStdDevY, Units.degreesToRadians(RobotMap.kPositionStdDevTheta)),
            VecBuilder.fill(RobotMap.kVisionStdDevX, RobotMap.kVisionStdDevY, Units.degreesToRadians(RobotMap.kVisionStdDevTheta)),
            TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight
        );
    }

    public static Command getStrafeCommand(BooleanSupplier offCenter, HashMap<Integer, Command> strafeCommands, HashMap<Integer, Command> altStrafeCommands){

        if (LimelightHelpers.getTV(RobotMap.LIMELIGHT_NAME)){
            int viewedID = (int) LimelightHelpers.getFiducialID(RobotMap.LIMELIGHT_NAME);
            Pair<Integer, Integer> blueRange = new Pair<Integer, Integer>(17,22);
            Pair<Integer, Integer> redRange = new Pair<Integer, Integer>(6,11);
            if ((viewedID>=blueRange.getFirst() && viewedID <= blueRange.getSecond()) || (viewedID>=redRange.getFirst() && viewedID <= redRange.getSecond())){
                if (offCenter.getAsBoolean()){
                    return altStrafeCommands.get(viewedID);
                }
                return strafeCommands.get(viewedID);
            }
        }
        return Commands.none();
    }

    public static Command defineEndPos(boolean offCenter, int id){
        double farX = 1;
        double xOffset = 0.67;
        double yOffset = offCenter ? -0.2 : 0.1;

        var tagPose = RobotMap.WELDED_FIELD2025.getTagPose(id).get();
        double tagRotation = tagPose.getRotation().getAngle();
        double angleOffset = Math.atan(yOffset/xOffset);
        double offset = Math.sqrt(Math.pow(xOffset,2) + Math.pow(yOffset,2));

        double startX = tagPose.getX() + farX*Math.cos(tagRotation);
        double startY = tagPose.getY() + farX*Math.sin(tagRotation);
        double endX = tagPose.getX() + offset*Math.cos(tagRotation+angleOffset);
        double endY = tagPose.getY() + offset*Math.sin(tagRotation+angleOffset);
        
        PathConstraints contraints = new PathConstraints(2, 1, Math.PI, Math.PI*2);
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(startX, startY, Rotation2d.fromRadians(angleOffset+tagRotation)),
            new Pose2d(endX, endY, Rotation2d.fromRadians(angleOffset+tagRotation))
        );

        SmartDashboard.putNumber("StartX" + id, startX);
        SmartDashboard.putNumber("StartY" + id, startY);
        SmartDashboard.putNumber("EndX" + id, endX);
        SmartDashboard.putNumber("EndY" + id, endY);
        PathPlannerPath newPath = new PathPlannerPath(waypoints, contraints, null, new GoalEndState(0.0, Rotation2d.fromRadians(tagRotation+Math.PI)));
        newPath.preventFlipping = true;

        // return AutoBuilder.pathfindThenFollowPath(
        //     newPath,
        //     contraints
        // );
        return AutoBuilder.pathfindToPose(
            new Pose2d(endX, endY, Rotation2d.fromRadians(tagRotation+Math.PI)),
            contraints
        );
    }
}