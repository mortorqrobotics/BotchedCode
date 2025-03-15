// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.BotchedCode.Commands.Pathfinding.PathfindToID;
import frc.BotchedCode.Commands.Pathfinding.PathfindToNearest;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.Constants.TunerConstants;
import frc.BotchedCode.Subsystems.Barb;
import frc.BotchedCode.Subsystems.Candle;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;
import frc.BotchedCode.Subsystems.Elevator;
import frc.BotchedCode.Subsystems.IntakeAlgae;
import frc.BotchedCode.Subsystems.IntakeCoral;
import frc.BotchedCode.Subsystems.Pivot;



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
    public static Pigeon2 gyro;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    
    

    public RobotContainer() {

        gyro = new Pigeon2(RobotMap.PIGEON_ID, "1515Canivore");
        elevator = new Elevator();
        pivot = new Pivot();
        intakeAlgae = new IntakeAlgae();
        intakeCoral = new IntakeCoral();
        barb = new Barb();
        candle = new Candle(()->intakeCoral.getLeds(), ()->intakeAlgae.getLeds());

        NamedCommands.registerCommand("L2Routine", Commands.sequence(Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L2_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        ), Commands.sequence(new IntakeCoralOut(intakeCoral),new InstantCommand(()->candle.coralOff())), Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.REST_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        )));
        NamedCommands.registerCommand("L3Routine", Commands.sequence(Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L3_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        ), Commands.sequence(new IntakeCoralOut(intakeCoral),new InstantCommand(()->candle.coralOff())), Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.REST_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        )));
        NamedCommands.registerCommand("L4Routine", Commands.sequence(Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L4_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L4_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        ), Commands.sequence(new IntakeCoralOut(intakeCoral),new InstantCommand(()->candle.coralOff())), Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.REST_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        )));
        NamedCommands.registerCommand("L3RoutineAlgae", Commands.sequence(Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L3_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        ), Commands.parallel(Commands.sequence(new IntakeCoralOut(intakeCoral),new InstantCommand(()->candle.coralOff())), Commands.sequence(new IntakeAlgaeIn(intakeAlgae),new InstantCommand(()->candle.algaeOn()))), Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.REST_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        )));

        NamedCommands.registerCommand("L2Position", Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L2_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        ));
        NamedCommands.registerCommand("L3Position", Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L3_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        ));
        NamedCommands.registerCommand("L4Position", Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L4_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L4_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        ));
        NamedCommands.registerCommand("RestPosition", Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.REST_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        ));
        NamedCommands.registerCommand("ProcessorPosition", Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L2_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        ));

        NamedCommands.registerCommand("IntakeAlgae", Commands.sequence(new IntakeAlgaeIn(intakeAlgae),new InstantCommand(()->candle.algaeOn())));
        NamedCommands.registerCommand("OuttakeAlgae", Commands.sequence(new IntakeAlgaeOut(intakeAlgae),new InstantCommand(()->candle.algaeOff())));
        NamedCommands.registerCommand("IntakeCoral", Commands.sequence(new IntakeCoralIn(intakeCoral),new InstantCommand(()->candle.coralOn())));
        NamedCommands.registerCommand("OuttakeCoral", Commands.sequence(new IntakeCoralOut(intakeCoral),new InstantCommand(()->candle.coralOff())));

        NamedCommands.registerCommand("Startup", Commands.parallel(Commands.sequence(new IntakeCoralIn(intakeCoral),new InstantCommand(()->candle.coralOn())), 
        Commands.sequence(
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.REST_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE))),
            new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint())
        )));

        NamedCommands.registerCommand("Strafe To 6", new PathfindToID(drivetrain, false, DriverStation.getAlliance().get() == Alliance.Red ? 6 : 19));
        NamedCommands.registerCommand("Strafe To 7", new PathfindToID(drivetrain, false,DriverStation.getAlliance().get() == Alliance.Red ? 7 : 18));
        NamedCommands.registerCommand("Strafe To 8", new PathfindToID(drivetrain, false, DriverStation.getAlliance().get() == Alliance.Red ? 8 : 17));
        NamedCommands.registerCommand("Strafe To 9", new PathfindToID(drivetrain, false, DriverStation.getAlliance().get() == Alliance.Red ? 9 :22));
        NamedCommands.registerCommand("Strafe To 10", new PathfindToID(drivetrain, false, DriverStation.getAlliance().get() == Alliance.Red ? 10 : 21));
        NamedCommands.registerCommand("Strafe To 11", new PathfindToID(drivetrain, false, DriverStation.getAlliance().get() == Alliance.Red ? 11 : 20));

        NamedCommands.registerCommand("Strafe To alt6", new PathfindToID(drivetrain, true, DriverStation.getAlliance().get() == Alliance.Red ? 6 : 19));
        NamedCommands.registerCommand("Strafe To alt7", new PathfindToID(drivetrain, true, DriverStation.getAlliance().get() == Alliance.Red ? 7 : 18));
        NamedCommands.registerCommand("Strafe To alt8", new PathfindToID(drivetrain, true, DriverStation.getAlliance().get() == Alliance.Red ? 8 : 17));
        NamedCommands.registerCommand("Strafe To alt9", new PathfindToID(drivetrain, true, DriverStation.getAlliance().get() == Alliance.Red ? 9 :22));
        NamedCommands.registerCommand("Strafe To alt10", new PathfindToID(drivetrain, true, DriverStation.getAlliance().get() == Alliance.Red ? 10 : 21));
        NamedCommands.registerCommand("Strafe To alt11", new PathfindToID(drivetrain, true, DriverStation.getAlliance().get() == Alliance.Red ? 11 : 20));

        autoChooser = AutoBuilder.buildAutoChooser("0 Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("Reset Gyro", Commands.sequence(new InstantCommand(()->gyro.setYaw(DriverStation.getAlliance().get() == Alliance.Blue ? Math.PI: 0)).ignoringDisable(true), new InstantCommand(()->drivetrain.resetRotation(new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? Math.PI: 0))).ignoringDisable(true)));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller1.getLeftY() * MaxSpeed * getRobotSpeed()) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller1.getLeftX() * MaxSpeed * getRobotSpeed()) // Drive left with negative X (left)
                    .withRotationalRate(-controller1.getRightX() * MaxAngularRate * getRobotYawSpeed()) // Drive counterclockwise with negative X (left)
            )
        );

        //controller1.back().onTrue( new InstantCommand(()->gyro.setYaw(DriverStation.getAlliance().get() == Alliance.Blue ? 180: 0)));

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

        // reset the field-centric heading on start button press
        controller1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        controller2.a().onTrue(Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L2_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE)))); //TODO
        controller2.b().onTrue(Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L3_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))));
        controller2.y().onTrue(Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L4_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L4_ANGLE))));
        controller2.x().onTrue(Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L4_PROCESSOR_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE))));
        controller2.start().onTrue(Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.REST_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE)))); //TODO

        controller2.povUp().whileTrue(new ManualElevatorUp(elevator));
        controller2.povDown().whileTrue(new ManualElevatorDown(elevator));
        controller2.povRight().whileTrue(new ManualPivotUp(pivot));
        controller2.povLeft().whileTrue(new ManualPivotDown(pivot));

        controller2.leftBumper().toggleOnTrue(Commands.sequence(new IntakeAlgaeIn(intakeAlgae),new InstantCommand(()->candle.algaeOn()))); 
        controller2.rightBumper().toggleOnTrue(Commands.sequence(new IntakeAlgaeOut(intakeAlgae),new InstantCommand(()->candle.algaeOff())));
        controller2.leftTrigger().toggleOnTrue(Commands.sequence(new IntakeCoralIn(intakeCoral),new InstantCommand(()->candle.coralOn())));
        controller2.rightTrigger().toggleOnTrue(Commands.sequence(new IntakeCoralOut(intakeCoral),new InstantCommand(()->candle.coralOff())));

        controller1.x().whileTrue(new BarbIn(barb));
        controller3.y().whileTrue(new BarbOut(barb));

        Command driveToNearestReefSideCommandLeft = new PathfindToNearest(drivetrain,
        true);
        Command driveToNearestReefSideCommandRight = new PathfindToNearest(drivetrain,
            false);
        controller1.leftBumper().onTrue(driveToNearestReefSideCommandLeft);
        controller1.rightBumper().onTrue(driveToNearestReefSideCommandRight);

        //controller1.y().and(controller1.rightTrigger().negate()).onTrue(new PathfindingCommand(drivetrain));
        //controller1.y().and(controller1.rightTrigger()).onTrue(new PathfindingCommandAlt(drivetrain));
        //controller1.y().onTrue(new InstantCommand(()->getStrafeCommand(controller1.rightBumper(), strafeCommands, altStrafeCommands).schedule()).until(controller1.start()));

        //drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    
    public static double getRobotSpeed() {
        
        return controller1.leftTrigger().getAsBoolean() ? 0.3 : 1.0;
    }

    public static double getRobotYawSpeed() {
        
        return controller1.leftTrigger().getAsBoolean() ? 0.3 : 1;
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        //System.out.println(autoChooser.getSelected().getName());
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
}