// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.BotchedCode;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.VecBuilder;
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
import frc.BotchedCode.Commands.RotateToTag;
import frc.BotchedCode.Commands.StrafeToTag;
import frc.BotchedCode.Commands.Barb.BarbIn;
import frc.BotchedCode.Commands.Barb.BarbOut;
import frc.BotchedCode.Commands.Intakes.IntakeAlgaeIn;
import frc.BotchedCode.Commands.Intakes.IntakeAlgaeOut;
import frc.BotchedCode.Commands.Intakes.IntakeCoralIn;
import frc.BotchedCode.Commands.Intakes.IntakeCoralOut;
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
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final static SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final static CommandXboxController controller1 = new CommandXboxController(0);
    private final static CommandXboxController controller2 = new CommandXboxController(1);
    private final static CommandXboxController controller3 = new CommandXboxController(2);
    
    public final static CommandSwerveDrivetrain drivetrain = createDrivetrain();
    public static Pigeon2 gyro = new Pigeon2(RobotMap.PIGEON_ID);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    
    

    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        elevator = new Elevator();

        pivot = new Pivot();

        intakeAlgae = new IntakeAlgae();

        intakeCoral = new IntakeCoral();

        barb = new Barb();

        candle = new Candle(()->intakeCoral.getLeds(), ()->intakeAlgae.getLeds());

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        Command[] intakes ={
            Commands.sequence(new IntakeAlgaeIn(intakeAlgae),new InstantCommand(()->candle.algaeOn())),
            Commands.sequence(new IntakeAlgaeOut(intakeAlgae),new InstantCommand(()->candle.algaeOff())),
            Commands.sequence(new IntakeCoralIn(intakeCoral),new InstantCommand(()->candle.coralOn())),
            Commands.sequence(new IntakeCoralOut(intakeCoral),new InstantCommand(()->candle.coralOff()))
        };
        Command[] positions = {
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L2_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))),
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L3_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L23_ANGLE))),
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.L4_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.L4_ANGLE))),
            Commands.parallel(new InstantCommand(()-> elevator.setSetpoint(RobotMap.REST_HEIGHT)), new InstantCommand(()-> pivot.setSetpoint(RobotMap.REST_ANGLE)))
        };
        Command atSetpoint = new WaitUntilCommand(() -> elevator.atSetpoint() && pivot.atSetpoint());

        NamedCommands.registerCommand("L2Position", Commands.sequence(positions[0], atSetpoint));
        NamedCommands.registerCommand("L3Position", Commands.sequence(positions[1], atSetpoint));
        NamedCommands.registerCommand("L4Position", Commands.sequence(positions[2], atSetpoint));
        NamedCommands.registerCommand("RestPosition", Commands.sequence(positions[3], atSetpoint));

        NamedCommands.registerCommand("IntakeAlgae", intakes[0]);
        NamedCommands.registerCommand("OuttaleAlgae", intakes[1]);
        NamedCommands.registerCommand("IntakeCoral", intakes[2]);
        NamedCommands.registerCommand("OuttakeCoral", intakes[3]);

        NamedCommands.registerCommand("Startup", Commands.parallel(intakes[2], Commands.sequence(positions[3], atSetpoint)));

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
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        controller1.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controller1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        controller2.a().onTrue(positions[0]); //TODO
        controller2.b().onTrue(positions[1]);
        controller2.y().onTrue(positions[2]);
        //controller2.x().onTrue(Positions[3]);
        controller2.start().onTrue(positions[3]); //TODO

        //controller2.x().whileTrue(new InstantCommand(()->elevator.down()));
        //controller2.y().whileTrue(new InstantCommand(()-> elevator.up()));


        //Controls for intakes without candle
        // controller2.leftBumper().onTrue(new IntakeAlgaeIn(intakeAlgae)); 
        // controller2.rightBumper().onTrue(new IntakeAlgaeOut(intakeAlgae));

        // controller2.leftTrigger().onTrue(new IntakeCoralIn(intakeCoral)); 
        // controller2.rightTrigger().onTrue(new IntakeCoralOut(intakeCoral));

        //Controls with candle
        controller2.leftBumper().onTrue(intakes[0]); 
        controller2.rightBumper().onTrue(intakes[1]);
        controller2.leftTrigger().onTrue(intakes[2]);
        controller2.rightTrigger().onTrue(intakes[3]);

        controller1.x().whileTrue(new BarbIn(barb));
        controller3.y().whileTrue(new BarbOut(barb));

        controller1.y().onTrue(Commands.sequence(new RotateToTag(drivetrain), new StrafeToTag(drivetrain, true)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    
    public static double getRobotSpeed() {
        
        return controller1.getLeftTriggerAxis() >= 0.25 ? 0.1 : 1.0;
    // return 0.7;
    }

    public static double getRobotYawSpeed() {
        
        return controller1.getLeftTriggerAxis() >= 0.25 ? 0.1 : 0.7*(1.0/0.9);
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
}