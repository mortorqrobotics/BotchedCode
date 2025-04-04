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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.BotchedCode.Commands.Auto.ProcessorSide;
import frc.BotchedCode.Commands.Barb.BarbIn;
import frc.BotchedCode.Commands.Barb.BarbInIgnoreLimit;
import frc.BotchedCode.Commands.Barb.BarbOut;
import frc.BotchedCode.Commands.ElevatorPivot.AutoLifts.AutoElevatorPivot;
import frc.BotchedCode.Commands.ElevatorPivot.AutoLifts.AutoPivot;
import frc.BotchedCode.Commands.ElevatorPivot.AutoLifts.AutoScoreRoutine;
import frc.BotchedCode.Commands.ElevatorPivot.AutoLifts.TeleopElevatorPivot;
import frc.BotchedCode.Commands.ElevatorPivot.ManualElevatorDown;
import frc.BotchedCode.Commands.ElevatorPivot.ManualElevatorUp;
import frc.BotchedCode.Commands.ElevatorPivot.ManualPivotDown;
import frc.BotchedCode.Commands.ElevatorPivot.ManualPivotUp;
import frc.BotchedCode.Commands.Intakes.IntakeAlgaeIn;
import frc.BotchedCode.Commands.Intakes.IntakeAlgaeOut;
import frc.BotchedCode.Commands.Intakes.IntakeCoralIn;
import frc.BotchedCode.Commands.Intakes.IntakeCoralOut;
import frc.BotchedCode.Commands.Pathfinding.PathfindToNearestReef;
import frc.BotchedCode.Commands.Pathfinding.PathfindToNearestStation;
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

        NamedCommands.registerCommand("L2Routine", new AutoScoreRoutine(elevator, pivot, intakeCoral, "L2"));
        NamedCommands.registerCommand("L3Routine", new AutoScoreRoutine(elevator, pivot, intakeCoral, "L3"));
        NamedCommands.registerCommand("L4Routine", new AutoScoreRoutine(elevator, pivot, intakeCoral, "L4"));
        NamedCommands.registerCommand("UpAngle", new AutoPivot(pivot, "Up"));

        NamedCommands.registerCommand("L2Position", new AutoElevatorPivot(elevator, pivot, "L2"));
        NamedCommands.registerCommand("L3Position", new AutoElevatorPivot(elevator, pivot, "L3"));
        NamedCommands.registerCommand("L4Position", new AutoElevatorPivot(elevator, pivot, "L4"));
        NamedCommands.registerCommand("RestPosition", new AutoElevatorPivot(elevator, pivot, "Rest"));
        
        //NamedCommands.registerCommand("ProcessorPosition", new AutoElevatorPivot(elevator, pivot, "Processor"));

        NamedCommands.registerCommand("IntakeAlgae", new IntakeAlgaeIn(intakeAlgae));
        NamedCommands.registerCommand("OuttakeAlgae", new IntakeAlgaeOut(intakeAlgae));
        NamedCommands.registerCommand("IntakeCoral", new IntakeCoralIn(intakeCoral));
        NamedCommands.registerCommand("OuttakeCoral", new IntakeCoralOut(intakeCoral));

        NamedCommands.registerCommand("Startup", Commands.parallel(new IntakeCoralIn(intakeCoral), new AutoElevatorPivot(elevator, pivot, "Up")));

        autoChooser = AutoBuilder.buildAutoChooser("0 Auto");
        autoChooser.addOption("ManualAuto", new ProcessorSide(drivetrain));
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
        //controller1.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // controller1.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-controller1.getLeftY(), -controller1.getLeftX()))
        // ));
        
        //robot centric d-pad
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

        // reset the field-centric heading on start button press
        controller1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //strafe to nearest
        Command driveToNearestReefSideCommandLeft = new PathfindToNearestReef(drivetrain, true);
        Command driveToNearestReefSideCommandRight = new PathfindToNearestReef(drivetrain, false);
        Command driveToNearestStationSideCommand = new PathfindToNearestStation(drivetrain);
        controller1.leftBumper().onTrue(driveToNearestReefSideCommandLeft.until(controller1.b()));
        controller1.rightBumper().onTrue(driveToNearestReefSideCommandRight.until(controller1.b()));
        controller1.rightTrigger().onTrue(driveToNearestStationSideCommand.until(controller1.b()));

        //barb commands
        controller1.x().whileTrue(new BarbIn(barb));
        controller1.a().whileTrue(new BarbInIgnoreLimit(barb));
        controller3.y().whileTrue(new BarbOut(barb));

        //elevator setpoints
        controller2.a().onTrue(new TeleopElevatorPivot(elevator, pivot, "L2"));
        controller2.b().onTrue(new TeleopElevatorPivot(elevator, pivot, "L3"));
        controller2.y().onTrue(new TeleopElevatorPivot(elevator, pivot, "L4"));
        //controller2.x().onTrue(new TeleopElevatorPivot(elevator, pivot, "Processor"));
        controller2.x().onTrue(new TeleopElevatorPivot(elevator, pivot, "Rest"));

        //manual elevator and pivot
        controller2.povUp().whileTrue(new ManualElevatorUp(elevator));
        controller2.povDown().whileTrue(new ManualElevatorDown(elevator));
        controller2.povRight().whileTrue(new ManualPivotUp(pivot));
        controller2.povLeft().whileTrue(new ManualPivotDown(pivot));

        //intakes and outtakes
        // controller2.leftBumper().toggleOnTrue(new IntakeAlgaeIn(intakeAlgae)); 
        // controller2.rightBumper().toggleOnTrue(new IntakeAlgaeOut(intakeAlgae));
        controller2.leftTrigger().toggleOnTrue(new IntakeCoralIn(intakeCoral));
        controller2.rightTrigger().toggleOnTrue(new IntakeCoralOut(intakeCoral));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
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