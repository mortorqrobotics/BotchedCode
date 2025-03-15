package frc.BotchedCode.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAlgae extends SubsystemBase {
    
    // private final SparkMax leftIntakeAlgae;
    // private final SparkMax rightIntakeAlgae;
    private boolean leds;


    public IntakeAlgae(){
        //leftIntakeAlgae = new SparkMax(RobotMap.LEFT_INTAKEALGAE_ID, MotorType.kBrushless);
        //rightIntakeAlgae = new SparkMax(RobotMap.RIGHT_INTAKEALGAE_ID, MotorType.kBrushless);
        leds = false;
        
        //leftIntakeAlgae.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //rightIntakeAlgae.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    }

    public void in(){
        // leftIntakeAlgae.set(-RobotMap.INTAKEALGAE_SPEED);
        // rightIntakeAlgae.set(RobotMap.INTAKEALGAE_SPEED);
    }

    public void out(){
        // leftIntakeAlgae.set((RobotMap.INTAKEALGAE_SPEED-0.2));
        // rightIntakeAlgae.set(-(RobotMap.INTAKEALGAE_SPEED-0.2));
    }

    public boolean pickedUp(){
        //return (leftIntakeAlgae.getOutputCurrent() > RobotMap.ALGAE_INTAKE_CURRENT_PICKUP) && (rightIntakeAlgae.getOutputCurrent() > RobotMap.ALGAE_INTAKE_CURRENT_PICKUP);
        return true;
    }

    public boolean released(){
        //return (leftIntakeAlgae.getOutputCurrent() < RobotMap.ALGAE_INTAKE_CURRENT_PICKUP) && (rightIntakeAlgae.getOutputCurrent() < RobotMap.ALGAE_INTAKE_CURRENT_PICKUP);
        return true;
    }

    public void ledsOff(){
        leds = false;
    }

    public void ledsOn(){
        leds = true;
    }

    public boolean getLeds(){
        return leds;
    }

    public void end(){
        // leftIntakeAlgae.set(-0.05);
        // rightIntakeAlgae.set(0.05);
    }
}
