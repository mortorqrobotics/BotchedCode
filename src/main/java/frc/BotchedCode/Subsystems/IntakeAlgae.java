package frc.BotchedCode.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;

public class IntakeAlgae extends SubsystemBase {
    
    private final SparkMax leftIntakeAlgae;
    private final SparkMax rightIntakeAlgae;
    private boolean leds;


    public IntakeAlgae(){
        leftIntakeAlgae = new SparkMax(RobotMap.LEFT_INTAKEALGAE_ID, MotorType.kBrushless);
        rightIntakeAlgae = new SparkMax(RobotMap.RIGHT_INTAKEALGAE_ID, MotorType.kBrushless);
        leds = false;
        
        leftIntakeAlgae.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightIntakeAlgae.configure(new SparkMaxConfig().inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    }

    public void in(){
        leftIntakeAlgae.set(-RobotMap.LEFT_INTAKEALGAE_SPEED);
        rightIntakeAlgae.set(RobotMap.RIGHT_INTAKEALGAE_SPEED);
    }

    public void out(){
        leftIntakeAlgae.set(RobotMap.LEFT_INTAKEALGAE_SPEED);
        rightIntakeAlgae.set(-RobotMap.RIGHT_INTAKEALGAE_SPEED);
    }

    public boolean pickedUp(){
        return (leftIntakeAlgae.getOutputCurrent() > RobotMap.ALGAE_INTAKE_CURRENT_PICKUP) && (rightIntakeAlgae.getOutputCurrent() > RobotMap.ALGAE_INTAKE_CURRENT_PICKUP);
    }

    public boolean released(){
        return (leftIntakeAlgae.getOutputCurrent() < RobotMap.ALGAE_INTAKE_CURRENT_PICKUP) && (rightIntakeAlgae.getOutputCurrent() < RobotMap.ALGAE_INTAKE_CURRENT_PICKUP);
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
        leftIntakeAlgae.set(0);
        rightIntakeAlgae.set(0);
    }
}
