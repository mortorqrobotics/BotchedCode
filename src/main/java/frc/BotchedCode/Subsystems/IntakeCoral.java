package frc.BotchedCode.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;

public class IntakeCoral extends SubsystemBase {
    
    private final SparkMax leftIntakeCoral;
    private final SparkMax rightIntakeCoral;


    public IntakeCoral(){
        leftIntakeCoral = new SparkMax(RobotMap.LEFT_INTAKECORAL_ID, MotorType.kBrushless);
        rightIntakeCoral = new SparkMax(RobotMap.RIGHT_INTAKECORAL_ID, MotorType.kBrushless);
        
        leftIntakeCoral.configure(new SparkMaxConfig().inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftIntakeCoral.configure(new SparkMaxConfig().inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    }

    public void in(){
        leftIntakeCoral.set(-RobotMap.LEFT_INTAKECORAL_SPEED);
        rightIntakeCoral.set(RobotMap.RIGHT_INTAKECORAL_SPEED);
    }

    public void out(){
        leftIntakeCoral.set(RobotMap.LEFT_INTAKECORAL_SPEED);
        rightIntakeCoral.set(-RobotMap.RIGHT_INTAKECORAL_SPEED);
    }

    public boolean pickedUp(){
        return (leftIntakeCoral.getOutputCurrent() > RobotMap.CORAL_INTAKE_CURRENT_PICKUP) && (rightIntakeCoral.getOutputCurrent() > RobotMap.CORAL_INTAKE_CURRENT_PICKUP);
    }

    public boolean released(){
        return (leftIntakeCoral.getOutputCurrent() < RobotMap.CORAL_INTAKE_CURRENT_PICKUP) && (rightIntakeCoral.getOutputCurrent() < RobotMap.CORAL_INTAKE_CURRENT_PICKUP);
    }

    public void end(){
        leftIntakeCoral.set(0);
        rightIntakeCoral.set(0);
    }
}
