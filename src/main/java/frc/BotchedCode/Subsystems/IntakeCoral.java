package frc.BotchedCode.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;

public class IntakeCoral extends SubsystemBase {
    
    private final SparkMax leftIntakeCoral;
    private final SparkMax rightIntakeCoral;
    private boolean leds;


    public IntakeCoral(){
        leftIntakeCoral = new SparkMax(RobotMap.LEFT_INTAKECORAL_ID, MotorType.kBrushless);
        rightIntakeCoral = new SparkMax(RobotMap.RIGHT_INTAKECORAL_ID, MotorType.kBrushless);
        leds = false;
        
        leftIntakeCoral.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightIntakeCoral.configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    }

    public void in(){
        leftIntakeCoral.set(-RobotMap.INTAKECORAL_SPEED);
        rightIntakeCoral.set(RobotMap.INTAKECORAL_SPEED);
    }

    public void out(){
        leftIntakeCoral.set(RobotMap.INTAKECORAL_SPEED);
        rightIntakeCoral.set(-RobotMap.INTAKECORAL_SPEED);
    }

    public boolean pickedUp(){
        return (leftIntakeCoral.getOutputCurrent() > RobotMap.CORAL_INTAKE_CURRENT_PICKUP) && (rightIntakeCoral.getOutputCurrent() > RobotMap.CORAL_INTAKE_CURRENT_PICKUP);
    }


    
    // public boolean released(){
    //     return (leftIntakeCoral.getOutputCurrent() < RobotMap.CORAL_INTAKE_CURRENT_PICKUP) && (rightIntakeCoral.getOutputCurrent() < RobotMap.CORAL_INTAKE_CURRENT_PICKUP/2);
    // }

    public void end(){
        leftIntakeCoral.set(0);
        rightIntakeCoral.set(0);
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
    
    // @Override
    // public void periodic(){
    //         SmartDashboard.putNumber("LIntakeDraw", leftIntakeCoral.getOutputCurrent());
    //         SmartDashboard.putNumber("RIntakeDraw", rightIntakeCoral.getOutputCurrent());
            
    // }
}
