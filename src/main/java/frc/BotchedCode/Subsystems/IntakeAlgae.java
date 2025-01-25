package frc.BotchedCode.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;

public class IntakeAlgae extends SubsystemBase {
    
    private SparkMax leftIntakeAlgae;
    private SparkMax rightIntakeAlgae;


    public IntakeAlgae(){
        leftIntakeAlgae = new SparkMax(RobotMap.LEFT_INTAKEALGAE_ID, MotorType.kBrushless);
        rightIntakeAlgae = new SparkMax(RobotMap.RIGHT_INTAKEALGAE_ID, MotorType.kBrushless);
        
        leftIntakeAlgae.configure(new SparkMaxConfig().inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftIntakeAlgae.configure(new SparkMaxConfig().inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    }

    public void in(){
        leftIntakeAlgae.set(-RobotMap.LEFT_INTAKEALGAE_SPEED);
        rightIntakeAlgae.set(RobotMap.RIGHT_INTAKEALGAE_SPEED);
    }

    public void out(){
        leftIntakeAlgae.set(RobotMap.LEFT_INTAKEALGAE_SPEED);
        rightIntakeAlgae.set(-RobotMap.RIGHT_INTAKEALGAE_SPEED);
    }

    public void end(){
        leftIntakeAlgae.set(0);
        rightIntakeAlgae.set(0);
    }
}
