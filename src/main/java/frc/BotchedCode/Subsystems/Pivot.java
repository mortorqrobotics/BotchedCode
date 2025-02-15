package frc.BotchedCode.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;

public class Pivot extends SubsystemBase{

    public TalonFX mpivot;
    private ProfiledPIDController angleController;
    private double setpoint;
    private CANcoder canCoder;
    private boolean wrapping;

    public Pivot(){
        mpivot = new TalonFX(RobotMap.PIVOT_ID, RobotMap.SUBSYSTEM_BUS); //TODO

        canCoder = new CANcoder(RobotMap.PIVOT_CANCODER_ID);
        canCoder.clearStickyFault_BadMagnet();
        canCoder.getConfigurator().apply(new CANcoderConfiguration());
        wrapping = true;

        angleController = new ProfiledPIDController(RobotMap.PIVOT_KP, RobotMap.PIVOT_KI, RobotMap.PIVOT_KD, new TrapezoidProfile.Constraints(RobotMap.PIVOT_MAX_SPEED, RobotMap.PIVOT_MAX_ACCELERATION)); //TODO
        setpoint = getCANCoderValue();
    }
    
    public void up(){
        // mpivot.set(RobotMap.pivot_SPEED);
        setSetpoint(getSetpoint() + RobotMap.MANUAL_PIVOT_INCREMENTATION);
        // mpivot.set(-RobotMap.PIVOT_MAX_SPEED);
    }

    public void down(){
        // mpivot.set(-RobotMap.pivot_SPEED);
        setSetpoint(getSetpoint() - RobotMap.MANUAL_PIVOT_INCREMENTATION);
        // mpivot.set(RobotMap.PIVOT_MAX_SPEED);

    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public double getSetpoint(){
        return setpoint;
    }

    public double getCANCoderValue(){
        //looping
        
        if (wrapping){
            if (canCoder.getAbsolutePosition().getValueAsDouble() < 0){
                return canCoder.getAbsolutePosition().getValueAsDouble()+1;
            }
            else{
                return canCoder.getAbsolutePosition().getValueAsDouble();
            }
        }
        else{
            return canCoder.getAbsolutePosition().getValueAsDouble();
        }

    }

    public boolean reachedLowerLimit(){
        return getCANCoderValue() < RobotMap.PIVOT_LOWER_LIMIT;
    }

    public boolean reachedUpperLimit(){
        return getCANCoderValue() > RobotMap.PIVOT_UPPER_LIMIT;
    }

    public void end(){
        mpivot.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pivot Angle", getCANCoderValue());
        double speed = angleController.calculate(getCANCoderValue(), setpoint);
        // if((reachedLowerLimit() && speed < 0) || (reachedUpperLimit() && speed > 0)){
        //     mpivot.set(0);
        // }
        // else{
            mpivot.set(-speed);
        // }
    }
}
