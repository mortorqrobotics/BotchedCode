package frc.BotchedCode.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;

public class Pivot extends SubsystemBase{

    public TalonFX mpivot;
    public CANcoder mCANcoder;
    private double setpoint;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0.0);
    boolean encoderZeroed;
    boolean wrapping = false;

    public Pivot(){
        mpivot = new TalonFX(RobotMap.PIVOT_ID); //TODO
        mCANcoder = new CANcoder(RobotMap.PIVOT_CANCODER_ID);

        var talonFXConfigs = new TalonFXConfiguration();

        // talonFXConfigs.Feedback.FeedbackRemoteSensorID = mCANcoder.getDeviceID();
        // talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        //talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 10; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        mpivot.getConfigurator().apply(talonFXConfigs);

        resetEncoder();

        setpoint = getPosition();
    }
    
    public void up(){
        mpivot.set(RobotMap.PIVOT_SPEED);
    }
    public void down(){
        mpivot.set(-RobotMap.PIVOT_SPEED);

    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public double getPosition(){
        return mpivot.getPosition().getValueAsDouble();
    }

    public boolean atSetpoint(){
        return Math.abs(getPosition()-setpoint)<1;
    }

    public boolean reachedLowerLimit(){
        return getPosition() < RobotMap.PIVOT_LOWER_LIMIT;
    }

    public boolean reachedUpperLimit(){
        return getPosition() > RobotMap.PIVOT_UPPER_LIMIT;
    }

    public boolean currentSpike(){
        return Math.abs(mpivot.getStatorCurrent().getValueAsDouble()) > RobotMap.PIVOT_CURRENT_LIMIT;
    }

    public void end(){
        mpivot.set(0);
    }

    public void zeroEncoder(){ 
        encoderZeroed = true;
        mpivot.setPosition(-1);
        setSetpoint(0);
    }

    public double getCANCoderValue(){
        double position = mCANcoder.getAbsolutePosition().getValueAsDouble();
        if (position<0 && wrapping){
            return position+1;
        }
        return position;
    }

    public void resetEncoder(){
        encoderZeroed = true;
        mpivot.setPosition(100*getCANCoderValue());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pivot Angle", getPosition());
        SmartDashboard.putNumber("Pivot Cancoder Angle", mCANcoder.getAbsolutePosition().getValueAsDouble());

        if (encoderZeroed){
            mpivot.setControl(m_request.withPosition(setpoint));
        }
    }
}
