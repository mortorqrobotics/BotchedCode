package frc.BotchedCode.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;

public class Elevator extends SubsystemBase{

    public TalonFX mElevator;
    public TalonFX mElevator2; 
    private DigitalInput bottomSwitch;
    private double setpoint;
    private boolean encoderZeroed;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    private CANrange canrange;


    public Elevator(){
        mElevator = new TalonFX(RobotMap.ELEVATOR_ID, "1515Canivore"); //TODO
        mElevator2 = new TalonFX(RobotMap.ELEVATOR2_ID, "1515Canivore"); //TODO
        mElevator2.setControl(new Follower(mElevator.getDeviceID(), false)); // TODO check mount for inverted motor

        var talonFXConfigs = new TalonFXConfiguration();
        //talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 3; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        slot0Configs.kG = 0.5;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        mElevator.getConfigurator().apply(talonFXConfigs);
        
        //bottomSwitch = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH_CHANNEL); //TODO
        canrange = new CANrange(0, "1515Canivore");
        setpoint = 0;
        encoderZeroed = false;
        zeroEncoder(); //TODO
        
    }
    
    public void up(){
        //mElevator.set(RobotMap.ELEVATOR_SPEED);
        setpoint+=0.1;
    }

    public void down(){
        //mElevator.set(-RobotMap.ELEVATOR_SPEED);
        setpoint-=0.1;
    }

    public double getPosition(){
        return mElevator.getPosition().getValueAsDouble();
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void zeroEncoder(){
        encoderZeroed = true;
        mElevator.setPosition(0);
        setSetpoint(0);
    }

    public boolean reachedLowerLimit(){ 
        //return !bottomSwitch.get();
        return true;
    }

    public boolean reachedUpperLimit(){
        return mElevator.getPosition().getValueAsDouble() > RobotMap.ELEVATOR_UPPER_LIMIT;
    }

    public void end(){
        mElevator.set(0);
        //mElevator2.set(0);
    }

    public boolean atSetpoint(){
        return Math.abs(getPosition()-setpoint)<0.3;
    }

    public void resetCanRange(){
        if (canrange.getDistance().getValueAsDouble()>0.05){
            //mElevator.setPosition(canrange.getDistance().getValueAsDouble()*26.67 + 1.08);
        }
    }

    // public void resetLimitSwitch(){
    //     mElevator.setPosition(RobotMap.L2_HEIGHT);
    // }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator encoder", mElevator.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Height", canrange.getDistance().getValueAsDouble());
        //if(encoderZeroed){
        mElevator.setControl(m_request.withPosition(setpoint));
        // if (!bottomSwitch.get()){
        //     resetLimitSwitch();
        // }
        //}
    }
}