package frc.BotchedCode.Subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.RobotContainer;

public class Elevator extends SubsystemBase{

    public TalonFX mElevator;
    public TalonFX mElevator2; 
    private DigitalInput bottomSwitch;
    private ProfiledPIDController heightController;
    private double setpoint;
    private boolean encoderZeroed;


    public Elevator(){
        mElevator = new TalonFX(RobotMap.ELEVATOR_ID, RobotMap.SUBSYSTEM_BUS); //TODO
        mElevator2 = new TalonFX(RobotMap.ELEVATOR2_ID, RobotMap.SUBSYSTEM_BUS); //TODO
        mElevator.setControl(new Follower(mElevator2.getDeviceID(), false)); // TODO check mount for inverted motor
        
        bottomSwitch = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH_CHANNEL); //TODO
        heightController = new ProfiledPIDController(RobotMap.ELEVATOR_KP, RobotMap.ELEVATOR_KI, RobotMap.ELEVATOR_KD, new TrapezoidProfile.Constraints(RobotMap.ELEVATOR_MAX_SPEED, RobotMap.ELEVATOR_MAX_ACCELERATION)); //TODO
        setpoint = 0;
        encoderZeroed = false;
        
    }
    
    public void up(){
        // mElevator.set(RobotMap.ELEVATOR_SPEED);
        RobotContainer.elevator.setSetpoint(mElevator.getPosition().getValueAsDouble() + RobotMap.MANUAL_ELEVATOR_INCREMENTATION);

    }

    public void down(){
        // mElevator.set(-RobotMap.ELEVATOR_SPEED);
        setSetpoint(mElevator.getPosition().getValueAsDouble() - RobotMap.MANUAL_ELEVATOR_INCREMENTATION);
    }

    public void manualDown(){
        mElevator.set(-RobotMap.MANUAL_ELEVATOR_SPEED);
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
        return !bottomSwitch.get();
    }

    public boolean reachedUpperLimit(){
        return mElevator.getPosition().getValueAsDouble() > RobotMap.ELEVATOR_UPPER_LIMIT;
    }

    public void end(){
        mElevator.set(0);
    }
    @Override
    public void periodic(){
        if(encoderZeroed){
            double speed = heightController.calculate(mElevator.getPosition().getValueAsDouble(), setpoint);
            if((reachedLowerLimit() && speed < 0) || (reachedUpperLimit() && speed > 0)){
                mElevator.set(0);
            }
            else{
                mElevator.set(speed);
            }
        }
    }
}