package frc.BotchedCode.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.RobotContainer;

public class Elevator extends SubsystemBase{

    public TalonFX mElevator;
    private DigitalInput bottomSwitch;
    private ProfiledPIDController heightController;
    private double setpoint;

    public Elevator(){
        mElevator = new TalonFX(RobotMap.ELEVATOR_ID, RobotMap.SUBSYSTEM_BUS); //TODO
        bottomSwitch = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH_CHANNEL); //TODO
        heightController = new ProfiledPIDController(RobotMap.ELEVATOR_KP, RobotMap.ELEVATOR_KI, RobotMap.ELEVATOR_KD, new TrapezoidProfile.Constraints(RobotMap.ELEVATOR_MAX_SPEED, RobotMap.ELEVATOR_MAX_ACCELERATION)); //TODO
        setpoint = 0;
        
    }
    
    public void up(){
        // mElevator.set(RobotMap.ELEVATOR_SPEED);
        RobotContainer.elevator.setSetpoint(mElevator.getPosition().getValueAsDouble() + RobotMap.MANUAL_ELEVATOR_INCREMENTATION);
    }

    public void down(){
        // mElevator.set(-RobotMap.ELEVATOR_SPEED);
        setSetpoint(mElevator.getPosition().getValueAsDouble() - RobotMap.MANUAL_ELEVATOR_INCREMENTATION);
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void zeroEncoder(){
        mElevator.setPosition(0);
    }

    public boolean reachedLowerLimit(){
        return bottomSwitch.get();
    }

    public boolean reachedUpperLimit(){
        return mElevator.getPosition().getValueAsDouble() > RobotMap.ELEVATOR_UPPER_LIMIT;
    }

    public void end(){
        mElevator.set(0);
    }
    @Override
    public void periodic(){
        double speed = heightController.calculate(mElevator.getPosition().getValueAsDouble(), setpoint);
        if((reachedLowerLimit() && speed < 0) || (reachedUpperLimit() && speed > 0)){
            mElevator.set(0);
        }
        else{
            mElevator.set(speed);
        }
    }
}