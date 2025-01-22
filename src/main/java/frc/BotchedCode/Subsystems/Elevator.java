package frc.BotchedCode.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.RobotContainer;

public class Elevator extends SubsystemBase{

    public TalonFX mElevator;
    private DigitalInput bottomSwitch;
    private PIDController heightController;
    private double setpoint;

    public Elevator(){
        mElevator = new TalonFX(RobotMap.ELEVATOR_ID, RobotMap.SUBSYSTEM_BUS); //TODO
        bottomSwitch = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH_CHANNEL); //TODO
        heightController = new PIDController(0, 0, 0); //TODO
        setpoint = 0;
        
    }
    
    public void up(){
        // mElevator.set(RobotMap.ELEVATOR_SPEED);
        RobotContainer.elevator.setSetpoint(mElevator.getPosition().getValueAsDouble() + RobotMap.MANUAL_ELEVATOR_INCREMENTATION);
    }

    public void down(){
        // mElevator.set(-RobotMap.ELEVATOR_SPEED);
        if(bottomSwitchTriggered()){
            mElevator.setPosition(0);
            setSetpoint(mElevator.getPosition().getValueAsDouble());
        }
        else{
            setSetpoint(mElevator.getPosition().getValueAsDouble() - RobotMap.MANUAL_ELEVATOR_INCREMENTATION);
        }
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void zeroEncoder(){
        mElevator.setPosition(0);
    }

    public boolean bottomSwitchTriggered(){
        return bottomSwitch.get();
    }

    public void end(){
        mElevator.set(0);
    }
    @Override
    public void periodic(){
        mElevator.set(heightController.calculate(mElevator.getPosition().getValueAsDouble(), setpoint));

    }
}