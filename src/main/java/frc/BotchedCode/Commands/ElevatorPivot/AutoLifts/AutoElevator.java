package frc.BotchedCode.Commands.ElevatorPivot.AutoLifts;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.Subsystems.Elevator;

public class AutoElevator extends Command {

    private final Elevator elevator;
    private final String setpoint;
    private HashMap<String, Double> elevatorSetpoints;

    public AutoElevator(Elevator elevator, String setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;

        this.elevatorSetpoints = new HashMap<>();

        this.elevatorSetpoints.put("Rest", RobotMap.REST_HEIGHT);
        this.elevatorSetpoints.put("L2", RobotMap.L2_HEIGHT);
        this.elevatorSetpoints.put("L3", RobotMap.L3_HEIGHT);
        this.elevatorSetpoints.put("L4", RobotMap.L4_HEIGHT);
        this.elevatorSetpoints.put("Processor", RobotMap.PROCESSOR_HEIGHT);
        this.elevatorSetpoints.put("Up", RobotMap.REST_HEIGHT);

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSetpoint(elevatorSetpoints.get(setpoint));
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }

    @Override
    public void end(boolean interrupted){
        if (setpoint.equals("Rest")){
            elevator.resetCanRange();
        }
    }
}