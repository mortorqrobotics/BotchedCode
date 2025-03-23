package frc.BotchedCode.Commands.ElevatorPivot.AutoLifts;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Constants.RobotMap;
import frc.BotchedCode.Subsystems.Pivot;

public class AutoPivot extends Command {

    private final Pivot pivot;
    private final String setpoint;
    private HashMap<String, Double> pivotSetpoints;

    public AutoPivot(Pivot pivot, String setpoint) {
        this.pivot = pivot;
        this.setpoint = setpoint;

        this.pivotSetpoints = new HashMap<>();
        this.pivotSetpoints.put("Rest", RobotMap.REST_ANGLE);
        this.pivotSetpoints.put("L2", RobotMap.L23_ANGLE);
        this.pivotSetpoints.put("L3", RobotMap.L23_ANGLE);
        this.pivotSetpoints.put("L4", RobotMap.L4_ANGLE);
        this.pivotSetpoints.put("Processor", RobotMap.PROCESSOR_ANGLE);

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setSetpoint(pivotSetpoints.get(setpoint));
    }

    @Override
    public boolean isFinished() {
        return pivot.atSetpoint();
    }
}