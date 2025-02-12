package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.Elevator;

public class ElevatorManualDown extends Command {

    private final Elevator elevator;

    public ElevatorManualDown(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.manualUp();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.end();
    }
}