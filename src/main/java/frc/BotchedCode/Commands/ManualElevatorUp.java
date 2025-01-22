package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.Elevator;

public class ManualElevatorUp extends Command {

    private final Elevator elevator;

    public ManualElevatorUp(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.up();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.end();
    }
}