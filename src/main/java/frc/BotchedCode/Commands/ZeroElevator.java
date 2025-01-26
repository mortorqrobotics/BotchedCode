package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.Elevator;

public class ZeroElevator extends Command {

    private final Elevator elevator;

    public ZeroElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.manualDown();
    }

    @Override
    public boolean isFinished(){
        return elevator.reachedLowerLimit();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.end();
        elevator.zeroEncoder();
    }
}
