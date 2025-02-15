package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.Pivot;

public class ManualPivotDown extends Command {

    private final Pivot pivot;

    public ManualPivotDown(Pivot pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.down();
    }

    @Override
    public void end(boolean interrupted) {
        pivot.end();
    }
}

