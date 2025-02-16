package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.Pivot;

public class ZeroPivot extends Command {

    private final Pivot pivot;

    public ZeroPivot(Pivot pivot) {
        this.pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void execute() {
        pivot.down();
    }

    @Override
    public boolean isFinished(){
        System.out.print(pivot.currentSpike());
        return pivot.currentSpike();
    }

    @Override
    public void end(boolean interrupted) {
        pivot.end();
        pivot.zeroEncoder();
    }
}
