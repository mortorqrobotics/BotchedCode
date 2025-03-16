package frc.BotchedCode.Commands.Barb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.Barb;

public class BarbInIgnoreLimit extends Command {

    private final Barb barb;

    public BarbInIgnoreLimit(Barb barb) {
        this.barb = barb;
        addRequirements(barb);
    }

    @Override
    public void execute() {
        barb.ignoreLimitIn();
    }

    @Override
    public void end(boolean interrupted) {
        barb.end();
    }
}