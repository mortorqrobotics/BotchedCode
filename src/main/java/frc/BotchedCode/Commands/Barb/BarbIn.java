package frc.BotchedCode.Commands.Barb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.Barb;

public class BarbIn extends Command {

    private final Barb barb;

    public BarbIn(Barb barb) {
        this.barb = barb;
        addRequirements(barb);
    }

    @Override
    public void execute() {
        barb.in();
    }

    @Override
    public void end(boolean interrupted) {
        barb.end();
    }
}