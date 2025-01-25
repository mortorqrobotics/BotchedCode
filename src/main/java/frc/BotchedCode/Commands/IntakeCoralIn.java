package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeCoral;

public class IntakeCoralIn extends Command {

    private final IntakeCoral intakeCoral;

    public IntakeCoralIn(IntakeCoral intakeCoral) {
        this.intakeCoral = intakeCoral;
        addRequirements(intakeCoral);
    }

    @Override
    public void execute() {
        intakeCoral.in();
    }

    @Override
    public void end(boolean interrupted) {
        intakeCoral.end();
    }
}
