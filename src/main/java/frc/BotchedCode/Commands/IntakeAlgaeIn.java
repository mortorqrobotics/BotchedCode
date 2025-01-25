package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeAlgae;

public class IntakeAlgaeIn extends Command {

    private final IntakeAlgae intakeAlgae;

    public IntakeAlgaeIn(IntakeAlgae intakeAlgae) {
        this.intakeAlgae = intakeAlgae;
        addRequirements(intakeAlgae);
    }

    @Override
    public void execute() {
        intakeAlgae.in();
    }

    @Override
    public void end(boolean interrupted) {
        intakeAlgae.end();
    }
}