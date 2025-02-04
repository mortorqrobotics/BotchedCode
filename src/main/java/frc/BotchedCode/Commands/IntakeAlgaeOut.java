package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeAlgae;

public class IntakeAlgaeOut extends Command {

    private final IntakeAlgae intakeAlgae;

    public IntakeAlgaeOut(IntakeAlgae intakeAlgae) {
        this.intakeAlgae = intakeAlgae;
        addRequirements(intakeAlgae);
    }

    @Override
    public void execute() {
        intakeAlgae.out();
    }

    @Override
    public boolean isFinished(){
        return intakeAlgae.released();
    }

    @Override
    public void end(boolean interrupted) {
        intakeAlgae.end();
    }
}