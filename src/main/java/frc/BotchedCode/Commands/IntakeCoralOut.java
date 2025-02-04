package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeCoral;

public class IntakeCoralOut extends Command {

    private final IntakeCoral intakeCoral;

    public IntakeCoralOut(IntakeCoral intakeCoral) {
        this.intakeCoral = intakeCoral;
        addRequirements(intakeCoral);
    }

    @Override
    public void execute() {
        intakeCoral.out();
    }

    @Override
    public boolean isFinished(){
        return intakeCoral.released();
    }

    @Override
    public void end(boolean interrupted) {
        intakeCoral.end();
    }
}
