package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeCoral;

public class IntakeCoralOut extends Command {

    private final IntakeCoral intakeCoral;
    private int count;

    public IntakeCoralOut(IntakeCoral intakeCoral) {
        this.intakeCoral = intakeCoral;
        addRequirements(intakeCoral);
    }

    @Override
    public void initialize(){
        count = 0;
    }

    @Override
    public void execute() {
        if (!intakeCoral.pickedUp()) {
            count++;
        }
        intakeCoral.out();
    }

    @Override
    public boolean isFinished(){
        return count > 20;
    }

    @Override
    public void end(boolean interrupted) {
        intakeCoral.end();
    }
}
