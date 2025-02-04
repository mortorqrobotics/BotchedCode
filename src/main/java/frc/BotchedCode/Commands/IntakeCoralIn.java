package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeCoral;

public class IntakeCoralIn extends Command {

    private int count;
    private final IntakeCoral intakeCoral;

    public IntakeCoralIn(IntakeCoral intakeCoral) {
        this.intakeCoral = intakeCoral;
        addRequirements(intakeCoral);
    }

    @Override
    public void initialize(){
        count = 0;
    }

    @Override
    public void execute() {
        count ++;
        intakeCoral.in();
    }

    @Override
    public boolean isFinished(){
        return intakeCoral.pickedUp() && count < 20;
    }

    @Override
    public void end(boolean interrupted) {
        intakeCoral.end();
    }
}
