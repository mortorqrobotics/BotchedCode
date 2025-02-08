package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeAlgae;

public class IntakeAlgaeOut extends Command {

    private final IntakeAlgae intakeAlgae;
    private int count;

    public IntakeAlgaeOut(IntakeAlgae intakeAlgae) {
        this.intakeAlgae = intakeAlgae;
        addRequirements(intakeAlgae);
    }

    @Override
    public void initialize(){
        count = 0;
    }

    @Override
    public void execute() {
        if(!intakeAlgae.pickedUp()){
            count++;
        }
        intakeAlgae.out();
    }

    @Override
    public boolean isFinished(){
        return count > 20;
    }

    @Override
    public void end(boolean interrupted) {
        intakeAlgae.end();
    }
}