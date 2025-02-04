package frc.BotchedCode.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeAlgae;

public class IntakeAlgaeIn extends Command {

    private int count;
    private final IntakeAlgae intakeAlgae;

    public IntakeAlgaeIn(IntakeAlgae intakeAlgae) {
        this.intakeAlgae = intakeAlgae;
        addRequirements(intakeAlgae);
    }

    @Override
    public void initialize(){
        count = 0;
    }

    @Override
    public void execute() {
        count++;
        intakeAlgae.in();
    }

    @Override 
    public boolean isFinished(){
        return intakeAlgae.pickedUp() && count > 20;
    }
    

    @Override
    public void end(boolean interrupted) {
        intakeAlgae.end();
    }
}