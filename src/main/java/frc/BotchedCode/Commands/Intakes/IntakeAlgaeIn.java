package frc.BotchedCode.Commands.Intakes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeAlgae;

public class IntakeAlgaeIn extends Command {

    private int initialCount;
    private int pickupCount;
    private final IntakeAlgae intakeAlgae;

    public IntakeAlgaeIn(IntakeAlgae intakeAlgae) {
        this.intakeAlgae = intakeAlgae;
        addRequirements(intakeAlgae);
    }

    @Override
    public void initialize(){
        initialCount = 0;
        pickupCount = 0;
    }

    @Override
    public void execute() {
        initialCount++;
        if(intakeAlgae.pickedUp() && initialCount > 20){
            pickupCount++;
        }
        else if(pickupCount > 0){
            pickupCount--;
        }
        intakeAlgae.in();
    }

    @Override 
    public boolean isFinished(){
        return pickupCount > 20;
    }
    

    @Override
    public void end(boolean interrupted) {
        intakeAlgae.end();
    }
}