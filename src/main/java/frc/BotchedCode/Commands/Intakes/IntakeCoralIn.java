package frc.BotchedCode.Commands.Intakes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeCoral;

public class IntakeCoralIn extends Command {

    private int initialCount;
    private int pickupCount;
    private final IntakeCoral intakeCoral;

    public IntakeCoralIn(IntakeCoral intakeCoral) {
        this.intakeCoral = intakeCoral;
        addRequirements(intakeCoral);
    }

    @Override
    public void initialize(){
        initialCount = 0;
        pickupCount = 0;
    }

    @Override
    public void execute() {
        initialCount ++;
        if(intakeCoral.pickedUp() && initialCount > 20){
            pickupCount++;
        }
        else if(pickupCount > 0){
            pickupCount--;
        }
        intakeCoral.in();
    }

    @Override
    public boolean isFinished(){
        return pickupCount > 20;
    }

    @Override
    public void end(boolean interrupted) {
        intakeCoral.end();
    }
}
