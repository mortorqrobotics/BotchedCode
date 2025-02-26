package frc.BotchedCode.Commands.Intakes;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeAlgae;

public class IntakeAlgaeIn extends Command {

    private int initialCount;
    private int pickupCount;
    private IntakeAlgae intakeAlgae;
    private CANdle candle;

    public IntakeAlgaeIn(IntakeAlgae intakeAlgae, CANdle candle) {
        this.intakeAlgae = intakeAlgae;
        this.candle = candle;
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
        candle.setLEDs(0, 0, 255, 100, 0, 10);
    }
}