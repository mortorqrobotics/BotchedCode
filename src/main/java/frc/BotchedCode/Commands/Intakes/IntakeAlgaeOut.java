package frc.BotchedCode.Commands.Intakes;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeAlgae;

public class IntakeAlgaeOut extends Command {

    private IntakeAlgae intakeAlgae;
    private int count;
    private CANdle candle;

    public IntakeAlgaeOut(IntakeAlgae intakeAlgae, CANdle candle) {
        this.intakeAlgae = intakeAlgae;
        this.candle = candle;
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
        candle.setLEDs(0,0,0,0,0,10);
    }
}