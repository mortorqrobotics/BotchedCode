package frc.BotchedCode.Commands.Intakes;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.BotchedCode.Subsystems.IntakeCoral;

public class IntakeCoralOut extends Command {

    private IntakeCoral intakeCoral;
    private int count;
    private CANdle candle;

    public IntakeCoralOut(IntakeCoral intakeCoral, CANdle candle) {
        this.intakeCoral = intakeCoral;
        this.candle = candle;
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
        candle.setLEDs(0,0,0,0,0,10);
    }
}
