package frc.BotchedCode.Subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase{
    //private CANdle candle;
    private BooleanSupplier coralLedsOn;
    private BooleanSupplier algaeLedsOn;

    public Candle(BooleanSupplier coralLedsOn, BooleanSupplier algaeLedsOn){
        //candle = new CANdle(RobotMap.CANDLE_ID);

        this.coralLedsOn = coralLedsOn;
        this.algaeLedsOn = algaeLedsOn;

        // CANdleConfiguration config = new CANdleConfiguration();
        // config.statusLedOffWhenActive = true;
        // config.stripType = LEDStripType.GRB;
        // config.v5Enabled = true;
        // config.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
        // config.brightnessScalar = 1;
        // candle.configAllSettings(config, 100);
        // candle.configLEDType(LEDStripType.GRB); //just added this after cd post
        // candle.setLEDs(0, 0, 0);
    }

    public void coralOn(){
        // if (coralLedsOn.getAsBoolean()){
        //     candle.setLEDs(255, 0, 0, 100, 0, 4);
        // }
        return;
    }

    public void coralOff(){
        // if (!coralLedsOn.getAsBoolean()){
        //     candle.setLEDs(0, 0, 0, 0, 0, 4);
        // }
        return;
    }

    public void algaeOn(){
        // if (algaeLedsOn.getAsBoolean()){
        //     candle.setLEDs(0, 0, 255, 100, 4, 4);
        // }
        return;
    }

    public void algaeOff(){
        // if (!algaeLedsOn.getAsBoolean()){
        //     candle.setLEDs(0, 0, 0, 0,4, 4);
        // }
        return;
    }

    public void setOnBoardLeds(int r, int b, int g) {
        //candle.setLEDs(r, g, b, 100, 0, 15);
        return;
    }
}
