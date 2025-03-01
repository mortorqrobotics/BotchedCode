package frc.BotchedCode.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;

public class Barb extends SubsystemBase {
    
    private final TalonFX barb;

    public Barb(){
        barb = new TalonFX(RobotMap.BARB_ID, "1515Canivore");
    }

    public void in(){
        barb.set(-RobotMap.BARB_SPEED);
        
    }

    public void out(){
        barb.set(RobotMap.BARB_SPEED);
        
    }

    public void end(){
        barb.set(0);
        
    }
}
