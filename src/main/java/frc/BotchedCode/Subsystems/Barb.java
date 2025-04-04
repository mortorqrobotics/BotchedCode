package frc.BotchedCode.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.BotchedCode.Constants.RobotMap;

public class Barb extends SubsystemBase {
    
    private final TalonFX barb;

    
    public Barb(){
        barb = new TalonFX(RobotMap.BARB_ID, "1515Canivore");
    }

    public void in(){
        if (barb.getPosition().getValueAsDouble() > -204){
            barb.set(-RobotMap.BARB_SPEED_FAST);
        }
        else{
            barb.set(0);
        }
    }

    public void out(){
        barb.set(RobotMap.BARB_SPEED);
        
    }

    public void end(){
        barb.set(0);
        
    }

    public void ignoreLimitIn(){
        barb.set(-RobotMap.BARB_SPEED);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Barb encoder", barb.getPosition().getValueAsDouble());
    }

   
}
