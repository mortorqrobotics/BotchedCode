package frc.BotchedCode.Commands.ElevatorPivot.AutoLifts;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.BotchedCode.Subsystems.Elevator;
import frc.BotchedCode.Subsystems.Pivot;

public class TeleopElevatorPivot extends SequentialCommandGroup {

    public TeleopElevatorPivot(Elevator elevator, Pivot pivot, String setpoint){
        addCommands(

            
            Commands.parallel(new AutoElevator(elevator, setpoint), new AutoPivot(pivot, setpoint))

        );
    }
    
}
