package frc.BotchedCode.Commands.ElevatorPivot.AutoLifts;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.BotchedCode.Commands.Intakes.IntakeCoralOut;
import frc.BotchedCode.Subsystems.Elevator;
import frc.BotchedCode.Subsystems.IntakeCoral;
import frc.BotchedCode.Subsystems.Pivot;

public class AutoScoreRoutine extends SequentialCommandGroup {

    public AutoScoreRoutine(Elevator elevator, Pivot pivot, IntakeCoral intakeCoral, String setpoint){
        addCommands(

            new AutoElevatorPivot(elevator,pivot,setpoint),
            new IntakeCoralOut(intakeCoral),
            new AutoElevatorPivot(elevator,pivot,"Rest")

        );
    }
    
}
