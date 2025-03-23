package frc.BotchedCode.Commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.BotchedCode.Commands.Pathfinding.PathfindToIDReef;
import frc.BotchedCode.Commands.Pathfinding.PathfindToNearestStation;
import frc.BotchedCode.Subsystems.CommandSwerveDrivetrain;

public class ProcessorSide extends SequentialCommandGroup{
    public ProcessorSide(CommandSwerveDrivetrain drivetrain){
        addCommands(
            new PathfindToIDReef(drivetrain, false, 21),
            new PathFollowing("10 to 2"),
            new PathfindToNearestStation(drivetrain),
            new PathfindToIDReef(drivetrain, false, 18),
            new PathFollowing("7 to 2"),
            new PathfindToNearestStation(drivetrain),
            new PathfindToIDReef(drivetrain, true, 18)
        );
    }
}
