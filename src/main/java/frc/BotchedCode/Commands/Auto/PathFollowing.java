package frc.BotchedCode.Commands.Auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PathFollowing extends Command{

    public String pathName;
    public Command commandToRun;

    public PathFollowing(String pathName){
        this.pathName = pathName;
        try {
            this.commandToRun = AutoBuilder.followPath((PathPlannerPath.fromPathFile(pathName)));
        } 
        catch (IOException | ParseException | FileVersionException ex) {
            this.commandToRun = Commands.none();
        }
    }

    @Override
    public void initialize(){
        commandToRun.schedule();
    }

    @Override
    public boolean isFinished(){
        return this.commandToRun.isFinished();
    }
}
