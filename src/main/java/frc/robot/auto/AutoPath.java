package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoPath extends AutoItem {
    private Command pathCommand;
    private String name;
    
    public AutoPath(String name) {
        this.name = name;
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(name);
            this.pathCommand = AutoBuilder.followPath(path);
        } catch(Exception e) {
            this.pathCommand = null;
            System.err.println("Error in construction of a path: " + e.toString());
        }
    }

    @Override
    public Command getCommand() {
        if(this.pathCommand == null) {
            System.err.println("Error: Null path used");
            return Commands.none();
        }

        return this.pathCommand;
    }

    @Override
    public String getName() {
        return this.name;
    }
}
