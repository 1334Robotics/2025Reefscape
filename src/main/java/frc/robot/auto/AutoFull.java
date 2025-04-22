package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoFull extends AutoItem {
    private Command autoCommand;
    private String name;

    public AutoFull(String autoName) {
        this.name = autoName;
        try {
            this.autoCommand = new PathPlannerAuto(autoName);
        } catch(Exception e) {
            this.autoCommand = null;
            System.err.println("Error in construction of a full auto: " + e.toString());
        }
    }

    @Override
    public Command getCommand() {
        if(this.autoCommand == null) {
            System.err.println("Error: Null auto used");
            return Commands.none();
        }

        return this.autoCommand;
    }

    @Override
    public String getName() {
        return this.name;
    }
}
