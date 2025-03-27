package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoItem {
    public AutoItem() {}

    public Command getCommand() {
        System.err.printf("getCommand unimplemented for %s\n", this.toString());
        return Commands.none();
    }

    public String getName() {
        return String.format("getName unimplemented for %s", this.toString());
    }
}
