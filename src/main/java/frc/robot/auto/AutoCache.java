package frc.robot.auto;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoCache {
    private static Map<String, AutoItem> paths = new HashMap<>();
    private static SendableChooser<AutoItem> autoChooser = new SendableChooser<>();
    private static AutoCache cache = null;

    public static void init() {
        if(AutoCache.cache == null) AutoCache.cache = new AutoCache();

        AutoConfigurer.configure();
        AutoConfigurer.registerNamedCommands();
    }

    public static AutoItem getPath(String pathName) {
        if(!AutoCache.paths.containsKey(pathName))
            AutoCache.paths.put(pathName, new AutoPath(pathName));
        return AutoCache.paths.get(pathName);
    }

    public static AutoItem getAuto(String autoName) {
        if(!AutoCache.paths.containsKey(autoName))
            AutoCache.paths.put(autoName, new AutoFull(autoName));
        return AutoCache.paths.get(autoName);
    }

    public static AutoItem getSelectedAuto() {
        return AutoCache.autoChooser.getSelected();
    }

    public static void afterLoad() {
        // Disable real swerve
        AutoConfigurer.allowDrive = false;

        for(AutoItem item : AutoCache.paths.values()) {
            System.out.printf("[AutoCache] Running %s\n", item.getName());

            // Initialize, execute, then cancel the command
            Command itemCommand = item.getCommand();
            itemCommand.initialize();
            itemCommand.execute();
            itemCommand.cancel();

            // Add the auto to the autoChooser
            AutoCache.autoChooser.addOption(item.getName(), item);
        }

        // Enable real swerve
        AutoConfigurer.allowDrive = true;

        // Publish the autoChooser to SmartDashboard
        SmartDashboard.putData("[AUTO] Chooser", AutoCache.autoChooser);
    }
}