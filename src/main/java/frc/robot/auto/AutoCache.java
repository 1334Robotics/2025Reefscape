package frc.robot.auto;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AutoCache {
    private static Map<String, AutoItem> paths = null;
    private static AutoCache cache = null;

    public AutoCache() {
        if(AutoCache.paths == null) AutoCache.paths = new HashMap<>();
    }

    public static void init() {
        if(AutoCache.cache == null) AutoCache.cache = new AutoCache();

        // Turn off real swerve
        AutoConfigurer.configure();
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
        }

        // Enable real swerve
        AutoConfigurer.allowDrive = true;
    }
}