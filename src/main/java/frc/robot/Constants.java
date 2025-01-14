// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DriveConstants {
        public static final double MAX_SPEED = Units.feetToMeters(14.5); // Kraken X60 free speed
        public static final double MAX_ANGULAR_SPEED = Math.PI * 2.0; // 1 rotation per second
        
        // The directory name for the deploy directory containing YAGSL JSON files
        public static final String SWERVE_CONFIG_DIR = "swerve";
        
        public static final double DEADBAND = 0.1;
    }
    
    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }
}
