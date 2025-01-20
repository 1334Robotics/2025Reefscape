package frc.robot.constants;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.XboxMappings;

public class RobotContainerConstants {
    public static final int DRIVER_CONTROLLER_PORT      = 0;
    public static final int OPERATOR_CONTROLLER_PORT    = 1;

    public static final int ELEVATOR_PRIMARY_MOTOR_ID   = 1;
    public static final int ELEVATOR_SECONDARY_MOTOR_ID = 2;

    public static final int MAILBOX_OUTPUT_BUTTON = XboxMappings.Button.A;
    public static final int MAILBOX_INPUT_BUTTON  = XboxMappings.Button.B;
    public static final int MAILBOX_STOP_BUTTON   = XboxMappings.Button.X;
    public static final int SOLENOID_EXTEND_BUTTON = XboxMappings.Button.RightBumper;
    public static final int SOLENOID_RETRACT_BUTTON = XboxMappings.Button.LeftBumper;
}
