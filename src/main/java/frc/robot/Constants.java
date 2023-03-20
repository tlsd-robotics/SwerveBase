package frc.robot;

import edu.wpi.first.math.util.Units;

// Adjust all values so that they match the robot

public class Constants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(21.73);
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.73);

    public static final class FrontLeft {
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 27;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 28;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(238.97);  
    }

    public static final class FrontRight {
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 21;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 22;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 13;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(140.62);
    }

    public static final class BackLeft {
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 25;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 29;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(325.63); 
    }
    
    public static final class BackRight {
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 23;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 24;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(266.48);
    }
   
    public static final int DRIVE_CONTROLLER_ID = 0;
    public static final int PNEUMATICS_HUB_ID = 3;

}
