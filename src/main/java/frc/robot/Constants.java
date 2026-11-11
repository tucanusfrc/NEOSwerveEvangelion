package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    public class SwerveConstants{

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double SPEED_GEAR_RELATION = 6.25;
        public static final double TURN_GEAR_RELATION = 150.0 / 7.0;

        public static final double SPEED_POSITION_CONVERSION = (WHEEL_DIAMETER * Math.PI) / SPEED_GEAR_RELATION;
        public static final double SPEED_VELOCITY_CONVERSION = SPEED_POSITION_CONVERSION / 60.0;
        public static final double TURN_POSITION_CONVERSION = (2 * Math.PI) / TURN_GEAR_RELATION;

        public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 2.0;
        
        public static final byte FL_SPEED_ID = 1, FL_TURN_ID = 2, FL_CANCODER_ID = 1;
        public static final byte FR_SPEED_ID = 3, FR_TURN_ID = 4, FR_CANCODER_ID = 2;
        public static final byte BL_SPEED_ID = 5, BL_TURN_ID = 6, BL_CANCODER_ID = 3;
        public static final byte BR_SPEED_ID = 7, BR_TURN_ID = 8, BR_CANCODER_ID = 4;

        public static final byte PIGEON2_ID = 0;


        public static final double WHEEL_BASE = 0.6;
        public static final double TRACK_WIDTH = 0.6;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),   // Front Left
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),  // Front Right
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),  // Back Right
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0)  // Back Left
        );

    }
}
