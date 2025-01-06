package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double stickDeadband = 0.1;
    
    public static final class IntakeSubsystemInfo {
        public static final int FRONT_MOTOR_ID = 1; // PWN changde from 3 for testing 
        public static final int WHEEL_MOTOR_ID = 15; // CANBUS 
        public static final int FLOOR_MOTOR_ID = 14; // CANBUS
    }

    public static final class ShooterSubsystemInfo {
        // arm aiming motors 
        public static final int RIGHT_ARM_MOTOR_ID = 18; // CANBUS 
        public static final int LEFT_ARM_MOTOR_ID = 19; // CANBUS 
        public static final int FEEDER_ENCODER = 1; // DIO 

        // feeding motors 
        public static final int LEFT_FEEDING_MOTOR_ID = 5;
        public static final int RIGHT_FEEDING_MOTOR_ID = 4; 

         // shooting motors
        public static final int TOP_SHOOTING_MOTOR_ID = 20; 
        public static final int BOTTOM_SHOOTING_MOTOR_ID = 21;
    }

    public static final class AmpSubsystemInfo {
        //arm motors 
        public static final int LEFT_ARM_MOTOR_ID = 16; 
        public static final int RIGHT_ARM_MOTOR_ID = 17; 
        // thing that put the note in 
        public static final int AMP_SCORER_MOTOR_ID = 0; // TODO: check if this is right 
        public static final int AMP_ENCODER = 0;
    }
}
