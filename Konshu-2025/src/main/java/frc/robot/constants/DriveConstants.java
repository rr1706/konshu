package frc.robot.constants;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.TunerConstants;

public final class DriveConstants {
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.0).in(RadiansPerSecond);
        public static final double DRIVE_DEADBAND = 0.006;
        public static final double ROTATION_DEADBAND = 0.006;

    public static final class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static final class AllianceConstants {
        public static final Rotation2d BLUE_ALLIANCE_ROTATION = Rotation2d.kZero;
        public static final Rotation2d RED_ALLIANCE_ROTATION = Rotation2d.k180deg;
    }
}