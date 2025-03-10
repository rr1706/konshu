package frc.robot.constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class AutoAlignConstants {

    public static final class RedAllianceConstants {
        public static final Rotation2d BLUE_ALLIANCE_ROTATION = Rotation2d.kZero;
        public static final Translation2d kCL = new Translation2d(12.499, 3.433);
        public static final Translation2d kCR = new Translation2d(12.783, 3.268);
        public static final Translation2d kBL = new Translation2d(13.278, 3.269);
        public static final Translation2d kBR = new Translation2d(13.563, 3.432);
        public static final Translation2d kAL = new Translation2d(13.810, 3.862);
        public static final Translation2d kAR = new Translation2d(13.811, 4.19);
        public static final Translation2d kFL = new Translation2d(13.563, 4.619);
        public static final Translation2d kFR = new Translation2d(13.279, 4.784);
        public static final Translation2d kEL = new Translation2d(12.811, 4.783);
        public static final Translation2d kER = new Translation2d(12.526, 4.62);
        public static final Translation2d kDL = new Translation2d(12.279, 4.19);
        public static final Translation2d kDR = new Translation2d(12.278, 3.862);
        public static final Rotation2d kDAlgea = new Rotation2d(0);
        public static final Rotation2d kCAlgea = new Rotation2d(Math.PI/3);
        public static final Rotation2d kBAlgea = new Rotation2d((2*Math.PI)/3);
        public static final Rotation2d kAAlgea = new Rotation2d(Math.PI);
        public static final Rotation2d kFAlgea = new Rotation2d((4*Math.PI)/3);
        public static final Rotation2d kEAlgea = new Rotation2d((5*Math.PI)/3);
    }

    public static final class BlueAllianceConstants {
        public static final Translation2d kCL = new Translation2d(5.021, 4.619);
        public static final Translation2d kCR = new Translation2d(4.737, 4.784);
        public static final Translation2d kBL = new Translation2d(4.242, 4.783);
        public static final Translation2d kBR = new Translation2d(3.957, 4.62);
        public static final Translation2d kAL = new Translation2d(3.71, 4.19);
        public static final Translation2d kAR = new Translation2d(3.709, 3.862);
        public static final Translation2d kFL = new Translation2d(3.957, 3.433);
        public static final Translation2d kFR = new Translation2d(4.241, 3.268);
        public static final Translation2d kEL = new Translation2d(4.737, 3.269);
        public static final Translation2d kER = new Translation2d(5.022, 3.432);
        public static final Translation2d kDL = new Translation2d(5.269, 3.862);
        public static final Translation2d kDR = new Translation2d(5.27, 4.19);
        public static final Rotation2d kDAlgea = new Rotation2d(0);
        public static final Rotation2d kCAlgea = new Rotation2d(Math.PI/3);
        public static final Rotation2d kBAlgea = new Rotation2d((2*Math.PI)/3);
        public static final Rotation2d kAAlgea = new Rotation2d(Math.PI);
        public static final Rotation2d kFAlgea = new Rotation2d((4*Math.PI)/3);
        public static final Rotation2d kEAlgea = new Rotation2d((5*Math.PI)/3);
    }

      public static InterpolatingDoubleTreeMap ElevatorAutoAlign = new InterpolatingDoubleTreeMap();
//interpolation lookup table to vary elevator height with target distance

        public static void SetUpAutoAlignConstants(){
            ElevatorAutoAlign.put(0.0,40.0);
            ElevatorAutoAlign.put(2.0,20.0);
            ElevatorAutoAlign.put(4.0,2.0);
            ElevatorAutoAlign.put(6.0,40.0);
            ElevatorAutoAlign.put(8.0,20.0);
            ElevatorAutoAlign.put(10.0,2.0);
            ElevatorAutoAlign.put(12.0,40.0);
            ElevatorAutoAlign.put(14.0,20.0);
            ElevatorAutoAlign.put(16.0,2.0);
            ElevatorAutoAlign.put(18.0,40.0);
            ElevatorAutoAlign.put(20.0,20.0);
        }

}
