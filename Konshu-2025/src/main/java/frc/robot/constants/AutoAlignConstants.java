package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class AutoAlignConstants {

    public static final class RedAllianceConstants {
        public static final Rotation2d BLUE_ALLIANCE_ROTATION = Rotation2d.kZero;
        public static final Translation2d kAL = new Translation2d(13.8385195,3.861569);//(13.810, 3.862);
        public static final Translation2d kAR = new Translation2d(13.8385195,4.190186);//(13.811, 4.19);
        public static final Translation2d kBL = new Translation2d(13.3064215,3.268563);//(13.278, 3.269);
        public static final Translation2d kBR = new Translation2d(13.5915835,3.431882);//(13.563, 3.432);
        public static final Translation2d kCL = new Translation2d(12.5268135,3.431882);//(12.499, 3.433);
        public static final Translation2d kCR = new Translation2d(12.8108335,3.268563);//(12.783, 3.268);
        public static final Translation2d kDL = new Translation2d(12.2781625,4.190186);//(12.279, 4.19);
        public static final Translation2d kDR = new Translation2d(12.2781625,3.861569);//(12.278, 3.862);
        public static final Translation2d kEL = new Translation2d(12.8108335,4.784181);//(12.811, 4.783);
        public static final Translation2d kER = new Translation2d(12.5268135,4.618883);//(12.526, 4.62);
        public static final Translation2d kFL = new Translation2d(13.5915835,4.618883);//(13.563, 4.619);
        public static final Translation2d kFR = new Translation2d(13.3064215,4.784181);//(13.279, 4.784);

        public static final Rotation2d kDAlgea = new Rotation2d(0);
        public static final Rotation2d kCAlgea = new Rotation2d(Math.PI / 3);
        public static final Rotation2d kBAlgea = new Rotation2d((2 * Math.PI) / 3);
        public static final Rotation2d kAAlgea = new Rotation2d(Math.PI);
        public static final Rotation2d kFAlgea = new Rotation2d((4 * Math.PI) / 3);
        public static final Rotation2d kEAlgea = new Rotation2d((5 * Math.PI) / 3);
    }

    public static final class BlueAllianceConstants {
        public static final Translation2d kAL = new Translation2d(3.7085865,4.190186);//(3.71, 4.19);
        public static final Translation2d kAR = new Translation2d(3.7085865,3.861569);//(3.709, 3.862);
        public static final Translation2d kBL = new Translation2d(4.2412575,4.784181);//(4.242, 4.783);
        public static final Translation2d kBR = new Translation2d(3.9572385,4.618883);//(3.957, 4.62);
        public static final Translation2d kCL = new Translation2d(5.0220075,4.618883);//(5.021, 4.619);
        public static final Translation2d kCR = new Translation2d(4.7368455,4.784181);//(4.737, 4.784);
        public static final Translation2d kDL = new Translation2d(5.2689445,3.861569);//(5.269, 3.862);
        public static final Translation2d kDR = new Translation2d(5.2689445,4.190186);//(5.27, 4.19);
        public static final Translation2d kEL = new Translation2d(4.7368455,3.268563);//(4.737, 3.269);
        public static final Translation2d kER = new Translation2d(5.0220075,3.431882);//(5.022, 3.432);
        public static final Translation2d kFL = new Translation2d(3.9572385,3.431882);//(3.957, 3.433);
        public static final Translation2d kFR = new Translation2d(4.2412575,3.268563);//(4.241, 3.268);

        public static final Rotation2d kDAlgea = new Rotation2d(0 + Math.PI);
        public static final Rotation2d kCAlgea = new Rotation2d(Math.PI / 3 + Math.PI);
        public static final Rotation2d kBAlgea = new Rotation2d((2 * Math.PI) / 3 + Math.PI);
        public static final Rotation2d kAAlgea = new Rotation2d(Math.PI + Math.PI);
        public static final Rotation2d kFAlgea = new Rotation2d((4 * Math.PI) / 3 + Math.PI);
        public static final Rotation2d kEAlgea = new Rotation2d((5 * Math.PI) / 3 + Math.PI);
    }

    public static InterpolatingDoubleTreeMap ElevatorAutoAlignL1 = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap ElevatorAutoAlignL2 = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap ElevatorAutoAlignL3 = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap ElevatorAutoAlignL4 = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap ArmAutoAlignL1 = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap ArmAutoAlignL2 = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap ArmAutoAlignL3 = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap ArmAutoAlignL4 = new InterpolatingDoubleTreeMap();


    // Interpolation lookup table to vary elevator height with target distance
    // Data is in METERS!!

    public static void SetUpElevatorAutoAlignConstantsL1() {
        ElevatorAutoAlignL1.put(0.5, 0.0);
        ElevatorAutoAlignL1.put(0.525, 0.0); // Against the reef
        ElevatorAutoAlignL1.put(0.65, 0.0);
        ElevatorAutoAlignL1.put(0.766, 0.0);
        ElevatorAutoAlignL1.put(0.882, 0.0);
        ElevatorAutoAlignL1.put(1.0, 0.0);
    }

    public static void SetUpElevatorAutoAlignConstantsL2() {
        ElevatorAutoAlignL2.put(0.5, 0.0);
        ElevatorAutoAlignL2.put(0.525, 0.0);
        ElevatorAutoAlignL2.put(0.65, 3.0);
        ElevatorAutoAlignL2.put(0.766, 6.0);
        ElevatorAutoAlignL2.put(0.882, 9.0);
        ElevatorAutoAlignL2.put(1.0, 9.0);
    }

    public static void SetUpElevatorAutoAlignConstantsL3() {
        ElevatorAutoAlignL3.put(0.5, 0.0);
        ElevatorAutoAlignL3.put(0.525, 0.0);
        ElevatorAutoAlignL3.put(0.65, 3.0);
        ElevatorAutoAlignL3.put(0.766, 6.0);
        ElevatorAutoAlignL3.put(0.882, 9.0);
        ElevatorAutoAlignL3.put(1.0, 9.0);
    }

    public static void SetUpElevatorAutoAlignConstantsL4() {
        ElevatorAutoAlignL4.put(0.5, 0.0);
        ElevatorAutoAlignL4.put(0.525, 0.0);
        ElevatorAutoAlignL4.put(0.65, 0.5);
        ElevatorAutoAlignL4.put(0.766, 1.0);
        ElevatorAutoAlignL4.put(0.882, 1.5);
        ElevatorAutoAlignL4.put(1.0, 1.5);
    }

    // Interpolation lookup table to vary arm angle with target distance
    // Data is in degrees
    public static void SetUpArmAutoAlignConstantsL1() {
        ArmAutoAlignL1.put(0.5, 0.0);
        ArmAutoAlignL1.put(0.525, 0.0); // Against the reef
        ArmAutoAlignL1.put(0.65, 0.0);
        ArmAutoAlignL1.put(0.766, 0.0);
        ArmAutoAlignL1.put(0.882, 0.0);
        ArmAutoAlignL1.put(1.0, 0.0);
    }

    public static void SetUpArmAutoAlignConstantsL2() {
        ArmAutoAlignL2.put(0.5, 0.0);
        ArmAutoAlignL2.put(0.525, 0.0);
        ArmAutoAlignL2.put(0.65, 0.0);
        ArmAutoAlignL2.put(0.766, 0.0);
        ArmAutoAlignL2.put(0.882, 0.0);
        ArmAutoAlignL2.put(1.0, 0.0);
    }

    public static void SetUpArmAutoAlignConstantsL3() {
        ArmAutoAlignL3.put(0.5, 0.0);
        ArmAutoAlignL3.put(0.525, 0.0);
        ArmAutoAlignL3.put(0.65, 0.0);
        ArmAutoAlignL3.put(0.766, 0.0);
        ArmAutoAlignL3.put(0.882, 0.0);
        ArmAutoAlignL3.put(1.0, 0.0);
    }

    public static void SetUpArmAutoAlignConstantsL4() {
        ArmAutoAlignL4.put(0.5, 0.0);
        ArmAutoAlignL4.put(0.525, 0.0);
        ArmAutoAlignL4.put(0.65, -4.0);
        ArmAutoAlignL4.put(0.766, -8.6);
        ArmAutoAlignL4.put(0.882, -8.6);
        ArmAutoAlignL4.put(1.0, -8.6);
    }

public static final class ReefAprilTags {
    // Translation values are in meters (x, y) as directly provided from the JSON.
    public static final Translation2d At1  = new Translation2d(16.697198, 0.65532);
    public static final Translation2d At2  = new Translation2d(16.697198, 7.39648);
    public static final Translation2d At3  = new Translation2d(11.56081, 8.05561);
    public static final Translation2d At4  = new Translation2d(9.27608, 6.137656);
    public static final Translation2d At5  = new Translation2d(9.27608, 1.914906);
    public static final Translation2d At6  = new Translation2d(13.474446, 3.306318);
    public static final Translation2d At7  = new Translation2d(13.890498, 4.0259);
    public static final Translation2d At8  = new Translation2d(13.474446, 4.745482);
    public static final Translation2d At9  = new Translation2d(12.643358, 4.745482);
    public static final Translation2d At10 = new Translation2d(12.227306, 4.0259);
    public static final Translation2d At11 = new Translation2d(12.643358, 3.306318);
    public static final Translation2d At12 = new Translation2d(0.851154, 0.65532);
    public static final Translation2d At13 = new Translation2d(0.851154, 7.39648);
    public static final Translation2d At14 = new Translation2d(8.272272, 6.137656);
    public static final Translation2d At15 = new Translation2d(8.272272, 1.914906);
    public static final Translation2d At16 = new Translation2d(5.987542, -0.00381);
    public static final Translation2d At17 = new Translation2d(4.073906, 3.306318);
    public static final Translation2d At18 = new Translation2d(3.6576, 4.0259);
    public static final Translation2d At19 = new Translation2d(4.073906, 4.745482);
    public static final Translation2d At20 = new Translation2d(4.90474, 4.745482);
    public static final Translation2d At21 = new Translation2d(5.321046, 4.0259);
    public static final Translation2d At22 = new Translation2d(4.90474, 3.306318);
}

    }
}
