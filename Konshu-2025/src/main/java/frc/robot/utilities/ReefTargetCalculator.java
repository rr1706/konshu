package frc.robot.utilities;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.constants.ButtonConstants;
import frc.robot.Robot;

public class ReefTargetCalculator {
    public enum AlignMode {LEFT, RIGHT, ALGAE};
    public static BooleanSupplier right = () -> false; 
    public static BooleanSupplier left = () -> false; 
        
            /**
             * Calculates the target translation based on the current alliance,
             * the state of reef (Coral) buttons, and the specified alignment mode.
             * Note that for LEFT and RIGHT only the Translation2d is currently used, but the Rotation2d
             * is also returned for future use by the dithering code (adjust the elevation/arm rotation slightly)
         
             * @return the selected target Pose2d, or null if no button is pressed.
             */
            public static Pose2d calculateTargetTranslation(AlignMode alignMode) {
                // Normalize alignment mode to uppercase.
                // Get current alliance; default to Blue if not present.
                DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        
                // Blue Alliance button polling.
                if (alliance == DriverStation.Alliance.Blue) {
                    if (DriverStation.getStickButton(2, ButtonConstants.kCoralA)) {
                        switch (alignMode) {
                            case LEFT: 
                            left = () -> true; 
                            right = () -> false; 
                            return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kAL, AutoAlignConstants.BlueAllianceConstants.kAAlgea);
                            case RIGHT: 
                            left = () -> false; 
                            right = () -> true; 
                            return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kAR, AutoAlignConstants.BlueAllianceConstants.kAAlgea); 
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kAAlgea);
                }
            } else if (DriverStation.getStickButton(2, ButtonConstants.kCoralB)) {
                switch (alignMode) {
                    case LEFT:  
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kBL, AutoAlignConstants.BlueAllianceConstants.kBAlgea);
                    case RIGHT: 
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kBR, AutoAlignConstants.BlueAllianceConstants.kBAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kBAlgea);  
                }
            } else if (DriverStation.getStickButton(2, ButtonConstants.kCoralC)) {
                switch (alignMode) {
                    case LEFT: 
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kCL, AutoAlignConstants.BlueAllianceConstants.kCAlgea);
                    case RIGHT: 
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kCR, AutoAlignConstants.BlueAllianceConstants.kCAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kCAlgea);
                }
            } else if (DriverStation.getStickButton(2, ButtonConstants.kCoralD)) {
                switch (alignMode) {
                    case LEFT: 
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kDL, AutoAlignConstants.BlueAllianceConstants.kDAlgea);
                    case RIGHT:
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kDR, AutoAlignConstants.BlueAllianceConstants.kDAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kDAlgea);
                }
            } else if (DriverStation.getStickButton(2, ButtonConstants.kCoralE)) {
                switch (alignMode) {
                    case LEFT: 
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kEL, AutoAlignConstants.BlueAllianceConstants.kEAlgea);
                    case RIGHT: 
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kER, AutoAlignConstants.BlueAllianceConstants.kEAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kEAlgea);
                }
            } else if (DriverStation.getStickButton(2, ButtonConstants.kCoralF)) {
                switch (alignMode) {
                    case LEFT: 
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kFL, AutoAlignConstants.BlueAllianceConstants.kFAlgea);
                    case RIGHT: 
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kFR, AutoAlignConstants.BlueAllianceConstants.kFAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kFAlgea);
                }
            }
        } 
        // Red Alliance button polling.
        else  {
            if (DriverStation.getStickButton(2, ButtonConstants.kCoralA)) {
                switch (alignMode) {
                    case LEFT: 
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kAL, AutoAlignConstants.RedAllianceConstants.kAAlgea);
                    case RIGHT:
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kAR, AutoAlignConstants.RedAllianceConstants.kAAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kAAlgea);
                }
            } else if (DriverStation.getStickButton(2, ButtonConstants.kCoralB)) {
                switch (alignMode) {
                    case LEFT:
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kBL, AutoAlignConstants.RedAllianceConstants.kBAlgea);
                    case RIGHT:
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kBR, AutoAlignConstants.RedAllianceConstants.kBAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kBAlgea);  
                }
            } else if (DriverStation.getStickButton(2, ButtonConstants.kCoralC)) {
                switch (alignMode) {
                    case LEFT:
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kCL, AutoAlignConstants.RedAllianceConstants.kCAlgea);
                    case RIGHT: 
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kCR,AutoAlignConstants.RedAllianceConstants.kCAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kCAlgea);
                }
            } else if (DriverStation.getStickButton(2, ButtonConstants.kCoralD)) {
                switch (alignMode) {
                    case LEFT: 
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kDL, AutoAlignConstants.RedAllianceConstants.kDAlgea);
                    case RIGHT: 
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kDR, AutoAlignConstants.RedAllianceConstants.kDAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kDAlgea);
                }
            } else if (DriverStation.getStickButton(2, ButtonConstants.kCoralE)) {
                switch (alignMode) {
                    case LEFT: 
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kEL, AutoAlignConstants.RedAllianceConstants.kEAlgea);
                    case RIGHT:
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kER, AutoAlignConstants.RedAllianceConstants.kEAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kEAlgea);
                }
            } else if (DriverStation.getStickButton(2, ButtonConstants.kCoralF)) {
                switch (alignMode) {
                    case LEFT: 
                    left = () -> true; 
                    right = () -> false; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kFL, AutoAlignConstants.RedAllianceConstants.kFAlgea);
                    case RIGHT:
                    left = () -> false; 
                    right = () -> true; 
                    return new Pose2d(AutoAlignConstants.RedAllianceConstants.kFR, AutoAlignConstants.RedAllianceConstants.kFAlgea);
                    case ALGAE: return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kFAlgea);
                }
            }
        }
        // No button pressed returns null.
        return null;
    }
}

