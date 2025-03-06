package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.constants.ButtonConstants;

public class ReefTargetCalculator {
    public enum AlignMode {LEFT, RIGHT, ALGAE};
    /**
     * Calculates the target translation based on the current alliance,
     * the state of reef (Coral) buttons, and the specified alignment mode.
 
     * @return the selected target Pose2d, or null if no button is pressed.
     */
    public static Pose2d calculateTargetTranslation(AlignMode alignMode) {
        // Normalize alignment mode to uppercase.
        // Get current alliance; default to Blue if not present.
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        // Blue Alliance button polling.
        if (alliance == DriverStation.Alliance.Blue) {
            if (DriverStation.getStickButton(1, ButtonConstants.kCoralA)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kAL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kAR, null); 
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kAAlgea);
                }
            } else if (DriverStation.getStickButton(1, ButtonConstants.kCoralB)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kBL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kBR, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kBAlgea);  
                }
            } else if (DriverStation.getStickButton(1, ButtonConstants.kCoralC)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kCL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kCR, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kCAlgea);
                }
            } else if (DriverStation.getStickButton(1, ButtonConstants.kCoralD)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kDL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kDR, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kDAlgea);
                }
            } else if (DriverStation.getStickButton(1, ButtonConstants.kCoralE)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kEL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kER, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kEAlgea);
                }
            } else if (DriverStation.getStickButton(1, ButtonConstants.kCoralF)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kFL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.BlueAllianceConstants.kFR, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kFAlgea);
                }
            }
        } 
        // Red Alliance button polling.
        else  {
            if (DriverStation.getStickButton(1, ButtonConstants.kCoralA)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kAL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kAR, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kAAlgea);
                }
            } else if (DriverStation.getStickButton(1, ButtonConstants.kCoralB)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kBL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kBR, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kBAlgea);  
                }
            } else if (DriverStation.getStickButton(1, ButtonConstants.kCoralC)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kCL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kCR, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kCAlgea);
                }
            } else if (DriverStation.getStickButton(1, ButtonConstants.kCoralD)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kDL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kDR, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kDAlgea);
                }
            } else if (DriverStation.getStickButton(1, ButtonConstants.kCoralE)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kEL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kER, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kEAlgea);
                }
            } else if (DriverStation.getStickButton(1, ButtonConstants.kCoralF)) {
                switch (alignMode) {
                    case LEFT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kFL, null);
                    case RIGHT -> new Pose2d(AutoAlignConstants.RedAllianceConstants.kFR, null);
                    case ALGAE -> new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kFAlgea);
                }
            }
        }
        // No button pressed returns null.
        return null;
    }
}
