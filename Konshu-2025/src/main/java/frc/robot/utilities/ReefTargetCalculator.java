package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.constants.ButtonConstants;
import frc.robot.Robot;
import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class ReefTargetCalculator {
    public enum AlignMode {LEFT, RIGHT, ALGAE};
    public enum ReefFace {None, BlueA, RedA, BlueB, RedB, BlueC, RedC, BlueD, RedD, BlueE, RedE, BlueF, RedF};
    private static ReefFace m_face = ReefFace.None;
    /**
    * Calculates the target translation based on the current alliance,
    * the state of reef (Coral) buttons, and the specified alignment mode.  
    * @return the selected target Pose2d, or null if no button is pressed.
    */
    public static Pose2d calculateTargetTranslation(AlignMode alignMode, Pose2d currentPose) {

        // Get the closest reef face
        m_face = getClosestReefFace(currentPose);
        
        // Get current alliance; default to Blue if not present.
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        if (alliance == DriverStation.Alliance.Blue) {         // Blue Alliance button polling.
            if ((m_face == ReefFace.BlueA) || (DriverStation.getStickButton(2, ButtonConstants.kCoralA))) {
                switch (alignMode) {
                    case LEFT: 
                        Robot.buttonLog.append("CoralA Left Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kAL, AutoAlignConstants.BlueAllianceConstants.kAAlgea);
                    case RIGHT: 
                        Robot.buttonLog.append("CoralA Right Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kAR, AutoAlignConstants.BlueAllianceConstants.kAAlgea); 
                    case ALGAE: 
                        Robot.buttonLog.append("CoralA Algea Blue");
                        return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kAAlgea);
                }
            } else if ((m_face == ReefFace.BlueB) || (DriverStation.getStickButton(2, ButtonConstants.kCoralB))) {
                switch (alignMode) {
                    case LEFT:  
                        Robot.buttonLog.append("CoralB Left Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kBL, AutoAlignConstants.BlueAllianceConstants.kBAlgea);
                    case RIGHT: 
                        Robot.buttonLog.append("CoralB Right Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kBR, AutoAlignConstants.BlueAllianceConstants.kBAlgea);
                    case ALGAE:
                        Robot.buttonLog.append("CoralB Algea Blue");
                        return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kBAlgea);  
                }
            } else if ((m_face == ReefFace.BlueC) || (DriverStation.getStickButton(2, ButtonConstants.kCoralC))) {
                switch (alignMode) {
                    case LEFT: 
                        Robot.buttonLog.append("CoralC Left Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kCL, AutoAlignConstants.BlueAllianceConstants.kCAlgea);
                    case RIGHT: 
                        Robot.buttonLog.append("CoralC Right Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kCR, AutoAlignConstants.BlueAllianceConstants.kCAlgea);
                    case ALGAE: 
                        Robot.buttonLog.append("CoralC Algea Blue");
                        return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kCAlgea);
                }
            } else if ((m_face == ReefFace.BlueD) || (DriverStation.getStickButton(2, ButtonConstants.kCoralD))) {
                switch (alignMode) {
                    case LEFT: 
                        Robot.buttonLog.append("CoralD Left Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kDL, AutoAlignConstants.BlueAllianceConstants.kDAlgea);
                    case RIGHT:
                        Robot.buttonLog.append("CoralD Right Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kDR, AutoAlignConstants.BlueAllianceConstants.kDAlgea);
                    case ALGAE: 
                        Robot.buttonLog.append("CoralD Algea Blue");
                        return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kDAlgea);
                }
            } else if ((m_face == ReefFace.BlueE) || (DriverStation.getStickButton(2, ButtonConstants.kCoralE))) {
                switch (alignMode) {
                    case LEFT: 
                        Robot.buttonLog.append("CoralE Left Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kEL, AutoAlignConstants.BlueAllianceConstants.kEAlgea);
                    case RIGHT: 
                        Robot.buttonLog.append("CoralE Right Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kER, AutoAlignConstants.BlueAllianceConstants.kEAlgea);
                    case ALGAE:
                        Robot.buttonLog.append("CoralE Algea Blue");
                        return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kEAlgea);
                }
            } else if ((m_face == ReefFace.BlueF) || (DriverStation.getStickButton(2, ButtonConstants.kCoralF))) {
                switch (alignMode) {
                    case LEFT: 
                        Robot.buttonLog.append("CoralF Left Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kFL, AutoAlignConstants.BlueAllianceConstants.kFAlgea);
                    case RIGHT: 
                        Robot.buttonLog.append("CoralF Right Blue");
                        return new Pose2d(AutoAlignConstants.BlueAllianceConstants.kFR, AutoAlignConstants.BlueAllianceConstants.kFAlgea);
                    case ALGAE:
                        Robot.buttonLog.append("CoralF Algea Blue");
                        return new Pose2d(null, AutoAlignConstants.BlueAllianceConstants.kFAlgea);
                }
            }
        } else  {         // Red Alliance button polling.
            if ((m_face == ReefFace.RedA) || (DriverStation.getStickButton(2, ButtonConstants.kCoralA))) {
                switch (alignMode) {
                    case LEFT: 
                        Robot.buttonLog.append("CoralA Left Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kAL, AutoAlignConstants.RedAllianceConstants.kAAlgea);
                    case RIGHT:
                        Robot.buttonLog.append("CoralA Right Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kAR, AutoAlignConstants.RedAllianceConstants.kAAlgea);
                    case ALGAE:
                        Robot.buttonLog.append("CoralA Algea Red");
                        return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kAAlgea);
                }
            } else if ((m_face == ReefFace.RedB) || (DriverStation.getStickButton(2, ButtonConstants.kCoralB))) {
                switch (alignMode) {
                    case LEFT:
                        Robot.buttonLog.append("CoralB Left Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kBL, AutoAlignConstants.RedAllianceConstants.kBAlgea);
                    case RIGHT:
                        Robot.buttonLog.append("CoralB Right Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kBR, AutoAlignConstants.RedAllianceConstants.kBAlgea);
                    case ALGAE:
                        Robot.buttonLog.append("CoralB Algea Red");
                        return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kBAlgea);  
                }
            } else if ((m_face == ReefFace.RedC) || (DriverStation.getStickButton(2, ButtonConstants.kCoralC))) {
                switch (alignMode) {
                    case LEFT:
                        Robot.buttonLog.append("CoralC Left Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kCL, AutoAlignConstants.RedAllianceConstants.kCAlgea);
                    case RIGHT: 
                        Robot.buttonLog.append("CoralC Right Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kCR,AutoAlignConstants.RedAllianceConstants.kCAlgea);
                    case ALGAE:
                        Robot.buttonLog.append("CoralC Algea Red");
                        return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kCAlgea);
                }
            } else if ((m_face == ReefFace.RedD) || (DriverStation.getStickButton(2, ButtonConstants.kCoralD))) {
                switch (alignMode) {
                    case LEFT: 
                        Robot.buttonLog.append("CoralD Left Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kDL, AutoAlignConstants.RedAllianceConstants.kDAlgea);
                    case RIGHT: 
                        Robot.buttonLog.append("CoralD Right Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kDR, AutoAlignConstants.RedAllianceConstants.kDAlgea);
                    case ALGAE:
                        Robot.buttonLog.append("CoralD Algea Red");
                        return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kDAlgea);
                }
            } else if ((m_face == ReefFace.RedE) || (DriverStation.getStickButton(2, ButtonConstants.kCoralE))) {
                switch (alignMode) {
                    case LEFT: 
                        Robot.buttonLog.append("CoralE Left Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kEL, AutoAlignConstants.RedAllianceConstants.kEAlgea);
                    case RIGHT:
                        Robot.buttonLog.append("CoralE Right Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kER, AutoAlignConstants.RedAllianceConstants.kEAlgea);
                    case ALGAE:
                        Robot.buttonLog.append("CoralE Algea Red");
                        return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kEAlgea);
                }
            } else if ((m_face == ReefFace.RedF) || (DriverStation.getStickButton(2, ButtonConstants.kCoralF))) {
                switch (alignMode) {
                    case LEFT: 
                        Robot.buttonLog.append("CoralF Left Red"); 
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kFL, AutoAlignConstants.RedAllianceConstants.kFAlgea);
                    case RIGHT:
                        Robot.buttonLog.append("CoralF Right Red");  
                        return new Pose2d(AutoAlignConstants.RedAllianceConstants.kFR, AutoAlignConstants.RedAllianceConstants.kFAlgea);
                    case ALGAE:
                        Robot.buttonLog.append("CoralF Algea Red");
                        return new Pose2d(null, AutoAlignConstants.RedAllianceConstants.kFAlgea);
                }
            }
        }

        // No button pressed returns null.
        return null;
    }

// Function to return the closest reef face enum based on the current pose.   There is probably a really
// cool OO way to do this, but this should work. 
    public static ReefFace getClosestReefFace(Pose2d CurrentPose) {
        double dist, minDist;
        int minIndex;
        // Note - the April tag numbers in reeftag must coorespond to the ReefFace enums in the same order
        final int[] reeftag = {18, 19, 20, 21, 22, 17, 7, 6, 11, 10, 9, 8};
        final ReefFace[] face = {ReefFace.BlueA, ReefFace.BlueB, ReefFace.BlueC, ReefFace.BlueD, ReefFace.BlueE, ReefFace.BlueF, ReefFace.RedA, ReefFace.RedB, ReefFace.RedC, ReefFace.RedD, ReefFace.RedE, ReefFace.RedF};
        final AprilTagFieldLayout layout;
        try {
            layout = new AprilTagFieldLayout(
                Path.of(
                    "src",
                    "main",
                    "deploy",
                    "apriltags",
                    "2025-official.json"));
        } catch (IOException e) {
            e.printStackTrace();
            return ReefFace.None;
        }

// NEED TO DEFINE THE OVERRIDE BUTTON IN BUTTONCONSTANTS
        // Return "None" if override button is pushed so we will use the reef buttons instead of closest reef face
        // if (DriverStation.getStickButton(2, ButtonConstants.kOverride)) {
        //    return ReefFace.None;
        // }

        // Determine the closest reef face and return the cooresponding enum
        minDist = 10000.0;
        minIndex = 0;
        for (int i = 0; i < 12; i++) {
            dist = layout.getTagPose(reeftag[i]).get().toPose2d().getTranslation().getDistance(CurrentPose.getTranslation());
            if (dist < minDist) {
                minDist = dist;
                minIndex = i;
            }
        }
        return(face[minIndex]);
    }
}




