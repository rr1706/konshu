package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.constants.ButtonConstants;
import frc.robot.commands.PIDRotateToTrajectory;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.utilities.ReefTargetCalculator;
import frc.robot.utilities.ReefTargetCalculator.AlignMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RotateArmToTarget calculates the desired arm angle based on the robot’s current pose,
 * a target translation selected via reef button polling, and the current elevator height.
 * 
 * The idea is that the target scoring element is at a fixed height (e.g. 72"),
 * and when the robot backs up a certain distance (e.g. 3"), a full offset of 48° is applied.
 * The computed offset is scaled by the current elevator height relative to a reference (80").
 * 
 * The desired arm angle is computed as:
 * 
 *     desiredArmAngle = BASE_ARM_ANGLE + (distanceAway / 3.0) * FULL_OFFSET_DEGREES * (elevatorHeight / REFERENCE_ELEVATOR_HEIGHT)
 * 
 * The command polls for the reef buttons (A–F) to set the target translation (or a preset rotation
 * in "ALGAE" mode, if needed) based on alliance color. If no valid button is pressed, the command ends.
 */
public class RotateArmToTarget extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Elevator elevator;
    private final Arm arm;
    // alignMode can be "LEFT", "RIGHT", or "ALGAE"
    private final AlignMode alignMode;
    
    // These will be updated via button polling.
    private Translation2d targetTranslation = new Translation2d(0.0, 0.0);
    // (For ALGAE mode we could use a preset rotation, but here we compute based on distance.)
    private boolean finished = false;

    // Tuning constants for the arm adjustment.
    private final double BASE_ARM_ANGLE = ArmConstants.kArmL4; // e.g., a tuned baseline angle for L4 scoring.
    private final double FULL_OFFSET_DEGREES = ArmConstants.kArmL4;       // full offset in degrees when backed up 3 inches
    private final double REFERENCE_ELEVATOR_HEIGHT = 80.0;   // reference elevator height in inches

    public RotateArmToTarget(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, AlignMode alignMode) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.arm = arm;
//        this.alignMode = alignMode.toUpperCase(); // ensure uppercase for consistency
        this.alignMode = alignMode;
        addRequirements(arm, elevator, drivetrain);
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        Pose2d target = ReefTargetCalculator.calculateTargetTranslation(alignMode);
        if (target == null) {
            // No valid reef button pressed – optionally mark the command as finished.
            finished = true;
            return;
        }
        targetTranslation = target.getTranslation();
        // Retrieve current robot pose (from drivetrain) to calculate the horizontal distance.
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d robotTranslation = currentPose.getTranslation();
        Translation2d vectorToTarget = targetTranslation.minus(robotTranslation);
        double distanceAway = Math.hypot(vectorToTarget.getX(), vectorToTarget.getY());
        SmartDashboard.putNumber("ArmDistanceToTarget", distanceAway);

        // Get current elevator height (in inches)
        double elevatorHeight = elevator.getPosition();

        // Compute the offset in degrees.
        // When distanceAway is 3 inches at a reference elevator height, the full offset is applied.
        double offset = (distanceAway / 3.0) * FULL_OFFSET_DEGREES * (elevatorHeight / REFERENCE_ELEVATOR_HEIGHT);

        // Calculate the desired arm angle.
        double desiredArmAngle = BASE_ARM_ANGLE + offset;
        SmartDashboard.putNumber("DesiredArmAngle", desiredArmAngle);

        // Command the arm to move to the computed angle.
        arm.setPosition(desiredArmAngle);
    }

    @Override
    public void end(boolean interrupted) {
        // Optionally, you could hold the current setpoint or stop the arm.
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
