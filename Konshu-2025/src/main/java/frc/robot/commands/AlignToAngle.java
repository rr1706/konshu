package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.subsystems.SSM;

public class AlignToAngle extends Command {
        private final Supplier<Rotation2d> m_target;
    private final Supplier<Pose2d> m_drivePose;

    private final PIDController rotPID = new PIDController(
            10.0, 0.0, 0.3);
 
    public AlignToAngle(Supplier<Pose2d> drivePose, Supplier<Rotation2d> target){
        m_target = target;
        m_drivePose = drivePose;

        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(0.05);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drivePose.get();
        double currentAngle = currentPose.getRotation().getRadians();

        double targetAngle = m_target.get().getRadians();


        double rotationOutput = rotPID.calculate(currentAngle, targetAngle);

        PPHolonomicDriveController.overrideRotationFeedback(() ->
        rotationOutput);

    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }
}