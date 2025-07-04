package frc.robot.commands;

import java.util.function.Supplier;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.subsystems.SSM;

public class AlignInPath extends Command {
    private final Supplier<Translation2d> m_target;
    private final Supplier<Pose2d> m_drivePose;
    private final SSM.States m_state;
    private final SSM m_ssm;

    private final PIDController rotPID = new PIDController(
            10.0, 0.0, 0.3);

    public AlignInPath(Supplier<Pose2d> drivePose, Supplier<Translation2d> target, SSM.States state, SSM ssm) {
        m_target = target;
        m_drivePose = drivePose;
        m_state = state;
        m_ssm = ssm;

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

        Translation2d target = m_target.get();

        Translation2d robotToGoal = target.minus(currentPose.getTranslation());
        double targetAngle = robotToGoal.getAngle().getRadians();
        double elevatorOffset = 0.0;
        double armOffset = 0.0;
        double dist = target.getDistance(currentPose.getTranslation()); // Distance to post from robot

        switch (m_state) {
            case L1:
                elevatorOffset = AutoAlignConstants.ElevatorAutoAlignL1.get(dist);
                armOffset = AutoAlignConstants.ArmAutoAlignL1.get(dist);
                break;
            case L2:
                elevatorOffset = AutoAlignConstants.ElevatorAutoAlignL2.get(dist);
                armOffset = AutoAlignConstants.ArmAutoAlignL2.get(dist);
                break;
            case L3:
                elevatorOffset = AutoAlignConstants.ElevatorAutoAlignL3.get(dist);
                armOffset = AutoAlignConstants.ArmAutoAlignL3.get(dist);
                break;
            case L4:
                elevatorOffset = AutoAlignConstants.ElevatorAutoAlignL4.get(dist);
                armOffset = AutoAlignConstants.ArmAutoAlignL4.get(dist);
                break;
            default:
                elevatorOffset = 0.0;
                armOffset = 0.0;
                break;
        }

        m_ssm.setState(m_state, armOffset, elevatorOffset);

        double rotationOutput = rotPID.calculate(currentAngle, targetAngle);

        PPHolonomicDriveController.overrideRotationFeedback(() ->
        rotationOutput);

    }

    @Override
    public void end(boolean interrupted) {
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }
}
