package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SSM;

public class AlignInAuto extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Supplier<Translation2d> m_target;
    private final SSM.States m_state;
    private final SSM m_ssm;
    private final PIDController rotPID = new PIDController(
            15.0, 0.0, 0.5);

    // Base CTRE FieldCentric swerve request (using velocity control)
    private final SwerveRequest.FieldCentric baseRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_SPEED * DriveConstants.DRIVE_DEADBAND)
            .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND)
            .withDriveRequestType(DriveRequestType.Velocity);

    public AlignInAuto(CommandSwerveDrivetrain drivetrain, Supplier<Translation2d> target, SSM.States state, SSM ssm) {
        m_drivetrain = drivetrain;
        m_target = target;
        m_state = state;
        m_ssm = ssm;
        // Enable continuous input for proper angle wrapping (from -π to π)
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(0.05);
        addRequirements(m_drivetrain);

    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drivetrain.getState().Pose;
        double currentAngle = currentPose.getRotation().getRadians();

        Translation2d target = m_target.get();

        double[] target_array = { target.getX(), target.getY() };
        SmartDashboard.putNumberArray("Target", target_array);
        Translation2d robotToGoal = target.minus(currentPose.getTranslation());
        double targetAngle = robotToGoal.getAngle().getRadians();
        double elevatorOffset = 0.0;
        double armOffset = 0.0;
        // Adjust elevator based on distance (and evantually delta angle) from post
        double dist = target.getDistance(currentPose.getTranslation()); // Distance to post from robot
        SmartDashboard.putNumber("Distance to target", dist);
        SmartDashboard.putNumber("Angle to Post (deg)", target.getAngle().getRadians() * 360.0 / Math.PI);
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

        SmartDashboard.putNumber("Target Angle", targetAngle);
        Double DifferenceinAngle = targetAngle - currentAngle;
        SmartDashboard.putNumber("Difference In Angle", DifferenceinAngle);

        double rotationOutput = rotPID.calculate(currentAngle, targetAngle);
        SmartDashboard.putNumber("Rot Out", rotationOutput);

        double velocityX = 0.0;
        double velocityY = 0.0;

        SwerveRequest request = baseRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationOutput);
        m_drivetrain.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveRequest request = baseRequest
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0);
        m_drivetrain.setControl(request);
    }

}
