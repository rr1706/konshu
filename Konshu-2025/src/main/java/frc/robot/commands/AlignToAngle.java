package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToAngle extends Command {
        private final Supplier<Rotation2d> m_target;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final double m_xSpeed;
    private final double m_ySpeed;

        private final SwerveRequest.RobotCentric baseRequest = new SwerveRequest.RobotCentric()
            .withDeadband(DriveConstants.MAX_SPEED * DriveConstants.DRIVE_DEADBAND)
            .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final PIDController rotPID = new PIDController(
        12.0, 0.0, 0.3);
 
    public AlignToAngle(CommandSwerveDrivetrain drivetrain, Supplier<Rotation2d> target, double xSpeed, double ySpeed){
        m_target = target;
        m_drivetrain = drivetrain;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(0.05);
        addRequirements(m_drivetrain);

    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drivetrain.getState().Pose;
        double currentAngle = currentPose.getRotation().getRadians();

        double targetAngle = m_target.get().getRadians();


        double rotationOutput = rotPID.calculate(currentAngle, targetAngle);

        double velocityX = m_xSpeed;
        double velocityY = m_ySpeed;

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
        m_drivetrain.setControl(request);    }
}