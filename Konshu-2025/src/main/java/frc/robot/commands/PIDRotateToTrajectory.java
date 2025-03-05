package frc.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.ReefTargetCalculator;
import frc.robot.utilities.ReefTargetCalculator.AlignMode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

/**
 * PIDRotateToTrajectory rotates the robot to face a target (selected via joystick button polling)
 * while allowing the driver to control translation (x and y) via the joysticks.
 * 
 * The alignment mode is provided as (LEFT, RIGHT, or ALGAE). Based on this mode,
 * the button polling code selects either a target Translation2d or a preset Rotation2d from the
 * AutoAlignConstants for the current alliance.
 */
public class PIDRotateToTrajectory extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier forwardBack;
    private final DoubleSupplier leftRight;
    private final AlignMode alignMode;

    // These will be updated via button polling.
    private Translation2d m_Translation = null;
    private Rotation2d m_Rotation = null;
    private Pose2d m_Pose = null;

    // PID controller for rotation. Tune gains and constraints as needed.
    private final PIDController rotPID = new PIDController(
            6.0, 0.0, 0.0    );

    // Base CTRE FieldCentric swerve request (using velocity control)
    private final SwerveRequest.FieldCentric baseRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_SPEED * DriveConstants.DRIVE_DEADBAND)
            .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND)
            .withDriveRequestType(DriveRequestType.Velocity);

    public PIDRotateToTrajectory(CommandSwerveDrivetrain drivetrain,
                                 DoubleSupplier forwardBack,
                                 DoubleSupplier leftRight,
                                 AlignMode alignMode) {
        this.drivetrain = drivetrain;
        this.forwardBack = forwardBack;
        this.leftRight = leftRight;
        this.alignMode = alignMode;
        
        // Enable continuous input for proper angle wrapping (from -π to π)
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(0.05);
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        double currentAngle = drivetrain.getPigeon2().getRotation2d().getRadians();
        SmartDashboard.putNumber("Current Angle", currentAngle);
    }

    @Override
    public void execute() {
        m_Pose = ReefTargetCalculator.calculateTargetTranslation(alignMode);
        m_Translation = m_Pose.getTranslation();
        m_Rotation = m_Pose.getRotation();
        if (m_Translation == null) {
            // No valid reef button pressed – optionally mark the command as finished.
            return;
        }
        double [] target_array ={m_Translation.getX(), m_Translation.getY()};
        SmartDashboard.putNumberArray("Target",target_array);
        Pose2d currentPose = drivetrain.getState().Pose;
        double currentAngle = currentPose.getRotation().getRadians();
        SmartDashboard.putNumber("CurrentAngle", currentAngle);
        Translation2d robotToGoal = m_Translation.minus(currentPose.getTranslation());
        double distToGoal = robotToGoal.getDistance(new Translation2d());


        double targetAngle;
        // For ALGAE mode, use the preset rotation; otherwise, compute the angle from the target translation.
        if (alignMode == AlignMode.ALGAE) {
            targetAngle = m_Rotation.getRadians();
        } else {
            targetAngle = robotToGoal.getAngle().getRadians();
        }

        SmartDashboard.putString("Align Mode", alignMode.toString());
        SmartDashboard.putNumber("Target Angle", targetAngle);
        // Compute the PID controller output.
        double rotationOutput = rotPID.calculate(currentAngle, targetAngle);
        SmartDashboard.putNumber("Rot Out", rotationOutput);
        // Compute translation speeds from joystick inputs with a custom curve.
        double transAdjustment = adjustInputCurve(forwardBack.getAsDouble(), leftRight.getAsDouble(), 0.7, 0.3);
        double velocityX = -forwardBack.getAsDouble() * DriveConstants.MAX_SPEED * transAdjustment;
        double velocityY = -leftRight.getAsDouble() * DriveConstants.MAX_SPEED * transAdjustment;

        // Build and apply the CTRE swerve request.
        SwerveRequest request = baseRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationOutput);
        drivetrain.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain for safety.
        SwerveRequest request = baseRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0);
        drivetrain.setControl(request);
    }


    // Helper method to adjust the joystick input curve.
    private double adjustInputCurve(double x, double y, double a, double b) {
        double magnitude = Math.sqrt(x * x + y * y);
        if (magnitude > 1.0) {
            magnitude = 1.0;
        }
        if (magnitude < 0.0) {
            magnitude = 0.0;
        }
        return a * Math.pow(magnitude, 3) + b * magnitude;
    }
}
