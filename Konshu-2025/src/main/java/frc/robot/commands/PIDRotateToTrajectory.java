package frc.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ButtonConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SSM;
import frc.robot.utilities.ReefTargetCalculator;
import frc.robot.commands.DriveCommands;
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
    private AlignMode alignMode;
    private final SSM m_SSM;

    // Updated via button polling
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
                                 SSM ssm) {
        this.drivetrain = drivetrain;
        this.forwardBack = forwardBack;
        this.leftRight = leftRight;
        this.m_SSM = ssm;
        
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

        if (DriverStation.getStickButton(1, ButtonConstants.kL1Left)) {
            m_SSM.setState(SSM.States.L1);
            alignMode = ReefTargetCalculator.AlignMode.LEFT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL1Right)) {
            m_SSM.setState(SSM.States.L1);
            alignMode = ReefTargetCalculator.AlignMode.RIGHT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL2Left)) {
            m_SSM.setState(SSM.States.L2);
            alignMode = ReefTargetCalculator.AlignMode.LEFT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL2Right)) {
            m_SSM.setState(SSM.States.L2);
            alignMode = ReefTargetCalculator.AlignMode.RIGHT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL3Left)) {
            m_SSM.setState(SSM.States.L3);
            alignMode = ReefTargetCalculator.AlignMode.LEFT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL3Right)) {
            m_SSM.setState(SSM.States.L3);
            alignMode = ReefTargetCalculator.AlignMode.RIGHT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL4Left)) {
            m_SSM.setState(SSM.States.L4);
            alignMode = ReefTargetCalculator.AlignMode.LEFT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL4Right)) {
            m_SSM.setState(SSM.States.L4);
            alignMode = ReefTargetCalculator.AlignMode.RIGHT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kLowAlgae)) {
            m_SSM.setState(SSM.States.ALGAELOW);
            alignMode = ReefTargetCalculator.AlignMode.ALGAE;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kHighAlgae)) {
            m_SSM.setState(SSM.States.ALGAEHIGH);
            alignMode = ReefTargetCalculator.AlignMode.ALGAE;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kBarge)) {
            m_SSM.setState(SSM.States.BARGE);
            return;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kProcessor)) {
            m_SSM.setState(SSM.States.PROCESSOR);
            return;
        } else {
            return;
        }

        m_Pose = ReefTargetCalculator.calculateTargetTranslation(alignMode);
        if (m_Pose == null) {
            // No valid reef button pressed – optionally mark the command as finished.
            return;
        }

        Pose2d currentPose = drivetrain.getState().Pose;
        double currentAngle = currentPose.getRotation().getRadians();
        SmartDashboard.putNumber("CurrentAngle", currentAngle);

        double targetAngle;
        // For ALGAE mode, use the preset rotation; otherwise, compute the angle from the target translation.
        if (alignMode == AlignMode.ALGAE) {     // m_Pose only has rotation populated
            targetAngle = m_Pose.getRotation().getRadians();
        } else {                                // m_Pose only has translation populated
            double [] target_array ={m_Pose.getTranslation().getX(), m_Pose.getTranslation().getY()};
            SmartDashboard.putNumberArray("Target",target_array);
            Translation2d robotToGoal = m_Pose.getTranslation().minus(currentPose.getTranslation());
            targetAngle = robotToGoal.getAngle().getRadians();
//          double distToGoal = robotToGoal.getDistance(new Translation2d());
        }

        SmartDashboard.putString("Align Mode", alignMode.toString());
        SmartDashboard.putNumber("Target Angle", targetAngle);
        // Compute the PID controller output.
        double rotationOutput = rotPID.calculate(currentAngle, targetAngle);
        SmartDashboard.putNumber("Rot Out", rotationOutput);
        // Compute translation speeds from joystick inputs with a custom curve.
        double transAdjustment = adjustInputCurve(forwardBack.getAsDouble(), leftRight.getAsDouble(), 0.7, 0.3);
        double velocityX = DriveCommands.m_slewX.calculate(-forwardBack.getAsDouble() * DriveConstants.MAX_SPEED * transAdjustment);
        double velocityY = DriveCommands.m_slewY.calculate(-leftRight.getAsDouble() * DriveConstants.MAX_SPEED * transAdjustment);

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
