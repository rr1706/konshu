package frc.robot.commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ButtonConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SSM;
import frc.robot.subsystems.SSM.States;
import frc.robot.utilities.ReefTargetCalculator;
import frc.robot.utilities.ReefTargetCalculator.AlignMode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Elevator;

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
    private final DoubleSupplier rotation;
    private final SSM m_SSM;
    private States m_state;
    private AlignMode m_alignMode;
    private boolean m_goforPID = true;

    // Updated via button polling
    private Pose2d m_Pose = new Pose2d();

    // PID controller for rotation. Tune gains and constraints as needed.
    private final PIDController rotPID = new PIDController(
            15.0, 0.0, 0.5    );

    // Base CTRE FieldCentric swerve request (using velocity control)
    private final SwerveRequest.FieldCentric baseRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_SPEED * DriveConstants.DRIVE_DEADBAND)
            .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND)
            .withDriveRequestType(DriveRequestType.Velocity);

    public PIDRotateToTrajectory(CommandSwerveDrivetrain drivetrain,
                                 DoubleSupplier forwardBack,
                                 DoubleSupplier leftRight,
                                 DoubleSupplier rotation,
                                  SSM ssm) {
        this.drivetrain = drivetrain;
        this.forwardBack = forwardBack;
        this.leftRight = leftRight;
        this.rotation = rotation;   
        // this.m_arm = arm;
        // this.m_elevator = el;
        this.m_SSM = ssm;
        
        // Enable continuous input for proper angle wrapping (from -π to π)
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(0.05);
        
        addRequirements(drivetrain, ssm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double rotationOutput;

        m_goforPID = true;
        if (DriverStation.getStickButton(1, ButtonConstants.kL1Left)) {
            m_state = SSM.States.L1;
            m_SSM.setState(m_state);
            m_alignMode = ReefTargetCalculator.AlignMode.LEFT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL1Right)) {
            m_state = SSM.States.L1;
            m_SSM.setState(m_state);
            m_alignMode = ReefTargetCalculator.AlignMode.RIGHT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL2Left)) {
            m_state = SSM.States.L2;
            m_SSM.setState(m_state);
            m_alignMode = ReefTargetCalculator.AlignMode.LEFT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL2Right)) {
            m_state = SSM.States.L2;
            m_SSM.setState(m_state);
            m_alignMode = ReefTargetCalculator.AlignMode.RIGHT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL3Left)) {
            m_state = SSM.States.L3;
            m_SSM.setState(m_state);
            m_alignMode = ReefTargetCalculator.AlignMode.LEFT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL3Right)) {
            m_state = SSM.States.L3;
            m_SSM.setState(m_state);
            m_alignMode = ReefTargetCalculator.AlignMode.RIGHT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL4Left)) {
            m_state = SSM.States.L4;
            m_SSM.setState(m_state);
            m_alignMode = ReefTargetCalculator.AlignMode.LEFT;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL4Right)) {
            m_state = SSM.States.L4;
            m_SSM.setState(m_state);
            m_alignMode = ReefTargetCalculator.AlignMode.RIGHT;
        } else if (DriverStation.getStickButton(2, ButtonConstants.kLowAlgae)) {
            m_state = SSM.States.ALGAELOW;
            m_SSM.setState(m_state);
            m_alignMode = ReefTargetCalculator.AlignMode.ALGAE;
        } else if (DriverStation.getStickButton(1, ButtonConstants.kHighAlgae)) {
            m_state = SSM.States.ALGAEHIGH;
            m_SSM.setState(m_state);
            m_alignMode = ReefTargetCalculator.AlignMode.ALGAE;
        } else if (DriverStation.getStickButton(2, ButtonConstants.kBarge)) {
            m_SSM.setState(SSM.States.BARGE);
            m_goforPID = false;
        } else if (DriverStation.getStickButton(2, ButtonConstants.kProcessor)) {
            m_SSM.setState(SSM.States.PROCESSOR);
            m_goforPID = false;
        } else m_goforPID = false;            // No level button pressed - do nothing

        if (m_goforPID) m_Pose = ReefTargetCalculator.calculateTargetTranslation(m_alignMode);
        if (m_Pose == null) m_goforPID = false;  // No reef button pressed – optionally mark the command as finished
        SmartDashboard.putBoolean("goForPID", m_goforPID);
        if (m_goforPID) {
            Pose2d currentPose = drivetrain.getState().Pose;
            double currentAngle = currentPose.getRotation().getRadians();
            SmartDashboard.putNumber("CurrentAngle", currentAngle);

            double targetAngle;
            // For ALGAE mode, use the preset rotation; otherwise, compute the angle from the target translation.
            if (m_alignMode == AlignMode.ALGAE) {     // m_Pose only has rotation populated
                targetAngle = m_Pose.getRotation().getRadians();
            } else {
              double [] target_array ={m_Pose.getTranslation().getX(), m_Pose.getTranslation().getY()};
              SmartDashboard.putNumberArray("Target",target_array);
              Translation2d robotToGoal = m_Pose.getTranslation().minus(currentPose.getTranslation());
              targetAngle = robotToGoal.getAngle().getRadians();

            }

            SmartDashboard.putString("Align Mode", m_alignMode.toString());
            SmartDashboard.putNumber("Target Angle", targetAngle);
            Double DifferenceinAngle = targetAngle - currentAngle;
            SmartDashboard.putNumber("Difference In Angle", DifferenceinAngle);
            // Compute the PID controller output.
            rotationOutput = rotPID.calculate(currentAngle, targetAngle);
            SmartDashboard.putNumber("Rot Out", rotationOutput);
        } else {
            double rotCurveAdjustment = DriveCommands.adjustRotCurve(rotation.getAsDouble(), 0.7, 0.3);
            rotationOutput =  DriveCommands.m_slewRot.calculate(-rotation.getAsDouble() * DriveConstants.MAX_ANGULAR_RATE*rotCurveAdjustment); 
        }

        // Compute translation speeds from joystick inputs with a custom curve.\
        double transAdjustment = adjustInputCurve(forwardBack.getAsDouble(), leftRight.getAsDouble(), 0.7, 0.3);
        double velocityX = DriveCommands.m_slewX.calculate(-forwardBack.getAsDouble() * DriveConstants.MAX_SPEED *.7* transAdjustment);
        double velocityY = DriveCommands.m_slewY.calculate(-leftRight.getAsDouble() * DriveConstants.MAX_SPEED * .7*transAdjustment);

        // Build and apply the CTRE swerve request.
        SwerveRequest request = baseRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationOutput);
        drivetrain.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        m_SSM.setState(States.LOADINGSTATION);
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
