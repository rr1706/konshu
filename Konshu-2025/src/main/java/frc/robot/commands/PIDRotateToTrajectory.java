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
import frc.robot.subsystems.Elevator;

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
            6.0, 0.0, 0.0    );

    // Base CTRE FieldCentric swerve request (using velocity control)
    private final SwerveRequest.FieldCentric baseRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_SPEED * DriveConstants.DRIVE_DEADBAND)
            .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND)
            .withDriveRequestType(DriveRequestType.Velocity);

    public PIDRotateToTrajectory(CommandSwerveDrivetrain drivetrain,
                                 DoubleSupplier forwardBack,
                                 DoubleSupplier leftRight,
                                 DoubleSupplier rotation,
                                 Arm arm, Elevator el, SSM ssm) {
        this.drivetrain = drivetrain;
        this.forwardBack = forwardBack;
        this.leftRight = leftRight;
        this.rotation = rotation;   
        this.m_arm = arm;
        this.m_elevator = el;
        this.m_SSM = ssm;
        
        // Enable continuous input for proper angle wrapping (from -π to π)
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        rotPID.setTolerance(0.05);
        
        addRequirements(drivetrain, ssm, arm, el);
    }

    @Override
    public void initialize() {
        double currentAngle = drivetrain.getPigeon2().getRotation2d().getRadians();
        SmartDashboard.putNumber("Current Angle", currentAngle);
    }

    @Override
    public void execute() {
        double rotationOutput;

        m_goforPID = false;
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

//      Dither - adjust elevation and arm angle as a function of distance (dist) from the coral post AND
//      the angle between coral wall normal and current pose angle (theta) AND
//      the elevator nominal height and arm nominal angle for the current state.
//      Set ArmNominimal and ElevatorNominal constants to be when the robot at MAX_DIST away and aligned normal to the coral wall

//      2DO: Do we want to also vary the eject speed based on m_d?

                // double dist = robotToGoal.getDistance(new Translation2d());    // Distance to post from robot
                // Rotation2d theta = m_Pose.getRotation().minus(currentPose.getRotation());    // Angle from coral wall normal
                // double elevatorNominal = m_SSM.getScoringElevatorPosition(m_state);   // Need to add and set m_state in above switch
                // double armNominal = m_SSM.getScoringArmPosition(m_state);             // Need to add amd set m_state in above switch
                // final double MAX_DIST = 30.0;                       // Max distance from the post in inches to start dithering based on distance
                // final double ELEVATOR_MAX_DITHER_ROTATION = 3.0;    // Max elevator dither in inches based on angle from coral wall normal
                // final double ARM_MAX_DITHER_ROTATION = 5.0;         // Max arm dither in degrees based on angle from coral wall normal
                // final double ELEVATOR_MAX_DITHER_DIST = 6.0;        // Max elevator dither in inches based on distance from post
                // final double ARM_MAX_DITHER_DIST = 10.0;            // Max arm dither in degrees based on distance from post
                // double anglefactor = 1.0 - Math.cos(theta.getRadians());
                // double distfactor = Math.max(0.0, 1.0 - dist/MAX_DIST);
                // double ElevatorDither = elevatorNominal + ELEVATOR_MAX_DITHER_ROTATION*anglefactor + ELEVATOR_MAX_DITHER_DIST*distfactor;
                // double ArmDither = armNominal + ARM_MAX_DITHER_ROTATION*anglefactor + ARM_MAX_DITHER_DIST*distfactor;
                // m_elevator.setPosition(ElevatorDither);
                // m_arm.setPosition(ArmDither*180.0/Math.PI);
            }

            SmartDashboard.putString("Align Mode", m_alignMode.toString());
            SmartDashboard.putNumber("Target Angle", targetAngle);
            // Compute the PID controller output.
            rotationOutput = rotPID.calculate(currentAngle, targetAngle);
            SmartDashboard.putNumber("Rot Out", rotationOutput);
        } else {
            double rotCurveAdjustment = DriveCommands.adjustRotCurve(rotation.getAsDouble(), 0.7, 0.3);
            rotationOutput =  DriveCommands.m_slewRot.calculate(-rotation.getAsDouble() * DriveConstants.MAX_ANGULAR_RATE*rotCurveAdjustment); 
        }

        // Compute translation speeds from joystick inputs with a custom curve.
        double transAdjustment = adjustInputCurve(forwardBack.getAsDouble(), leftRight.getAsDouble(), 0.7, 0.3);
        double velocityX = slewX.calculate(-forwardBack.getAsDouble() * DriveConstants.MAX_SPEED * transAdjustment);
        double velocityY = slewY.calculate(-leftRight.getAsDouble() * DriveConstants.MAX_SPEED * transAdjustment);

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
