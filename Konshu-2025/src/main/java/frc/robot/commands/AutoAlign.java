package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.AutoAlignConstants;
import frc.robot.constants.ButtonConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SSM;
import frc.robot.subsystems.SSM.States;
import frc.robot.utilities.ReefTargetCalculator;
import frc.robot.utilities.ReefTargetCalculator.AlignMode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import frc.robot.subsystems.LED;

/**
 * PIDRotateToTrajectory rotates the robot to face a target (selected via
 * joystick button polling)
 * while allowing the driver to control translation (x and y) via the joysticks.
 * 
 * The alignment mode is provided as (LEFT, RIGHT, or ALGAE). Based on this
 * mode,
 * the button polling code selects either a target Translation2d or a preset
 * Rotation2d from the
 * AutoAlignConstants for the current alliance.
 */
public class AutoAlign extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final DoubleSupplier m_forwardBackSupplier;
    private final DoubleSupplier m_leftRightSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final SSM m_SSM;
    private final LED m_LED;
    private States m_state;
    private AlignMode m_alignMode;
    private boolean m_goForPID = true;

    // Updated via button polling
    private Pose2d m_pose = new Pose2d();

    // PID controller for rotation. Tune gains and constraints as needed.
    private final PIDController rotPID = new PIDController(12.0, 0.0, 0.5);

    // Base CTRE FieldCentric swerve request (using velocity control)
    private final SwerveRequest.FieldCentric baseRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_SPEED * DriveConstants.DRIVE_DEADBAND)
            .withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * DriveConstants.ROTATION_DEADBAND)
            .withDriveRequestType(DriveRequestType.Velocity);

    public AutoAlign(CommandSwerveDrivetrain drivetrain,
            DoubleSupplier forwardBack,
            DoubleSupplier leftRight,
            DoubleSupplier rotation,
            SSM ssm, LED led) {
        this.m_drivetrain = drivetrain;
        this.m_forwardBackSupplier = forwardBack;
        this.m_leftRightSupplier = leftRight;
        this.m_rotationSupplier = rotation;
        this.m_SSM = ssm;
        this.m_LED = led;

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
        double rotationOutput, armOffset, elevatorOffset, dist;

        armOffset = 0.0;
        elevatorOffset = 0.0;
        dist = 1.0; // Default value for LED logic
        m_state = SSM.States.LOADINGSTATION; // Default to here if trigger with no button pressed
        m_goForPID = true;
        if (DriverStation.getStickButton(1, ButtonConstants.kL1Left)) {
            m_state = SSM.States.L1;
            m_alignMode = ReefTargetCalculator.AlignMode.LEFT;
            Robot.buttonLog.append("L1Left");
// todo - need a dedicated button for L1_SPECIAL - using L1Right for testing
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL1Right)) {
            m_state = SSM.States.L1;
            m_alignMode = ReefTargetCalculator.AlignMode.RIGHT;
            Robot.buttonLog.append("L1Right");
        // } else if (DriverStation.getStickButton(1, ButtonConstants.kL1Right)) {
        //     m_state = SSM.States.L1_SPECIAL;
        //     m_goForPID = false;
        //     Robot.buttonLog.append("L1Special");
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL2Left)) {
            m_state = SSM.States.L2;
            m_alignMode = ReefTargetCalculator.AlignMode.LEFT;
            Robot.buttonLog.append("L2Left");
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL2Right)) {
            m_state = SSM.States.L2;
            m_alignMode = ReefTargetCalculator.AlignMode.RIGHT;
            Robot.buttonLog.append("L2Right");
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL3Left)) {
            m_state = SSM.States.L3;
            m_alignMode = ReefTargetCalculator.AlignMode.LEFT;
            Robot.buttonLog.append("L3Left");
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL3Right)) {
            m_state = SSM.States.L3;
            m_alignMode = ReefTargetCalculator.AlignMode.RIGHT;
            Robot.buttonLog.append("L3Right");
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL4Left)) {
            m_state = SSM.States.L4;
            m_alignMode = ReefTargetCalculator.AlignMode.LEFT;
            Robot.buttonLog.append("L4Left");
        } else if (DriverStation.getStickButton(1, ButtonConstants.kL4Right)) {
            m_state = SSM.States.L4;
            m_alignMode = ReefTargetCalculator.AlignMode.RIGHT;
            Robot.buttonLog.append("L4Right");
        } else if (DriverStation.getStickButton(2, ButtonConstants.kLowAlgae)) {
            m_state = SSM.States.ALGAELOW;
            m_alignMode = ReefTargetCalculator.AlignMode.ALGAE;
            Robot.buttonLog.append("LowAlgae");
        } else if (DriverStation.getStickButton(1, ButtonConstants.kHighAlgae)) {
            m_state = SSM.States.ALGAEHIGH;
            m_alignMode = ReefTargetCalculator.AlignMode.ALGAE;
            Robot.buttonLog.append("HighAlgae");
        } else if (DriverStation.getStickButton(2, ButtonConstants.kBarge)) {
            m_state = SSM.States.BARGE;
            m_goForPID = false;
            Robot.buttonLog.append("Barge");
        } else if (DriverStation.getStickButton(2, ButtonConstants.kProcessor)) {
            m_state = SSM.States.PROCESSOR;
            m_goForPID = false;
            Robot.buttonLog.append("Processor");
        } else
            m_goForPID = false; // No level button pressed - do nothing

        Pose2d currentPose = m_drivetrain.getState().Pose;
        if (m_goForPID) m_pose = ReefTargetCalculator.calculateTargetTranslation(m_alignMode);
        if (m_pose == null) m_goForPID = false; // No reef button pressed – optionally mark the command as finished
        SmartDashboard.putBoolean("goForPID", m_goForPID);

        if (m_goForPID) {
            double currentAngle = currentPose.getRotation().getRadians();
            SmartDashboard.putNumber("CurrentAngle", currentAngle);

            double targetAngle;
            // For ALGAE mode, use the preset rotation; otherwise, compute the angle from
            // the target translation.
            if (m_alignMode == AlignMode.ALGAE) { // m_pose only has rotation populated
                targetAngle = m_pose.getRotation().getRadians();

                // Override the high/low algae buttons when in auto rotate so that the state is based on
                // which coral button is pressed - either algae button will enable
                if (isAlgaeHigh(m_pose)) 
                    m_state = SSM.States.ALGAEHIGH;
                else
                    m_state = SSM.States.ALGAELOW;
            } else {
                Translation2d robotToGoal = m_pose.getTranslation().minus(currentPose.getTranslation());
                double[] target_array = { m_pose.getTranslation().getX(), m_pose.getTranslation().getY() };
                SmartDashboard.putNumberArray("Target", target_array);

                // If at L1, fix the rotation to the field at the selected coral angle plus/minus a fixed offset
                if (m_state == SSM.States.L1) {
                    if (m_alignMode == ReefTargetCalculator.AlignMode.LEFT) {
                      targetAngle = m_pose.getRotation().plus(Rotation2d.fromDegrees(10.0)).getRadians();
                    } else {
                      targetAngle = m_pose.getRotation().minus(Rotation2d.fromDegrees(10.0)).getRadians();
                    }
                } else targetAngle = robotToGoal.getAngle().getRadians();
            }

            SmartDashboard.putString("Align Mode", m_alignMode.toString());
            SmartDashboard.putNumber("Target Angle", targetAngle);
            Double DifferenceinAngle = targetAngle - currentAngle;
            SmartDashboard.putNumber("Difference In Angle", DifferenceinAngle);
            // Compute the PID controller output.
            rotationOutput = rotPID.calculate(currentAngle, targetAngle);
            SmartDashboard.putNumber("Rot Out", rotationOutput);

            if (m_alignMode != AlignMode.ALGAE) {
                // Adjust elevator based on distance (and evantually delta angle) from post
                dist = m_pose.getTranslation().getDistance(currentPose.getTranslation()); // Distance to post from robot
                Rotation2d theta = m_pose.getRotation().minus(currentPose.getRotation()); // Angle from coral wall
                                                                                          // normal
                SmartDashboard.putNumber("Distance to target", dist);
                SmartDashboard.putNumber("Angle to Post (deg)", theta.getRadians() * 180.0 / Math.PI);
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
            }
        } else {
            double rotCurveAdjustment = DriveCommands.adjustRotCurve(m_rotationSupplier.getAsDouble(), 0.7, 0.3);
            rotationOutput = DriveCommands.m_slewRot.calculate(
                    -m_rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_RATE * rotCurveAdjustment);
        }

        // Set the state
        if (m_state == SSM.States.BARGE && DriverStation.getStickAxis(0, 3)>= 0.25) {
            armOffset = -30.0;
        }
        m_SSM.setState(m_state, armOffset, elevatorOffset);

        // Set the distance for LEDs
        if (dist <= 0.766)
            m_LED.setLEDScore(true);
        else
            m_LED.setLEDScore(false);

        // Compute translation speeds from joystick inputs with a custom curve.
        double transAdjustment = adjustInputCurve(m_forwardBackSupplier.getAsDouble(),
                m_leftRightSupplier.getAsDouble(), 0.7, 0.3);
        double velocityX = DriveCommands.m_slewX
                .calculate(-m_forwardBackSupplier.getAsDouble() * DriveConstants.MAX_SPEED * .7 * transAdjustment);
        double velocityY = DriveCommands.m_slewY
                .calculate(-m_leftRightSupplier.getAsDouble() * DriveConstants.MAX_SPEED * .7 * transAdjustment);

        // Build and apply the CTRE swerve request.
        SwerveRequest request = baseRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationOutput);
        m_drivetrain.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        m_SSM.setState(States.LOADINGSTATION);
        m_LED.setLEDScore(false);
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

    private boolean isAlgaeHigh(Pose2d pos) {
        if ((pos.getRotation() == AutoAlignConstants.BlueAllianceConstants.kAAlgea) ||
                (pos.getRotation() == AutoAlignConstants.BlueAllianceConstants.kCAlgea) ||
                (pos.getRotation() == AutoAlignConstants.BlueAllianceConstants.kEAlgea) ||
                (pos.getRotation() == AutoAlignConstants.RedAllianceConstants.kAAlgea) ||
                (pos.getRotation() == AutoAlignConstants.RedAllianceConstants.kCAlgea) ||
                (pos.getRotation() == AutoAlignConstants.RedAllianceConstants.kEAlgea))
            return true;
        return false;
    }
}
