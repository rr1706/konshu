package frc.robot;

import java.lang.Thread.State;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.*;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeFromFunnel;
import frc.robot.commands.PIDRotateToTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArm;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FunnelMotor;
import frc.robot.subsystems.LED;
//import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SSM;
import frc.robot.subsystems.SSM.States;
import frc.robot.utilities.ReefTargetCalculator;
import frc.robot.utilities.ReefTargetCalculator.AlignMode;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Climber;

// import frc.robot.subsystems.PoseEstimator;

public class RobotContainer {
    private final CommandXboxController driverController = 
    new CommandXboxController(DriveConstants.OperatorConstants.DRIVER_CONTROLLER_PORT);
    //  drivetrain;
    private final Elevator m_elevator = new Elevator();
    private final Arm m_arm = new Arm();
    private final FunnelMotor m_FunnelMotor = new FunnelMotor();

    //private final Claw m_claw = new Claw();
    private final CommandGenericHID m_operatorBoard = new CommandGenericHID(1);
    private final CommandGenericHID m_operatorBoard2 = new CommandGenericHID(2);

    private final SSM m_SSM = new SSM(m_arm, m_elevator);        // Defaults to DISABLED - no action until trigger
    private final AlgaeIntake m_algaeintake = new AlgaeIntake();
    // private final SSM m_SSM = new SSM(m_arm, m_elevator, SSM.States.L1);   // Alternative constructor, starts moving to initial state given 
    public final CommandSwerveDrivetrain drivetrain;
    public final Climber m_climber = new Climber();
    public final CoralArm m_coralarm = new CoralArm();
    private final LED m_Led = new LED(m_coralarm);
    public final FunnelMotor m_Funnel = new FunnelMotor();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureNamedCommands();

        drivetrain = TunerConstants.createDrivetrain();
        autoChooser = AutoBuilder.buildAutoChooser("4 L4");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            DriveCommands.fieldOrientedDrive(
                drivetrain,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX()
            )
        );
    }

    private void configureTelemetry() {
    }



    private void configureBindings() {
        driverController.start().onTrue(DriveCommands.resetFieldOrientation(drivetrain));

        driverController.rightBumper().onTrue(new InstantCommand(() -> m_elevator.jogging(false)));
        driverController.leftBumper().onTrue(new InstantCommand(() -> m_elevator.jogging(true)));

        driverController.povUp().onTrue(new InstantCommand(() -> m_arm.jogging(false)));
        driverController.povDown().onTrue(new InstantCommand(() -> m_arm.jogging(true)));
        
        driverController.leftTrigger().onFalse(new InstantCommand(() -> m_FunnelMotor.runCoralIn(-.5)).alongWith(new IntakeFromFunnel(m_coralarm)));
        driverController.x().onTrue(new InstantCommand(() -> m_climber.prepClimb()));
        driverController.b().onTrue(new InstantCommand(() -> m_climber.Climb())).onFalse(new InstantCommand(()->m_climber.stop()));

        driverController.rightTrigger().whileTrue((m_coralarm.runCoralCmd(-0.7)))
            .onFalse(new InstantCommand(() -> m_FunnelMotor.runCoralIn(-.5)).alongWith(new IntakeFromFunnel(m_coralarm)));

        driverController.leftTrigger().whileTrue(new PIDRotateToTrajectory(
                    drivetrain,
                    () -> driverController.getLeftY(),
                    () -> driverController.getLeftX(),
                    () -> driverController.getRightX(),
                    m_SSM));
                
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("Score", 
                (new WaitCommand(.5))
                .andThen(m_coralarm.runCoralCmd(-0.7).withTimeout(.2)));
        NamedCommands.registerCommand("GoL4", 
            new InstantCommand(() -> m_SSM.setState(States.L4)));
        NamedCommands.registerCommand("LoadingStation",
            new InstantCommand(() -> m_SSM.setState(States.LOADINGSTATION))
            .andThen(new InstantCommand(() -> m_FunnelMotor.runCoralIn(-.5)).alongWith(new IntakeFromFunnel(m_coralarm))));
        NamedCommands.registerCommand("ClawLowSpeed",
            new InstantCommand(() -> m_coralarm.runCoralCmd(-0.7)));
    }

    /**
     *
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }

    public Command getTeleInitCommand(){
        return new InstantCommand(()->m_climber.setPosition(25.0));
    }
}
