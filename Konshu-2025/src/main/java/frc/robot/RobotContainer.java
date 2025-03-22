package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.*;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeIntakeEndCommand;
import frc.robot.commands.AlignInAuto;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeFromFunnel;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArm;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.SSM;
import frc.robot.subsystems.SSM.States;
import frc.robot.subsystems.Climber;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(
            DriveConstants.OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandGenericHID operatorcontoller1 = new CommandGenericHID(1);
    private final CommandGenericHID operatorcontoller2 = new CommandGenericHID(2);

    public final CoralArm m_coralArm = new CoralArm();
    public final AlgaeArm m_algaeArm = new AlgaeArm();
    private final Elevator m_elevator = new Elevator(m_coralArm::haveCoral, m_algaeArm::haveAlgae);
    private final Arm m_arm = new Arm();
    private final Funnel m_funnel = new Funnel();

    private final SSM m_SSM = new SSM(m_arm, m_elevator, m_coralArm::haveCoral);
    public final CommandSwerveDrivetrain m_drivetrain;
    public final Climber m_climber = new Climber();

    public final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();

    private final LED m_LED = new LED(m_coralArm, m_algaeArm);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        m_drivetrain = TunerConstants.createDrivetrain();

        configureNamedCommands();
        configureAutoBuilder();

        autoChooser = AutoBuilder.buildAutoChooser("4 L4");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureDefaultCommands();
        configureBindings();
    }

    private void configureDefaultCommands() {
        m_drivetrain.setDefaultCommand(
                DriveCommands.fieldOrientedDrive(
                        m_drivetrain,
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> driverController.getRightX()));
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> m_drivetrain.getState().Pose, // Supplier of current robot pose
                    m_drivetrain::resetPose, // Consumer for seeding pose against auto

                    () -> m_drivetrain.getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> m_drivetrain.setControl(
                            m_drivetrain.m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    m_drivetrain // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    private void configureBindings() {
        driverController.start().onTrue(DriveCommands.resetFieldOrientation(m_drivetrain));

        driverController.leftBumper().whileTrue(new AlgaeIntakeCommand(m_algaeArm, m_AlgaeIntake, m_SSM))
                .onFalse(new AlgaeIntakeEndCommand(m_AlgaeIntake));

        driverController.povUp().onTrue(new InstantCommand(() -> m_arm.jogging(false)));
        driverController.povDown().onTrue(new InstantCommand(() -> m_arm.jogging(true)));

        driverController.leftTrigger().onFalse(
                new InstantCommand(() -> m_funnel.runCoralIn(-.5)).alongWith(new IntakeFromFunnel(m_coralArm)));
        driverController.x().onTrue(new InstantCommand(() -> m_climber.prepClimb()).alongWith(new InstantCommand(()->m_funnel.runCoralIn(0.0))));
        driverController.b().onTrue(new InstantCommand(() -> m_climber.Climb()))
                .onFalse(new InstantCommand(() -> m_climber.stop()));

        // driverController.rightTrigger()
        // .onTrue(new ConditionalCommand(new
        // WaitCommand(0.030).andThen(m_algaeArm.spitAlgae()),
        // new ConditionalCommand(m_algaeArm.slowSpitAlgae(),
        // m_coralArm.runCoralCmd(-0.35), () -> {
        // return m_SSM.getState() == (SSM.States.PROCESSOR);
        // }), () -> {
        // return m_SSM.getState() == (SSM.States.BARGE);
        // }))

        // .onFalse(
        // new InstantCommand(() -> m_funnel.runCoralIn(-.4)).alongWith(new
        // IntakeFromFunnel(m_coralArm)));

        driverController.rightTrigger()
                .onTrue(new ConditionalCommand(new WaitCommand(0.030).andThen(m_algaeArm.spitAlgae()),
                        new ConditionalCommand(m_algaeArm.slowSpitAlgae(),
                                new ConditionalCommand(m_coralArm.runCoralCmd(-0.45), m_coralArm.runCoralCmd(-0.35),
                                        () -> {
                                            return m_SSM.getState() == (SSM.States.L1);
                                        }),
                                () -> {
                                    return m_SSM.getState() == (SSM.States.PROCESSOR);
                                }),
                        () -> {
                            return m_SSM.getState() == (SSM.States.BARGE);
                        }))
                .onFalse(
                        new InstantCommand(() -> m_funnel.runCoralIn(-.4)).alongWith(new IntakeFromFunnel(m_coralArm)));

        driverController.a().onTrue(m_algaeArm.grabAlgae(0.8));
        operatorcontoller1.button(9).onTrue(m_algaeArm.grabAlgae(0.8));
        operatorcontoller2.button(1).onTrue(m_algaeArm.grabAlgae(0.8));

        operatorcontoller2.button(2).whileTrue(new InstantCommand(() -> m_funnel.runCoralIn(.2)))
                .onFalse(
                        new InstantCommand(() -> m_funnel.runCoralIn(-.4)).alongWith(new IntakeFromFunnel(m_coralArm)));

        operatorcontoller2.button(11).whileTrue(m_algaeArm.grabAlgae(.8)
                .alongWith(new InstantCommand(() -> m_SSM.setState(States.PROCESSOR))))
                .onFalse(new InstantCommand(() -> m_SSM.setState(States.LOADINGSTATION)));
        operatorcontoller2.button(12).whileTrue(m_algaeArm.spitAlgae());
        driverController.leftTrigger().whileTrue(new AutoAlign(
                m_drivetrain,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX(),
                m_SSM, m_LED));

    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("MoveClimberOut", new InstantCommand(() -> m_climber.setPosition(27.0)));
        NamedCommands.registerCommand("ScoreL4",
                (new WaitCommand(.8))
                        .andThen(m_coralArm.runCoralCmd(-0.35).withTimeout(.2)));
        NamedCommands.registerCommand("ScoreL4Fast",
                (new WaitCommand(.4))
                        .andThen(m_coralArm.runCoralCmd(-0.35).withTimeout(.2)));
        NamedCommands.registerCommand("GoL4",
                new InstantCommand(() -> m_SSM.setState(States.L4)));
        NamedCommands.registerCommand("GoLoadingStationPOS",
                new InstantCommand(() -> m_SSM.setState(States.LOADINGSTATION)));
        NamedCommands.registerCommand("Run Funnel",
                new InstantCommand(() -> m_funnel.runCoralIn(-.7)).alongWith(new IntakeFromFunnel(m_coralArm)));

        NamedCommands.registerCommand("AlignCRL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kCR;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kCR;
            }
        }, SSM.States.L4, m_SSM));

        NamedCommands.registerCommand("AlignCLL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kCL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kCL;
            }
        }, SSM.States.L4, m_SSM));

        NamedCommands.registerCommand("AlignBRL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kBR;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kBR;
            }
        }, SSM.States.L4, m_SSM));

        NamedCommands.registerCommand("AlignBLL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kBL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kBL;
            }
        }, SSM.States.L4, m_SSM));

        NamedCommands.registerCommand("AlignARL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kAR;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kAR;
            }
        }, SSM.States.L4, m_SSM));
        NamedCommands.registerCommand("AlignALL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kAL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kAL;
            }
        }, SSM.States.L4, m_SSM));

        NamedCommands.registerCommand("AlignFRL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kFR;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kFR;
            }
        }, SSM.States.L4, m_SSM));
        NamedCommands.registerCommand("AlignFLL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kFL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kFL;
            }
        }, SSM.States.L4, m_SSM));

        NamedCommands.registerCommand("AlignERL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kER;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kER;
            }
        }, SSM.States.L4, m_SSM));
        NamedCommands.registerCommand("AlignELL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kEL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kEL;
            }
        }, SSM.States.L4, m_SSM));

        NamedCommands.registerCommand("AlignDRL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kDR;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kDR;
            }
        }, SSM.States.L4, m_SSM));
        NamedCommands.registerCommand("AlignDLL4", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kDL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kDL;
            }
        }, SSM.States.L4, m_SSM));

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

    public Command getTeleInitCommand() {
        return new InstantCommand(() -> m_climber.setPosition(27.0));
    }
}
