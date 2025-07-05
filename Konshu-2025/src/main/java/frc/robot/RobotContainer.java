package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.*;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeIntakeEndCommand;
import frc.robot.commands.AlignInAuto;
import frc.robot.commands.AlignInPath;
import frc.robot.commands.AlignToAngle;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeFromFunnel;
import frc.robot.commands.MoveIntakeToUnJam;
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
    private final Elevator m_elevator = new Elevator();
    private final Arm m_arm = new Arm();

    private final SSM m_SSM = new SSM(m_arm, m_elevator, m_coralArm::haveCoral);
    private final Funnel m_funnel = new Funnel(m_coralArm::haveCoral, () -> {
        if (m_SSM.getState() == SSM.States.LOADINGSTATION)
            return true;
        else
            return false;
    });

    public final CommandSwerveDrivetrain m_drivetrain;
    public final Climber m_climber = new Climber();
    public final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();
    private final LED m_LED = new LED(m_coralArm::haveCoral, m_algaeArm::haveAlgae);
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        m_drivetrain = TunerConstants.createDrivetrain();
        configureNamedCommands();
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

    private void configureBindings() {
        // new Trigger(m_coralArm::haveCoral).onTrue(new
        // InstantCommand(()->m_SSM.setState(States.L2)));

        driverController.start().onTrue(DriveCommands.resetFieldOrientation(m_drivetrain));

        driverController.leftBumper().whileTrue(new AlgaeIntakeCommand(m_algaeArm, m_AlgaeIntake, m_SSM))
                .onFalse(new AlgaeIntakeEndCommand(m_AlgaeIntake, m_SSM));
        driverController.rightBumper().whileTrue(new MoveIntakeToUnJam(m_AlgaeIntake, m_SSM))
                .onFalse(new AlgaeIntakeEndCommand(m_AlgaeIntake, m_SSM));

        driverController.povUp().onTrue(new InstantCommand(() -> m_arm.jogging(false)));
        driverController.povDown().onTrue(new InstantCommand(() -> m_arm.jogging(true)));

        driverController.leftTrigger().onFalse(
                new InstantCommand(() -> m_funnel.runCoralIn(-.25)).andThen(new IntakeFromFunnel(m_coralArm)));
        driverController.x().onTrue(new InstantCommand(() -> m_climber.prepClimb())
                .alongWith(new InstantCommand(() -> m_funnel.runCoralIn(0.0))));
        driverController.b().whileTrue(new Climb(m_climber));

        driverController.rightTrigger()
                .onTrue(new ConditionalCommand(new WaitCommand(0.110).andThen(m_algaeArm.spitAlgae()),
                        new ConditionalCommand(m_algaeArm.slowSpitAlgae(),
                                new ConditionalCommand(new ConditionalCommand(m_coralArm.runCoralCmd(-6.0),
                                        m_coralArm.runCoralCmd(-2.25), () -> {
                                            return operatorcontoller2.button(2).getAsBoolean();
                                        }), m_coralArm.runCoralCmd(-3.6),
                                        () -> {
                                            return m_SSM.getState() == (SSM.States.L1_IN);
                                        }),
                                () -> {
                                    return m_SSM.getState() == (SSM.States.PROCESSOR);
                                }),
                        () -> {
                            return m_SSM.getState() == SSM.States.BARGE || m_SSM.getState() == SSM.States.TOSS;
                        }))
                .onFalse(
                        new InstantCommand(() -> m_funnel.runCoralIn(-.25)).andThen(new IntakeFromFunnel(m_coralArm)));

        operatorcontoller1.button(9).onTrue(m_algaeArm.grabAlgae(0.8));
        operatorcontoller2.button(1).onTrue(m_algaeArm.grabAlgae(0.8));

        driverController.a().onTrue(new InstantCommand(() -> m_funnel.runCoralIn(.2)))
                .onFalse(
                        new InstantCommand(() -> m_funnel.runCoralIn(-0.25))
                                .andThen(new IntakeFromFunnel(m_coralArm)));

        operatorcontoller2.button(11).whileTrue(m_algaeArm.grabAlgae(.8)
                .alongWith(new InstantCommand(() -> m_SSM.setState(States.PROCESSOR))))
                .onFalse(new InstantCommand(() -> m_SSM.setState(States.LOADINGSTATION)));

        operatorcontoller2.button(12).whileTrue(m_algaeArm.grabAlgae(0.8)
                .alongWith(new InstantCommand(() -> m_SSM.ejectStuckAlgae())))
                .onFalse(m_algaeArm.grabAlgae(0)
                        .alongWith(new InstantCommand(() -> m_SSM.setState(States.LOADINGSTATION))));

        driverController.leftTrigger().whileTrue(new AutoAlign(
                m_drivetrain,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX(),
                m_SSM, m_LED));

    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("MoveClimberOut",
                new InstantCommand(() -> m_climber.setPosition(ClimberConstants.kDeployPosition)));
        NamedCommands.registerCommand("ScoreL4",
                (new WaitCommand(.6))
                        .andThen(m_coralArm.runCoralCmd(-4.0).withTimeout(.2)));
        NamedCommands.registerCommand("ScoreL4Fast",
                (new WaitCommand(.17))
                        .andThen(m_coralArm.runCoralCmd(-4.0).withTimeout(.2)));
        NamedCommands.registerCommand("ScoreL4Faster",
                (new WaitCommand(.07))
                        .andThen(m_coralArm.runCoralCmd(-4.0).withTimeout(.2)));
        NamedCommands.registerCommand("ScoreL4Final",
                (new WaitCommand(.17))
                        .andThen(m_coralArm.runCoralCmd(-4.0).withTimeout(.5)));
        NamedCommands.registerCommand("ScoreL2Fast",
                (new WaitCommand(.040))
                        .andThen(m_coralArm.runCoralCmd(-4.0).withTimeout(.2)));
        NamedCommands.registerCommand("GoL4",
                new InstantCommand(() -> m_SSM.setState(States.L4)));
        NamedCommands.registerCommand("GoL3",
        new InstantCommand(() -> m_SSM.setState(States.L3)));

        NamedCommands.registerCommand("GoL3BL", new AlignInPath(() -> m_drivetrain.getState().Pose, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kBL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kBL;
            }
        }, SSM.States.L3, m_SSM));

        NamedCommands.registerCommand("GoL4CR", new AlignInPath(() -> m_drivetrain.getState().Pose, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kCR;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kCR;
            }
        }, SSM.States.L4, m_SSM));

        NamedCommands.registerCommand("GoL4BL", new AlignInPath(() -> m_drivetrain.getState().Pose, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kBL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kBL;
            }
        }, SSM.States.L4, m_SSM));

        NamedCommands.registerCommand("GoL4BR", new AlignInPath(() -> m_drivetrain.getState().Pose, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kBR;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kBR;
            }
        }, SSM.States.L4, m_SSM));

        NamedCommands.registerCommand("GoL4AL", new AlignInPath(() -> m_drivetrain.getState().Pose, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kAL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kAL;
            }
        }, SSM.States.L4, m_SSM));
        NamedCommands.registerCommand("GoL1",
                new InstantCommand(() -> m_SSM.setState(States.L1)));
        NamedCommands.registerCommand("GoL2",
                new InstantCommand(() -> m_SSM.setState(States.L2)));
        NamedCommands.registerCommand("GoLoadingStationPOS",
                new InstantCommand(() -> m_SSM.setState(States.LOADINGSTATION)));
        NamedCommands.registerCommand("UseMT2", m_drivetrain.useMT2(true));
        NamedCommands.registerCommand("NoMT2", m_drivetrain.useMT2(false));
        NamedCommands.registerCommand("Run Funnel",
                new InstantCommand(() -> m_funnel.runCoralIn(-0.25)).alongWith(new IntakeFromFunnel(m_coralArm)));
        NamedCommands.registerCommand("PickUpAlgae", m_algaeArm.grabAlgae(.8));
        NamedCommands.registerCommand("ThrowAlgae",
                new InstantCommand(() -> m_SSM.setState(States.BARGE, -30.0, 0.0)).andThen(new WaitCommand(.090))
                        .andThen(m_algaeArm.spitAlgae().withTimeout(0.200))
                        .andThen(new InstantCommand(() -> m_SSM.setState(States.ALGAEHIGH))));
        NamedCommands.registerCommand("GoBarge",
                new InstantCommand(() -> m_SSM.setState(States.BARGE)));
        NamedCommands.registerCommand("GoAlgaeHigh",
                new InstantCommand(() -> m_SSM.setState(States.ALGAEHIGH)));
        NamedCommands.registerCommand("GoAlgaeLow",
                new InstantCommand(() -> m_SSM.setState(States.ALGAELOW)));

        NamedCommands.registerCommand("GrabBAlgae", new InstantCommand(()->m_SSM.setState(States.ALGAELOW)).andThen(new AlignToAngle(m_drivetrain,() -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kBAlgea;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kBAlgea;
            }
        },0.25,-0.15)).alongWith(m_algaeArm.grabAlgae(0.8)).withTimeout(1.0));

        NamedCommands.registerCommand("GrabAAlgae", new InstantCommand(()->m_SSM.setState(States.ALGAELOW)).andThen(new AlignToAngle(m_drivetrain,() -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kAAlgea;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kAAlgea;
            }
        },0.25,-0.15)).alongWith(m_algaeArm.grabAlgae(0.8)).withTimeout(1.0));

        NamedCommands.registerCommand("WaitForElevator2",
                new WaitUntilCommand(m_elevator::atSetpoint).withTimeout(0.5));
        NamedCommands.registerCommand("WaitForElevator",
                new WaitUntilCommand(m_elevator::atSetpoint).alongWith(new WaitCommand(0.100)).withTimeout(0.5));

        NamedCommands.registerCommand("ScorePerp", new InstantCommand(()->m_coralArm.runCoral(-4.0),m_coralArm));
        NamedCommands.registerCommand("Score", m_coralArm.runCoralCmd(-4.0).withTimeout(.2));


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

        NamedCommands.registerCommand("AlignBLL3", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kBL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kBL;
            }
        }, SSM.States.L3, m_SSM));

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
        NamedCommands.registerCommand("AlignARL2", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kAR;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kAR;
            }
        }, SSM.States.L2, m_SSM));
        NamedCommands.registerCommand("AlignALL2", new AlignInAuto(m_drivetrain, () -> {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Blue) {
                return AutoAlignConstants.BlueAllianceConstants.kAL;
            } else {
                return AutoAlignConstants.RedAllianceConstants.kAL;
            }
        }, SSM.States.L2, m_SSM));
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

    public void setSSMDisabled() {
        m_SSM.setState(States.DISABLED);
    }

    public Command getTeleInitCommand() {
        return new InstantCommand(() -> m_climber.setPosition(ClimberConstants.kDeployPosition))
                .andThen(m_drivetrain.useMT2(true)).andThen(m_funnel.runFunnelIfReady(-0.25).asProxy()
                        .alongWith(new IntakeFromFunnel(m_coralArm)).asProxy());
    }
}
