package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.*;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeFromFunnel;
import frc.robot.subsystems.AlgaeArm;
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
    private final CommandXboxController driverController = 
    new CommandXboxController(DriveConstants.OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final Elevator m_elevator = new Elevator();
    private final Arm m_arm = new Arm();
    private final Funnel m_funnel = new Funnel();

    private final SSM m_SSM = new SSM(m_arm, m_elevator);
    public final CommandSwerveDrivetrain drivetrain;
    public final Climber m_climber = new Climber();
    public final CoralArm m_coralArm = new CoralArm();
    public final AlgaeArm m_algaeArm = new AlgaeArm();

    private final LED m_Led = new LED(m_coralArm, m_algaeArm);

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

    private void configureBindings() {
        driverController.start().onTrue(DriveCommands.resetFieldOrientation(drivetrain));

        driverController.rightBumper().onTrue(new InstantCommand(() -> m_elevator.jogging(false)));
        driverController.leftBumper().onTrue(new InstantCommand(() -> m_elevator.jogging(true)));

        driverController.povUp().onTrue(new InstantCommand(() -> m_arm.jogging(false)));
        driverController.povDown().onTrue(new InstantCommand(() -> m_arm.jogging(true)));
        
        driverController.leftTrigger().onFalse(new InstantCommand(() -> m_funnel.runCoralIn(-.7)).alongWith(new IntakeFromFunnel(m_coralArm)));
        driverController.x().onTrue(new InstantCommand(() -> m_climber.prepClimb()));
        driverController.b().onTrue(new InstantCommand(() -> m_climber.Climb())).onFalse(new InstantCommand(()->m_climber.stop()));

        driverController.rightTrigger().onTrue(new ConditionalCommand(m_algaeArm.spitAlgae(),m_coralArm.runCoralCmd(-0.7),()->{
            return m_SSM.getState() == (SSM.States.BARGE) || m_SSM.getState() == (SSM.States.PROCESSOR);
        }))
            .onFalse(new InstantCommand(() -> m_funnel.runCoralIn(-.7)).alongWith(new IntakeFromFunnel(m_coralArm)));


        //for testing ONLY
        driverController.a().onTrue(m_algaeArm.grabAlgae(0.8));

        driverController.leftTrigger().whileTrue(new AutoAlign(
                    drivetrain,
                    () -> driverController.getLeftY(),
                    () -> driverController.getLeftX(),
                    () -> driverController.getRightX(),
                    m_SSM));
                
    }

    public void configureNamedCommands() {
        NamedCommands.registerCommand("ScoreL4", 
                (new WaitCommand(.35))
                .andThen(m_coralArm.runCoralCmd(-0.7).withTimeout(.3)));
        NamedCommands.registerCommand("GoL4", 
            new InstantCommand(() -> m_SSM.setState(States.L3)));
        NamedCommands.registerCommand("GoLoadingStationPOS",
            new InstantCommand(() -> m_SSM.setState(States.LOADINGSTATION)));
        NamedCommands.registerCommand("Run Funnel",
            new InstantCommand(() -> m_funnel.runCoralIn(-.7)).alongWith(new IntakeFromFunnel(m_coralArm)));

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
        return new InstantCommand(()->m_climber.setPosition(30.0));
    }
}
