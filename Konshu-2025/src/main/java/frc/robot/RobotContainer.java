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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArm;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FunnelMotor;
//import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SSM;
import frc.robot.subsystems.SSM.States;
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
    private final SSM m_SSM = new SSM(m_arm, m_elevator);        // Defaults to DISABLED - no action until trigger
    private final AlgaeIntake m_algaeintake = new AlgaeIntake();
    // private final SSM m_SSM = new SSM(m_arm, m_elevator, SSM.States.L1);   // Alternative constructor, starts moving to initial state given 
    public final CommandSwerveDrivetrain drivetrain;
    public final Climber m_Climber = new Climber();
    public final CoralArm m_coralarm = new CoralArm();
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

        // driverController.povUp().onTrue(new InstantCommand(() -> m_algaeintake.jogging(true)));
        // driverController.povDown().onTrue(new InstantCommand(() -> m_algaeintake.jogging(false)));

        driverController.povUp().onTrue(new InstantCommand(() -> m_arm.jogging(false)));
        driverController.povDown().onTrue(new InstantCommand(() -> m_arm.jogging(true)));
        //driverController.povDown().onTrue(new InstantCommand(() -> m_arm.jogging(false)));
       // driverController.povLeft().onTrue(new InstantCommand(() -> m_claw.runCoralIn(.7)).andThen(new InstantCommand(() ->m_arm.setPosition(ArmConstants.kArmFeeder))));
       // driverController.povRight().onTrue(new InstantCommand(() -> m_claw.runCoralOut(.7)))
                                    //.onFalse(new InstantCommand(() -> m_claw.stopBoth()));
        // driverController.a().onTrue(new InstantCommand(() -> m_claw.runAlgaeIn(.7)));
        // driverController.x().onTrue(drivetrain.setControl(() -> PositionVoltage(6)));
        
       // driverController.b().onTrue(new InstantCommand(() -> m_claw.runAlgaeIn(-.7)));
        driverController.a().onFalse(new InstantCommand(() -> m_FunnelMotor.runCoralIn(-.7)).andThen(new IntakeFromFunnel(m_coralarm)).andThen(new InstantCommand(() -> m_SSM.setState(States.LOADINGSTATION))));
        //    .onFalse(new InstantCommand(() -> m_FunnelMotor.runCoralIn(-.7)).andThen(new IntakeFromFunnel(m_coralarm))); 

        //driverController.rightBumper().onTrue(new InstantCommand(() -> m_claw.runAlgaeIn(.7))
           // .andThen(new InstantCommand(() -> m_SSM.setState(States.GROUNDALGAE))));
 
        //driverController.rightTrigger().onTrue(new InstantCommand(() -> m_claw.runAlgaeIn(-.7))
            //.andThen(new InstantCommand(() ->  m_claw.runCoralOut(1.0))));
        driverController.rightTrigger().whileTrue((m_coralarm.runCoralCmd(-25.0)));
        // driverController.rightTrigger().onFalse(new InstantCommand(() -> m_coralarm.runCoral(0.0)));

        // Create a trigger for another button press while the left trigger is held down
        m_operatorBoard.button(12)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM.setState(SSM.States.L1)));
        m_operatorBoard.button(9)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM.setState(SSM.States.L2)));
        m_operatorBoard.button(6)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM.setState(SSM.States.L3)));
        m_operatorBoard.button(3)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM.setState(SSM.States.L4)));
        m_operatorBoard.button(1)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM.setState(SSM.States.LOADINGSTATION)));
        m_operatorBoard.button(4)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM.setState(SSM.States.PROCESSOR)));
        m_operatorBoard.button(5)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM.setState(SSM.States.BARGE)));
        m_operatorBoard.button(10)
            .and(driverController.pov(270)) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() ->m_Climber.prepClimb(.50*ClimberConstants.kWheelRotRatio, .25)));
        m_operatorBoard.button(11)
            .and(driverController.pov(270)) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() ->m_Climber.prepClimb(.50*ClimberConstants.kWheelRotRatio, .1)));
        
        // driverController.leftBumper().onTrue(new InstantCommand(() ->m_FunnelMotor.runCoralIn(-.7)).andThen(new IntakeFromFunnel(m_coralarm)));
        driverController.leftTrigger().onFalse(new InstantCommand(() -> m_SSM.setState(SSM.States.LOADINGSTATION)));
    }


    public void configureNamedCommands() {
    //     NamedCommands.registerCommand("Score", new InstantCommand(() -> m_SSM.setState(States.L4)).andThen(new WaitCommand(0.9)).andThen((new InstantCommand(() -> m_coralarm.runCoral(.7)))));
    //     NamedCommands.registerCommand("LoadingStation",new InstantCommand(() -> m_coralarm.runCoral(.7)).andThen(new InstantCommand(() -> m_SSM.setState(States.LOADINGSTATION))));
    //     NamedCommands.registerCommand("ClawLowSpeed",new InstantCommand(() -> m_coralarm.runCoral(.2)));
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


}
