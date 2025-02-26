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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SSM;
import frc.robot.subsystems.SSM.States;
// import frc.robot.subsystems.PoseEstimator;

public class RobotContainer {
    private final CommandXboxController driverController = 
        new CommandXboxController(DriveConstants.OperatorConstants.DRIVER_CONTROLLER_PORT);
    //  drivetrain;
    private final Elevator m_elevator = new Elevator();
    private final Arm m_arm = new Arm();
    private final Claw m_claw = new Claw();
    private final CommandGenericHID m_operatorBoard = new CommandGenericHID(1);
    private final SSM m_SSM2 = new SSM(m_arm, m_elevator);        // Defaults to DISABLED - no action until trigger
    // private final SSM2 m_SSM2 = new SSM2(m_arm, m_elevator, SSM2.States.L1);   // Alternative constructor, starts moving to initial state given
    private SSM.States m_queuedState = SSM.States.DISABLED;   
    public final CommandSwerveDrivetrain drivetrain;

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

        // driverController.rightBumper().onTrue(new InstantCommand(() -> m_elevator.jogging(false)));
        driverController.leftBumper().onTrue(new InstantCommand(() -> m_elevator.jogging(true)));

        driverController.povUp().onTrue(new InstantCommand(() -> m_arm.jogging(true)));
        driverController.povDown().onTrue(new InstantCommand(() -> m_arm.jogging(false)));
        driverController.povLeft().onTrue(new InstantCommand(() -> m_claw.runCoralIn(.7)).andThen(new InstantCommand(() ->m_arm.setPosition(ArmConstants.kArmFeeder))));
        driverController.povRight().onTrue(new InstantCommand(() -> m_claw.runCoralOut(.7)))
                                    .onFalse(new InstantCommand(() -> m_claw.stopBoth()));
        // driverController.a().onTrue(new InstantCommand(() -> m_claw.runAlgaeIn(.7)));
        // driverController.x().onTrue(drivetrain.setControl(() -> PositionVoltage(6)));
        
        driverController.b().onTrue(new InstantCommand(() -> m_claw.runAlgaeIn(-.7)));
        driverController.a().onTrue(new InstantCommand(() -> m_claw.runCoralIn(.7)).andThen(new InstantCommand(() -> m_SSM2.setState(States.LOADINGSTATION))))
            .onFalse(new InstantCommand(() -> m_claw.runCoralIn(0.2))); 

        driverController.rightBumper().onTrue(new InstantCommand(() -> m_claw.runAlgaeIn(.7))
            .andThen(new InstantCommand(() -> m_SSM2.setState(States.GROUNDALGAE))));
 
        driverController.rightTrigger().onTrue(new InstantCommand(() -> m_claw.runAlgaeIn(-.7))
            .andThen(new InstantCommand(() ->  m_claw.runCoralOut(1.0))));

        // Set new queued state to be used in the next SSM2 periodic call
        // driverController.leftTrigger().onTrue((new InstantCommand(() -> m_SSM2.setState(m_queuedState)))); 

// 2DO: See if this works to revert to L1 when trigger is released
        // Set new queued state to be used in the next SSM2 periodic call, revert to stow on trigger release


        // Store the button push so it's ready to use when the trigger is pulled
        // m_operatorBoard.button(12).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.L1)));
        // m_operatorBoard.button(9).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.L2)));
        // m_operatorBoard.button(6).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.L3)));
        // m_operatorBoard.button(3).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.L4)));
        // m_operatorBoard.button(1).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.LOADINGSTATION)));
        // m_operatorBoard.button(4).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.PROCESSOR)));
        // m_operatorBoard.button(5).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.BARGE)));


// 2DO: See if this works to change state when button pushed AND trigger is held

        // m_operatorBoard.button(12).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.L1)
        //     .driverController.leftTrigger().onTrue((new InstantCommand(() -> m_SSM2.setState(SSM2.States.L1))))));
        // Create a trigger for when the left trigger is held down
        // Trigger leftTriggerHeld = new Trigger(() -> driverController.leftTrigger().getAsBoolean());

        // leftTriggerHeld.onFalse((new InstantCommand(() -> m_SSM2.setState(States.LOADINGSTATION))));
        // .onFalse((new InstantCommand(() -> m_SSM2.setState(SSM2.States.L1))));  

        // Create a trigger for another button press while the left trigger is held down
        m_operatorBoard.button(12)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM2.setState(SSM.States.L1)));
        m_operatorBoard.button(9)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM2.setState(SSM.States.L2)));
        m_operatorBoard.button(6)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM2.setState(SSM.States.L3)));
        m_operatorBoard.button(3)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM2.setState(SSM.States.L4)));
        m_operatorBoard.button(1)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM2.setState(SSM.States.LOADINGSTATION)));
        m_operatorBoard.button(4)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM2.setState(SSM.States.PROCESSOR)));
        m_operatorBoard.button(5)
            .and(driverController.leftTrigger()) // This ensures both conditions must be met
            .onTrue(new InstantCommand(() -> m_SSM2.setState(SSM.States.BARGE)));

        driverController.leftTrigger().onFalse(new InstantCommand(() -> m_SSM2.setState(SSM.States.LOADINGSTATION)));
        // m_operatorBoard.button(9).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.L2)))
        //     .driverController.leftTrigger().onTrue((new InstantCommand(() -> m_SSM2.setState(SSM2.States.L2))));
        // m_operatorBoard.button(6).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.L3)))
        //     .driverController.leftTrigger().onTrue((new InstantCommand(() -> m_SSM2.setState(SSM2.States.L3))));
        // m_operatorBoard.button(3).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.L4)))
        //     .driverController.leftTrigger().onTrue((new InstantCommand(() -> m_SSM2.setState(SSM2.States.L4))));
        // m_operatorBoard.button(1).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.LOADINGSTATION)))
        //     .driverController.leftTrigger().onTrue((new InstantCommand(() -> m_SSM2.setState(SSM2.States.LOADINGSTATION))));
        // m_operatorBoard.button(4).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.PROCESSOR)))
        //     .driverController.leftTrigger().onTrue((new InstantCommand(() -> m_SSM2.setState(SSM2.States.PROCESSOR))));
        // m_operatorBoard.button(5).onTrue(new InstantCommand(() -> setQueuedState(SSM2.States.BARGE)))
        //     .driverController.leftTrigger()onTrue((new InstantCommand(() -> m_SSM2.setState(SSM2.States.BARGE))));

    }


    public void configureNamedCommands() {


        NamedCommands.registerCommand("Score", new InstantCommand(() -> m_SSM2.setState(States.L4)).andThen(new WaitCommand(0.9)).andThen((new InstantCommand(() -> m_claw.runCoralOut(.7)))));
        NamedCommands.registerCommand("LoadingStation",new InstantCommand(() -> m_claw.runCoralIn(.7)).andThen(new InstantCommand(() -> m_SSM2.setState(States.LOADINGSTATION))));
        NamedCommands.registerCommand("ClawLowSpeed",new InstantCommand(() -> m_claw.runCoralIn(.2)));

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
