// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.SSM;
import frc.robot.subsystems.AlgaeIntake;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeCommand extends Command {
  private final AlgaeArm m_algaeArm;
  private final AlgaeIntake m_AlgaeIntake;
  private final SSM m_ssm;
  /** Creates a new AlgaeIntake. */
  public AlgaeIntakeCommand(AlgaeArm algaeArm, AlgaeIntake AlgaeIntake, SSM ssm) {
    m_algaeArm = algaeArm;
    m_AlgaeIntake = AlgaeIntake;
    m_ssm = ssm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_algaeArm, m_AlgaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_algaeArm.runAlgae(1.0);
    m_AlgaeIntake.setPosition(16.0);
    m_AlgaeIntake.setRollers(-0.6);
    m_ssm.setState(SSM.States.PROCESSOR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



