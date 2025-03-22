package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class AlgaeIntakeEndCommand extends Command{
    private final AlgaeIntake m_AlgaeIntake;
    private Timer m_time = new Timer();
    private boolean m_finshed = false;
    /** Creates a new AlgaeIntake. */
    public AlgaeIntakeEndCommand(AlgaeIntake AlgaeIntake) {
      m_AlgaeIntake = AlgaeIntake;
    
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_AlgaeIntake);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_AlgaeIntake.setPosition(1.0);
      m_AlgaeIntake.setRollers(0.3);
      m_time.reset();
      m_time.start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (m_time.get() > 0.5) {
        m_AlgaeIntake.stop();
        m_finshed = true;
      }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return m_finshed;
    }
  }
