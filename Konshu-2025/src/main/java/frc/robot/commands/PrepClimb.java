package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class PrepClimb extends Command {
    private final Climber m_climb;

    public PrepClimb(Climber climber){
        m_climb = climber;
        addRequirements(m_climb);
    }
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
    
    
}
