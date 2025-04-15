package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class Climb extends Command {

    private final Climber m_climber;

    public Climb(Climber climber){
        m_climber = climber;
        addRequirements(climber);
    }
    
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
        m_climber.setVoltage(-5.0);
        m_climber.stopMinion();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        if(m_climber.getPose() < 16.0){
            m_climber.setVoltage(-3.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        m_climber.stop();
        m_climber.stopMinion();
    }
}
