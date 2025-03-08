package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;

public class HealthCheck extends SubsystemBase {
    private final Arm m_arm = new Arm();

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator 9 Stator Current", getCurrent());
        SmartDashboard.putNumber("Elevator 10 Stator Current", getPosition());
        SmartDashboard.putNumber("Arm Stator Current", getCurrent());
        SmartDashboard.putNumber("Claw Stator Current", m_setpoint);
        SmartDashboard.putNumber("Drive 1 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Drive 2 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Drive 3 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Drive 4 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Drive 5 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Drive 6 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Drive 7 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Drive 8 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Algae on Arm Stator Current", m_setpoint);
        SmartDashboard.putNumber("Coral on Arm Stator Current", m_setpoint);
        SmartDashboard.putNumber("Funnel Stator Current", m_setpoint);
        SmartDashboard.putNumber("Algae Intake Stator Current", m_setpoint);
        SmartDashboard.putNumber("Claw Stator Current", m_setpoint);
        SmartDashboard.putNumber("Climber 17 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Climber 18 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Climber 19 Stator Current", m_setpoint);
        SmartDashboard.putNumber("Climber 20 Stator Current", m_setpoint);


    }
}
