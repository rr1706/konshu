package frc.robot.subsystems;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
    public static final ThriftyNova m_InNova = new ThriftyNova(13, MotorType.MINION);

    public Funnel() {
        m_InNova.setMaxCurrent(CurrentType.STATOR, 50.0);
        m_InNova.setMaxCurrent(CurrentType.SUPPLY, 40.0);
        m_InNova.setVoltageCompensation(0.0);
    }

    public void runCoralIn(double speed) {
        m_InNova.set(speed);
    }

    public void stop() {
        m_InNova.stopMotor();
    }

}
