package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelMotor extends SubsystemBase {
    private final TalonFX m_CoralFX = new TalonFX(13);
    public FunnelMotor() {
        m_CoralFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(false));

        m_CoralFX.setNeutralMode(NeutralModeValue.Brake);
        m_CoralFX.setInverted(true);               
    }

    public void runCoralIn(double speed) {
        m_CoralFX.set(speed);
    }
}
