package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private final TalonFX m_AlgaeFX = new TalonFX(12);
    private final TalonFX m_CoralFX = new TalonFX(13);
    public Claw() {
        m_AlgaeFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(false));
        m_AlgaeFX.setNeutralMode(NeutralModeValue.Brake);
        m_AlgaeFX.setInverted(false);       

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

    public void runCoralOut(double speed) {
        m_CoralFX.set(-speed);
    }

    public void runAlgaeIn(double speed) {
        m_AlgaeFX.set(speed);
    }

    public void runAlgaeOut(double speed) {
        m_AlgaeFX.set(speed);
    }

    public void stopBoth() {
        m_AlgaeFX.set(0);
        m_CoralFX.set(0);
    }

    public void stopAlgaeRollers() {
        m_AlgaeFX.set(0);
    }

    public StatusSignal getAlgaeRollerStatorCurrent() {
        return m_AlgaeFX.getStatorCurrent();
    }
}