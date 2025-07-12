package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
    private final TalonFX m_InFX = new TalonFX(13, "Rio");
    private final BooleanSupplier m_hasCoral;
    private final BooleanSupplier m_manReady;
    private final Debouncer m_debounceJam = new Debouncer(0.050);

    public Funnel(BooleanSupplier hasCoral, BooleanSupplier manipulatorReady) {
        m_InFX.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(false));

        m_InFX.setNeutralMode(NeutralModeValue.Brake);
        m_InFX.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        m_hasCoral = hasCoral;
        m_manReady = manipulatorReady;
    }

    public void runCoralIn(double volts) {
        m_InFX.setVoltage(volts);
    }

    public boolean coralJam(){
        return m_debounceJam.calculate(m_InFX.getStatorCurrent().getValueAsDouble() >= 25.0);
    }
    public Command runFunnelIfReady(double volts) {
        return runEnd(() -> {
            if(m_hasCoral.getAsBoolean() || !m_manReady.getAsBoolean()){
                stop();
            }
            else{
                runCoralIn(volts);
            }
        }, () -> stop());
    }

    public void stop() {
        m_InFX.stopMotor();
    }
}
