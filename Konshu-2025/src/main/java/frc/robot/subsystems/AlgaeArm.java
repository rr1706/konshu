package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
        private final TalonFX m_AlgaeFX = new TalonFX(12);
        public AlgaeArm() {
            m_AlgaeFX.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true));
            m_AlgaeFX.setNeutralMode(NeutralModeValue.Brake);
            m_AlgaeFX.setInverted(false);                    
        }
    
        public void runAlgae(double speed) {
            m_AlgaeFX.set(speed);
        }
    
        public void stopAlgae() {
            m_AlgaeFX.set(0);
        }
    
        public double getAlgaeRollerStatorCurrent() {
            return m_AlgaeFX.getStatorCurrent().getValueAsDouble();
        }
    }

