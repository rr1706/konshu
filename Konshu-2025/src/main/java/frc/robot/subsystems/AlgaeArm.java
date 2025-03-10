package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
        private final TalonFX m_AlgaeFX = new TalonFX(12);
        public AlgaeArm() {
            m_AlgaeFX.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true));
            m_AlgaeFX.setNeutralMode(NeutralModeValue.Brake);
            m_AlgaeFX.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));                 
        }
    
        public void runAlgae(double speed) {
            m_AlgaeFX.set(speed);
        }

        public Command spitAlgae(){
            return startEnd(()->runAlgae(-1.0),()->stop());
        }

        public Command grabAlgae(double speed){
            return run(()->runAlgae(speed));
        }
    
        public void stop() {
            m_AlgaeFX.stopMotor();
        }
    
        public double getAlgaeRollerStatorCurrent() {
            return m_AlgaeFX.getStatorCurrent().getValueAsDouble();
        }

        // TODO: need to verify current value
        public boolean haveAlgae() {
            return (getAlgaeRollerStatorCurrent() > 30);
        }

        // public boolean haveAlgae() {
        //     Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
        //     if (m_debouncer.calculate(getAlgaeRollerStatorCurrent() > 30)) return true;
        //     return false;
        // }
    }


