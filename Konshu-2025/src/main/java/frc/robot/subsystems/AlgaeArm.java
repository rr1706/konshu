package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArm extends SubsystemBase {
        private final TalonFX m_AlgaeFX = new TalonFX(12);
        public AlgaeArm() {
            m_AlgaeFX.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(50)
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
            return startEnd(()->runAlgae(-0.50),()->stop());
        }

        public Command spitAlgaeRev(){
            return startEnd(()->runAlgae(0.50),()->stop());
        }

        public Command slowSpitAlgae(){
            return startEnd(()->runAlgae(-0.5),()->stop());
        }


        public Command grabAlgae(double speed){
            return runOnce(()->runAlgae(speed));
        }
    
        public void stop() {
            m_AlgaeFX.stopMotor();
        }
    
        public double getAlgaeRollerStatorCurrent() {
            return m_AlgaeFX.getStatorCurrent().getValueAsDouble();
        }

        public boolean haveAlgae() {
            return (getAlgaeRollerStatorCurrent() > 35);
        }

        @Override
        public void periodic() {
            SmartDashboard.putNumber("Algae Current", m_AlgaeFX.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putBoolean("Algae Present", haveAlgae());
            SmartDashboard.putNumber("Algae Arm Volts Out", m_AlgaeFX.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Algae Temp", m_AlgaeFX.getAncillaryDeviceTemp().getValueAsDouble());
            
        }
    }


