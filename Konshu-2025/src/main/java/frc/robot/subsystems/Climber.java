package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.thethriftybot.ThriftyNova;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class Climber  extends SubsystemBase{
    public static final TalonFX m_climberLeftFX = new TalonFX(17);
    public static final TalonFX m_climberRightFx = new TalonFX(18);
    public double m_setpoint = getPosition();
    public static final ThriftyNova m_NovaLeft = new ThriftyNova(19);
    public static final ThriftyNova m_NovaRight = new ThriftyNova(20);

public Climber() {
            m_climberLeftFX.setControl(new Follower(18, true).withUpdateFreqHz(100));

           m_climberRightFx.getConfigurator().apply(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(ClimberConstants.kStatorCurrent)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(ClimberConstants.kSupplyCurrent)
                        .withSupplyCurrentLimitEnable(true));

           m_climberRightFx.getConfigurator().apply(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(ClimberConstants.kStatorCurrent)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(ClimberConstants.kSupplyCurrent)
                        .withSupplyCurrentLimitEnable(true));

            m_NovaRight.setInversion(true);
            
        
    }

    public double getPosition() {
        return (m_climberLeftFX.getPosition().getValueAsDouble() * 360);
    
    }

        public void setPosition(double pos) {
        m_climberLeftFX.setControl(new MotionMagicVoltage(pos / 360.0));
        m_climberRightFx.setControl(new MotionMagicVoltage(pos / 360.0));
        m_setpoint = pos;
    }

    public void stop() {
        m_climberLeftFX.set(0);
        m_climberRightFx.set(0);
    }

     public void setZero() {
        m_climberLeftFX.setPosition(0); // Zero the encoder
        setPosition(ClimberConstants.kZeroPosition); // Move the arm to horizontal
        m_climberLeftFX.setPosition(0); // Rezero the encoder
        m_climberRightFx.setPosition(0); // Zero the encoder
        setPosition(ClimberConstants.kZeroPosition); // Move the arm to horizontal
        m_climberRightFx.setPosition(0); // Rezero the encoder
    }

    public double getCurrent() {
        return m_climberLeftFX.getStatorCurrent().getValueAsDouble();
    }


    public double prepClimb(double percent, double angle) {
        m_climberLeftFX.setPosition(angle);
        m_climberRightFx.setPosition(angle);
        m_NovaLeft.setPercent(percent);
        m_NovaRight.setPercent(percent);
                return prepClimb(percent, angle);
    }



    



    
}
