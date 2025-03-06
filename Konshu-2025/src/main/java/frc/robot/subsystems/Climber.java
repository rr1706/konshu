package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.thethriftybot.ThriftyNova;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClimberConstants;

public class Climber  extends SubsystemBase{
    public static final TalonFX m_climberLeftFX = new TalonFX(17);
    public static final TalonFX m_climberRightFx = new TalonFX(18);
    public double m_setpoint = getPosition();
    public static final ThriftyNova m_NovaLeft = new ThriftyNova(19);
    public static final ThriftyNova m_NovaRight = new ThriftyNova(20);
    private Slot0Configs m_Slot0Configs;


public Climber() {
            m_climberLeftFX.setControl(new Follower(18, false).withUpdateFreqHz(100));

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

            m_climberRightFx.getConfigurator().apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ArmConstants.kArmAcceleration)
                .withMotionMagicCruiseVelocity(ArmConstants.kArmCruiseVelocity));
                // .withMotionMagicJerk(ArmConstants.kArmJerk));
                
                m_Slot0Configs = new Slot0Configs()
                .withKG(ArmConstants.kArmG)
                .withKS(ArmConstants.kArmS)
                .withKV(ArmConstants.kArmV)
                .withKP(ArmConstants.kArmP)
                .withKI(ArmConstants.kArmI)
                .withKD(ArmConstants.kArmD)
                .withKA(ArmConstants.kArmA)
                .withGravityType(GravityTypeValue.Arm_Cosine);
            m_climberRightFx.getConfigurator().apply(m_Slot0Configs);
            m_NovaRight.setInversion(true);
            
        
    }

    public double getPosition() {
        return (m_climberLeftFX.getPosition().getValueAsDouble() * ClimberConstants.kRotationGearRatio * 360);
    
    }

        public void setPosition(double pos) {
        m_climberRightFx.setControl(new MotionMagicVoltage(pos * ClimberConstants.kRotationGearRatio / 360.0));
        m_setpoint = pos;
    }

    public void stop() {
        // m_climberLeftFX.set(0);
        m_climberRightFx.set(0);
    }
// method used in zeroing command, otherwise not called
     public void setZero() {
        m_climberLeftFX.setPosition(0); // Zero the encoder
        setPosition(ClimberConstants.kZeroPosition); // Move the arm to horizontal
        m_climberLeftFX.setPosition(0); // Rezero the encoder
        m_climberRightFx.setPosition(0); // Zero the encoder
        setPosition(ClimberConstants.kZeroPosition); // Move the arm to horizontal
        m_climberRightFx.setPosition(0); // Rezero the encoder
    }

    public double getCurrent() {
        double right_current = m_climberRightFx.getStatorCurrent().getValueAsDouble();
        double left_current = m_climberLeftFX.getStatorCurrent().getValueAsDouble();
        double max = Math.max(right_current,left_current);
        return max;
    }


    public void prepClimb(double percent, double angle) {
        m_climberLeftFX.setPosition(angle*ClimberConstants.kRotationGearRatio);
        m_climberRightFx.setPosition(angle*ClimberConstants.kRotationGearRatio);
        m_NovaLeft.setPercent(percent*ClimberConstants.kWheelRotRatio);
        m_NovaRight.setPercent(percent*ClimberConstants.kWheelRotRatio);
        // return prepClimb(percent, angle);
    }



    



    
}
