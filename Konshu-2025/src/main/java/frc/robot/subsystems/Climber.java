package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClimberConstants;

public class Climber  extends SubsystemBase{
    // public static final TalonFX m_climberLeftFX = new TalonFX(17);
    public static final TalonFX m_climberRightFx = new TalonFX(18, "*");
   // public double m_setpoint = getPosition();
    public static final ThriftyNova m_NovaLeft = new ThriftyNova(19, MotorType.MINION);
    public static final ThriftyNova m_NovaRight = new ThriftyNova(20, MotorType.MINION);
    private Slot0Configs m_Slot0Configs;
    private SoftwareLimitSwitchConfigs m_limits;


public Climber() {
            // m_climberLeftFX.setControl(new Follower(18, false).withUpdateFreqHz(100));

    

        m_climberRightFx.getConfigurator().apply(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(ClimberConstants.kStatorCurrent)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(ClimberConstants.kSupplyCurrent)
                        .withSupplyCurrentLimitEnable(true));

                         m_limits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ClimberConstants.kUpperSoftLimit)
        .withReverseSoftLimitEnable(false)
        .withReverseSoftLimitThreshold(ClimberConstants.kLowerSoftLimit);
        m_climberRightFx.getConfigurator().apply(m_limits);

        m_climberRightFx.getConfigurator().apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ClimberConstants.kAcceleration)
                .withMotionMagicCruiseVelocity(ClimberConstants.kCruiseVelocity));
                
        m_Slot0Configs = new Slot0Configs()
                .withKG(0)
                .withKS(.25)
                .withKV(.12)
                .withKP(10)
                .withKI(0)
                .withKD(0)
                .withKA(0);
        m_climberRightFx.getConfigurator().apply(m_Slot0Configs);
        m_NovaRight.setInversion(true);
        m_climberRightFx.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        m_climberRightFx.setNeutralMode(NeutralModeValue.Coast);

      
    }

   // public double getPosition() {
   //     return (m_climberLeftFX.getPosition().getValueAsDouble() * ClimberConstants.kRotationGearRatio * 360);
    
  //  }

    public void deployPosition(double pos) {
        setPosition(ClimberConstants.kDeployPosition);
    }

    public void stop() {
        m_climberRightFx.stopMotor();
    }

    public void setPosition(double pos) {
        m_climberRightFx.setControl(new MotionMagicVoltage(pos));
    }
    public double getCurrent() {
        double right_current = m_climberRightFx.getStatorCurrent().getValueAsDouble();
        // double left_current = m_climberLeftFX.getStatorCurrent().getValueAsDouble();
        double climber_current = (right_current);
        return climber_current;
    }


    public void prepClimb() {
        setPosition(ClimberConstants.kClimbPosition);
        m_NovaLeft.setPercent(.5);
        m_NovaRight.setPercent(.5);
        // return prepClimb(percent, angle);
    }

    public void Climb() {
        m_climberRightFx.setVoltage(-4.0);
        m_NovaLeft.setPercent(0);
        m_NovaRight.setPercent(0);
        // return prepClimb(percent, angle);
    }

   
}
