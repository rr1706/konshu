package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final TalonFX m_armFX = new TalonFX(11, "rio");
    private SoftwareLimitSwitchConfigs m_limits;
    private Slot0Configs m_Slot0Configs;
    private CANcoder m_cancoder = new CANcoder(11, "rio");
    private double m_setpoint = getPosition();

    public Arm() {
        m_armFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ArmConstants.ArmCurrents.kStatorCurrent)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ArmConstants.ArmCurrents.kSupplyCurrent)
            .withSupplyCurrentLimitEnable(true));
           
        m_armFX.setNeutralMode(NeutralModeValue.Brake);

        m_limits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ArmConstants.kArmUpperLimit/360.0)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ArmConstants.kArmLowerLimit/360.0);
        m_armFX.getConfigurator().apply(m_limits);

        m_armFX.getConfigurator().apply(new MotionMagicConfigs()
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
        m_armFX.getConfigurator().apply(m_Slot0Configs);

        CANcoderConfiguration m_ccConfig = new CANcoderConfiguration();
            m_ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            m_ccConfig.MagnetSensor.MagnetOffset = ArmConstants.kMangentOffSet;
            m_cancoder.getConfigurator().apply(m_ccConfig);

        FeedbackConfigs m_feedbackConfig = new FeedbackConfigs();
            m_feedbackConfig.FeedbackRemoteSensorID = m_cancoder.getDeviceID();
            m_feedbackConfig.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            m_feedbackConfig.SensorToMechanismRatio = ArmConstants.kArmRotorToSensor;
            m_feedbackConfig.RotorToSensorRatio = ArmConstants.kArmGearRatio/ArmConstants.kArmRotorToSensor;
            m_armFX.getConfigurator().apply(m_feedbackConfig); 

        m_armFX.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    }

    // Get the position of the motor in degrees from hortizontal
    public double getPosition() {
        return (m_armFX.getPosition().getValueAsDouble() * 360.0);
    }

    // Set the position of the motor given position as an angle in degrees where 0 degrees is hortizontal
    public void setPosition(double pos) {
        m_armFX.setControl(new MotionMagicVoltage(pos / 360.0));
        m_setpoint = pos;
    }

    public double getCurrent() {
        return m_armFX.getStatorCurrent().getValueAsDouble();
    }

    public void stop() {
        m_armFX.set(0);
    }

    public void jogging(boolean direction) {
        double count = 0;
        if (direction) {
            setPosition(getPosition() + 2.0);
            count += 1.0;
            SmartDashboard.putNumber("Count", count);
        } else {
            setPosition(getPosition() - 2.0);
            count += 1.0;
            SmartDashboard.putNumber("Count", count);
        }

    }

    // Called by ZeroArm to move the arm toward the lower hard limit.
    public void zero() {
        m_armFX.set(-0.25);
    }

    // Called by ZeroArm when are at the lower limit. Zero the encoder, move to
    // horizontal (0 deg), and rezero the encoder
    public void setZero() {
        m_armFX.setPosition(0); // Zero the encoder
        setPosition(ArmConstants.kArmLowerLimit); // Move the arm to horizontal
        m_armFX.setPosition(0); // Rezero the encoder
    }

    // Set the soft limits based on a boolean parameter - used by ZeroArm
    public void enableLimits(boolean limitsState) {
        m_limits.ReverseSoftLimitEnable = limitsState;
        m_limits.ForwardSoftLimitEnable = limitsState;
        m_armFX.getConfigurator().apply(m_limits);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle - degrees", getPosition());
        SmartDashboard.putNumber("Arm Stator Current", getCurrent());
        SmartDashboard.putNumber("Arm Set Point", m_setpoint);
        
    }
}
