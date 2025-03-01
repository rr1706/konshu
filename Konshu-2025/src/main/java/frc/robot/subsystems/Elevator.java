package frc.robot.subsystems;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    public final static TalonFX m_ElevatorRightFX = new TalonFX(9, "Drivetrain");
        public final static TalonFX m_ElevatorLeftFX = new TalonFX(10, "Drivetrain");
                public double m_setPoint = getPosition();
                private double m_ffvolts = 0;
                private SoftwareLimitSwitchConfigs m_limits;

                public Elevator() {
                    m_ElevatorLeftFX.setControl(new Follower(9, true).withUpdateFreqHz(100));

                    m_ElevatorRightFX.getConfigurator().apply(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(ElevatorConstants.ElevatorCurrents.kStatorCurrent)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(ElevatorConstants.ElevatorCurrents.kSupplyCurrent)
                        .withSupplyCurrentLimitEnable(true));

                    m_ElevatorLeftFX.getConfigurator().apply(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(ElevatorConstants.ElevatorCurrents.kStatorCurrent)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(ElevatorConstants.ElevatorCurrents.kSupplyCurrent)
                        .withSupplyCurrentLimitEnable(true));
            
                    m_ElevatorRightFX.getConfigurator().apply(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(ElevatorConstants.kAccelerationElevator)
                        .withMotionMagicCruiseVelocity(ElevatorConstants.kVelocityElevator)
                        .withMotionMagicJerk(ElevatorConstants.kJerkElevator));

                    
                    m_ElevatorRightFX.getConfigurator().apply( new Slot0Configs()
                        .withKP(ElevatorConstants.kPElevator)
                        .withKI(ElevatorConstants.kIElevator)
                        .withKD(ElevatorConstants.kDElevator)
                        .withKG(ElevatorConstants.kGElevator)
                        .withKS(ElevatorConstants.kSElevator)
                        .withKV(ElevatorConstants.kVElevator)
                        .withKA(ElevatorConstants.kAElevator)
                        .withGravityType(GravityTypeValue.Elevator_Static));

                    // Steve:  Added divide by InchPerRotation
                    m_ElevatorRightFX.getConfigurator().apply(m_limits = new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ElevatorConstants.kUpperLimitElevator/ElevatorConstants.kInchPerRotation)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(ElevatorConstants.kULowerLimitElevator/ElevatorConstants.kInchPerRotation));

                    m_ElevatorRightFX.setNeutralMode(NeutralModeValue.Brake);

                    m_ElevatorRightFX.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));                       
                }
    public void setPosition(double position){        // Position in inches
// 2DO: Need to set m_ffvolts based on position and what we are carrying
        m_setPoint = position;
        m_ElevatorRightFX.setControl(new MotionMagicVoltage(position/ElevatorConstants.kInchPerRotation).withFeedForward(m_ffvolts).withOverrideBrakeDurNeutral(true));
        
    }


    @SuppressWarnings("rawtypes")
    public StatusSignal getCurrentLeft() {
        return (m_ElevatorLeftFX.getSupplyCurrent());
    }

    @SuppressWarnings("rawtypes")
    public StatusSignal getCurrentRight() {
        return (m_ElevatorRightFX.getSupplyCurrent());
    }

    public double getPosition() {      // Returns inches
        return (ElevatorConstants.kInchPerRotation*m_ElevatorRightFX.getPosition().getValueAsDouble());
                // ElevatorConstants.kElevatorGearRatio) * (2 * Math.PI * ElevatorConstants.kPulleyRadius)
    }

    public void enableLimits(boolean limitsState){
        m_limits.ReverseSoftLimitEnable = limitsState;
        m_limits.ForwardSoftLimitEnable = limitsState;
        m_ElevatorRightFX.getConfigurator().apply(m_limits);
    } 
    
    public void stop() {
        m_ElevatorRightFX.set(0);
    }

   public void jogging(boolean direction){
        if (direction == true){
            setPosition(getPosition()+0.5*ElevatorConstants.kInchPerRotation);
        }
        else{
            setPosition(getPosition()-0.5*ElevatorConstants.kInchPerRotation);
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Left Elevator Current", getCurrentLeft().getValueAsDouble());
        SmartDashboard.putNumber("Right Elevator Current", getCurrentRight().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Set Point", m_setPoint);
    }

}
