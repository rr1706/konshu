package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
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
    public final static TalonFX m_ElevatorRightFX = new TalonFX(9, "*");
    public final static TalonFX m_ElevatorLeftFX = new TalonFX(10, "*");
    public double m_setPoint = getPosition();
    private SoftwareLimitSwitchConfigs m_limits;

    public Elevator() {
        m_ElevatorLeftFX.setControl(new Follower(9, true).withUpdateFreqHz(250));

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
            .withMotionMagicAcceleration(ElevatorConstants.kDownAccelerationElevator)
            .withMotionMagicCruiseVelocity(ElevatorConstants.kDownVelocityElevator)
            .withMotionMagicJerk(ElevatorConstants.kDownJerkElevator));
                    
        m_ElevatorRightFX.getConfigurator().apply( new Slot0Configs()
            .withKP(ElevatorConstants.kPElevator)
            .withKI(ElevatorConstants.kIElevator)
            .withKD(ElevatorConstants.kDElevator)
            .withKG(ElevatorConstants.kGElevator)
            .withKS(ElevatorConstants.kSElevator)
            .withKV(ElevatorConstants.kVElevator)
            .withKA(ElevatorConstants.kAElevator)
            .withGravityType(GravityTypeValue.Elevator_Static));
                        
        m_ElevatorRightFX.getConfigurator().apply(m_limits = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ElevatorConstants.kUpperLimitElevator/ElevatorConstants.kInchPerRotation)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ElevatorConstants.kULowerLimitElevator/ElevatorConstants.kInchPerRotation));

        m_ElevatorRightFX.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));                       
        m_ElevatorRightFX.setNeutralMode(NeutralModeValue.Brake);
        m_ElevatorLeftFX.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setPosition(double position){        // Position in inches

        m_setPoint = position;
        if (position > getPosition()) {     // Going up
            m_ElevatorRightFX.setControl(new DynamicMotionMagicVoltage(position/ElevatorConstants.kInchPerRotation, 
                ElevatorConstants.kUpVelocityElevator, ElevatorConstants.kUpAccelerationElevator, ElevatorConstants.kUpJerkElevator));
        } else {                            // Going down
            m_ElevatorRightFX.setControl(new DynamicMotionMagicVoltage(position/ElevatorConstants.kInchPerRotation, 
                ElevatorConstants.kDownVelocityElevator, ElevatorConstants.kDownAccelerationElevator, ElevatorConstants.kDownJerkElevator));
        }
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
    }

    public void enableLimits(boolean limitsState){
        m_limits.ReverseSoftLimitEnable = limitsState;
        m_limits.ForwardSoftLimitEnable = limitsState;
        m_ElevatorRightFX.getConfigurator().apply(m_limits);
    } 
    
    public void stop() {
        m_ElevatorRightFX.set(0);
    }

    public boolean atSetpoint(){
        return Math.abs(getPosition()-m_setPoint) <= 1.0;
    }

   public void jogging(boolean direction){
        if (direction == true){
            SmartDashboard.putNumber("Elevator Position", getPosition());
            setPosition(getPosition()+2.0*ElevatorConstants.kInchPerRotation);
        }
        else{
            SmartDashboard.putNumber("Elevator Position", getPosition());
            setPosition(getPosition()-2.0*ElevatorConstants.kInchPerRotation);
        }
    }

    @Override
    public void periodic() {
        // if getPosition() < 
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Left Elevator Current", getCurrentLeft().getValueAsDouble());
        SmartDashboard.putNumber("Right Elevator Current", getCurrentRight().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Set Point", m_setPoint);
    }
}
