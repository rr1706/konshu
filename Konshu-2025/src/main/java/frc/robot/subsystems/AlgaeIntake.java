package frc.robot.subsystems;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;


public class AlgaeIntake extends SubsystemBase{
    private final TalonFX m_AlgaeIntakeFX = new TalonFX(15);
    public double m_setPoint = getPosition();
    private double m_ffvolts = 0;
    private SoftwareLimitSwitchConfigs m_limits;

    
    public AlgaeIntake() {

        m_AlgaeIntakeFX.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(IntakeConstants.kStatorCurrent)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(IntakeConstants.kSupplyCurrent)
        .withSupplyCurrentLimitEnable(true));

        m_AlgaeIntakeFX.getConfigurator().apply(new MotionMagicConfigs()
        .withMotionMagicAcceleration(IntakeConstants.kAccelerationAglaeIntake)
        .withMotionMagicCruiseVelocity(IntakeConstants.kVelocityAglaeIntake));
    
        m_AlgaeIntakeFX.getConfigurator().apply( new Slot0Configs()
        .withKP(IntakeConstants.kPAlgaeIntake)
        .withKI(IntakeConstants.kIAlgaeIntake)
        .withKD(IntakeConstants.kDAlgaeIntake)
        .withKG(IntakeConstants.kGAlgaeIntake)
        .withKS(IntakeConstants.kSAlgaeIntake)
        .withKV(IntakeConstants.kVAlgaeIntake)
        .withKA(IntakeConstants.kAAlgaeIntake));

        m_AlgaeIntakeFX.getConfigurator().apply(m_limits = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(IntakeConstants.kUpperLimit/ElevatorConstants.kInchPerRotation)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(IntakeConstants.kLowerLimit/ElevatorConstants.kInchPerRotation));

        m_AlgaeIntakeFX.setNeutralMode(NeutralModeValue.Brake);
                                        
    }
    


    public double getPosition() {      //make own inch per rotation for intake
        return (ElevatorConstants.kInchPerRotation*m_AlgaeIntakeFX.getPosition().getValueAsDouble());

        }
    public void setPosition(double position){        
        m_setPoint = position;
        m_AlgaeIntakeFX.setControl(new MotionMagicVoltage(position/ElevatorConstants.kInchPerRotation).withFeedForward(m_ffvolts).withOverrideBrakeDurNeutral(true));
    }
    public void enableLimits(boolean limitsState){
        m_limits.ReverseSoftLimitEnable = limitsState;
        m_limits.ForwardSoftLimitEnable = limitsState;
        m_AlgaeIntakeFX.getConfigurator().apply(m_limits);
    } 
    
    public void stop() {
        m_AlgaeIntakeFX.set(0);
    }

    @SuppressWarnings("rawtypes")
    public StatusSignal getCurrent() {
        return (m_AlgaeIntakeFX.getSupplyCurrent());
    }
   /*public void jogging(boolean direction){
        if (direction == true){
            setPosition(getPosition()+0.5*ElevatorConstants.kInchPerRotation);
        }
        else{
            setPosition(getPosition()-0.5*ElevatorConstants.kInchPerRotation);
        }

    }*/

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Intake Position", getPosition());
        SmartDashboard.putNumber("Algae Intake Current", getCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Algae Intake Set Point", m_setPoint);
    }
    }

