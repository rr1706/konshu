package frc.robot.subsystems;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.ctre.phoenix6.configs.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;


//Example usage of a TalonSRX motor controller
//TalonSRX motor = new TalonSRX(0); // creates a new TalonSRX with ID 0
//TalonSRXConfiguration config = new TalonSRXConfiguration();
//config.peakCurrentLimit = 40; // the peak current, in amps
//config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
//config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
//motor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder

//motor.set(TalonSRXControlMode.PercentOutput, 0.5); // runs the motor at 50% power

public class AlgaeIntake extends SubsystemBase{
    private final TalonFX m_algaeintakeFX = new TalonFX(15, "*");
    ThriftyNova m_algaeintakeSRX = new ThriftyNova(16);
    public double m_setPoint = getPosition();

    public AlgaeIntake() {
        m_algaeintakeFX.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(IntakeConstants.kStatorCurrent)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(IntakeConstants.kSupplyCurrent)
        .withSupplyCurrentLimitEnable(true));

        m_algaeintakeFX.getConfigurator().apply(new MotionMagicConfigs()
        .withMotionMagicAcceleration(IntakeConstants.kAccelerationAglaeIntake)
        .withMotionMagicCruiseVelocity(IntakeConstants.kVelocityAglaeIntake));
    
        m_algaeintakeFX.getConfigurator().apply( new Slot0Configs()
        .withKP(IntakeConstants.kPAlgaeIntake)
        .withKI(IntakeConstants.kIAlgaeIntake)
        .withKD(IntakeConstants.kDAlgaeIntake)
        .withKG(IntakeConstants.kGAlgaeIntake)
        .withKS(IntakeConstants.kSAlgaeIntake)
        .withKV(IntakeConstants.kVAlgaeIntake)
        .withKA(IntakeConstants.kAAlgaeIntake));
        
        m_algaeintakeFX.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(IntakeConstants.kUpperLimit/IntakeConstants.kInchPerRotation)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(IntakeConstants.kLowerLimit/IntakeConstants.kInchPerRotation));

        m_algaeintakeFX.setNeutralMode(NeutralModeValue.Brake); 
        
        m_algaeintakeSRX.setMaxCurrent(CurrentType.STATOR, 25);
        m_algaeintakeSRX.setNTLogging(true); 
    }
    
    public double getPosition() {      //Inches
        return (m_algaeintakeFX.getPosition().getValueAsDouble());
    }

    public void setPosition(double position) {       // Inches  
        m_setPoint = position;
        m_algaeintakeFX.setControl(new MotionMagicVoltage(position));
    }

    public void setRollers(double speed) {
        m_algaeintakeSRX.set(speed);
    }
    
    public void stop() {
        m_algaeintakeSRX.set(0);
    }

    public double getCurrent() {
        return (m_algaeintakeFX.getSupplyCurrent().getValueAsDouble());
    }

   public void jogging(boolean direction){
        if (direction == true) {
            setPosition(getPosition()+0.5);
        }
        else {
            setPosition(getPosition()-0.5);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algae Intake Position", getPosition());
        SmartDashboard.putNumber("Algae Intake Current", getCurrent());
        SmartDashboard.putNumber("Algae Intake Set Point", m_setPoint);
    }
}

