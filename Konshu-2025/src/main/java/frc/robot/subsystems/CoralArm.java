package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class CoralArm extends SubsystemBase {
    private final TalonFXS m_motor;
    private LaserCan lc1;
    private LaserCan lc2;
    private boolean m_startCoral = false;
    private boolean m_haveCoral = false;

    public CoralArm() {
        m_motor = new TalonFXS(30, "rio");
        m_motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        m_motor.getConfigurator()
                .apply(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST));
        m_motor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(40.0)
                .withStatorCurrentLimitEnable(true).withSupplyCurrentLimit(20.0).withSupplyCurrentLimitEnable(true));

        // lc1 is the first laser (at start), lc2 is the second laser (at end)
        lc1 = new LaserCan(20);
        try {
            lc1.setRangingMode(LaserCan.RangingMode.SHORT);
            lc1.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc1.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("LC1: Configuration failed! " + e);
        }
        lc2 = new LaserCan(10);
        try {
            lc2.setRangingMode(LaserCan.RangingMode.SHORT);
            lc2.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc2.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("LC2: Configuration failed! " + e);
        }
    }

    public void runCoral(double volts) {
        m_motor.setVoltage(volts);
    }

    public Command runCoralCmd(double volts) {
        return startEnd(() -> runCoral(volts),
                () -> stop());
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public int getMeasurement1() {
        LaserCan.Measurement measurement1 = lc1.getMeasurement();
        if (measurement1 != null && measurement1.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SmartDashboard.putBoolean("LaserCAN 1 Invalid", false);
            SmartDashboard.putNumber("LaserCAN 1 dist", measurement1.distance_mm);
            return (measurement1.distance_mm);
        } else
            SmartDashboard.putBoolean("LaserCAN 1 Invalid", true);
        return (20);
    }

    public int getMeasurement2() {
        LaserCan.Measurement measurement2 = lc2.getMeasurement();
        if (measurement2 != null && measurement2.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SmartDashboard.putBoolean("LaserCAN 2 Invalid", false);
            SmartDashboard.putNumber("LaserCAN 2 dist", measurement2.distance_mm);
            return (measurement2.distance_mm);
        } else
            SmartDashboard.putBoolean("LaserCAN 2 Invalid", true);
        return (20);
    }

    public boolean haveCoral() {
        return (m_haveCoral);
    }

    public boolean startCoral() {
        m_startCoral = getMeasurement1() < 40;
        return (m_startCoral);
    }

    // @Override
    public void periodic() {
        m_haveCoral = getMeasurement2() < 40;
    }
}