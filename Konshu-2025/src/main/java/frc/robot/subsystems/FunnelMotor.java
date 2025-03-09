package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.TimedRobot;

public class FunnelMotor extends SubsystemBase {
    private final TalonFX m_InFX = new TalonFX(13, "Drivetrain");

    public FunnelMotor() {
        m_InFX.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(false));

        m_InFX.setNeutralMode(NeutralModeValue.Brake);
        m_InFX.setInverted(true);
    }

    private LaserCan lc;

    private int speed; // TODO: Figure out value for speed later

    public void robotInit() {
        lc = new LaserCan(0);
        // Optionally initialise the settings of the LaserCAN, if you haven't already
        // done so in GrappleHook
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public void robotPeriodic() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            System.out.println("The target is " + measurement.distance_mm + "mm away!");
        } else {
            System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
            // You can still use distance_mm in here, if you're ok tolerating a clamped
            // value or an unreliable measurement.
        }
    }

    public void runCoralIn(double speed) {
        m_InFX.set(speed);
    }

    public void stopCoralIn() {
        m_InFX.set(0);
    }

    public void teleopInit() {
        stopCoralIn();
        try {
            Thread.sleep(3000); // Pause for 3 seconds
        } catch (InterruptedException e) {
            // Handle the exception
            e.printStackTrace();
        }
        runCoralIn(speed);
    }

    public void autonomousInit() {
        stopCoralIn();
        try {
            Thread.sleep(3000); // Pause for 3 seconds
        } catch (InterruptedException e) {
            // Handle the exception
            e.printStackTrace();
        }
        runCoralIn(speed);
    }

}
