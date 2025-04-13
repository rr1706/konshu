package frc.robot.subsystems;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class CoralArm extends SubsystemBase{
    private ThriftyNova m_Nova;
    private LaserCan lc1;
    private LaserCan lc2;
    private boolean m_startCoral = false;
    private boolean m_haveCoral = false;

    public CoralArm() {
        m_Nova = new ThriftyNova(10, MotorType.MINION);
        m_Nova.setMaxCurrent(CurrentType.STATOR, 40.0);
        m_Nova.setMaxCurrent(CurrentType.SUPPLY, 30.0);
        m_Nova.setVoltageCompensation(0.0);
        m_Nova.setBrakeMode(false);

        // lc1 is the first laser (at start), lc2 is the second laser (at end)
        lc1 = new LaserCan(20);
        try {
              lc1.setRangingMode(LaserCan.RangingMode.SHORT);
              lc1.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
              lc1.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
            } catch (ConfigurationFailedException e) {System.out.println("LC1: Configuration failed! " + e);}
        lc2 = new LaserCan(10);
        try {
              lc2.setRangingMode(LaserCan.RangingMode.SHORT);
              lc2.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
              lc2.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
            } catch (ConfigurationFailedException e) {System.out.println("LC2: Configuration failed! " + e);}
    }

    public void runCoral(double speed) {
        m_Nova.set(speed);
    }

    public Command runCoralCmd(double speed){
        return startEnd(()->runCoral(speed), 
        ()->stop());
    }

    public void stop(){
        m_Nova.stopMotor();
    }

    public int getMeasurement1() {
        LaserCan.Measurement measurement1 = lc1.getMeasurement();
        if (measurement1 != null && measurement1.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SmartDashboard.putBoolean("LaserCAN 1 Invalid", false);
            SmartDashboard.putNumber("LaserCAN 1 dist", measurement1.distance_mm);
            return (measurement1.distance_mm); }
        else
        SmartDashboard.putBoolean("LaserCAN 1 Invalid", true);
        return (20);
    }

    public int getMeasurement2() {
        LaserCan.Measurement measurement2 = lc2.getMeasurement();
        if (measurement2 != null && measurement2.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            SmartDashboard.putBoolean("LaserCAN 2 Invalid", false);
            SmartDashboard.putNumber("LaserCAN 2 dist", measurement2.distance_mm);
            return (measurement2.distance_mm); }
        else
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

        // Call getMeasurement2 only once (in periodic) since it is an expensive call and 
        // haveCoral is called multiple times per frame (SSM, LED, and IntakeFromFunnel);
        // whereas getMeasurement1 is only called once (IntakeFromFunnel command)
        m_haveCoral = getMeasurement2() < 40;
        SmartDashboard.putNumber("Coral Velo", m_Nova.getVelocity()/42.0);
        SmartDashboard.putNumber("Coral Current", m_Nova.getStatorCurrent());
    }
}