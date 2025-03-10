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
    private LaserCan lc;

    public CoralArm(){
        m_Nova = new ThriftyNova(10, MotorType.MINION);
        m_Nova.setMaxCurrent(CurrentType.STATOR, 60.0);
        m_Nova.setMaxCurrent(CurrentType.SUPPLY, 40.0);
        m_Nova.setVoltageCompensation(0.0);
        m_Nova.pid0.setFF(0.0000266);
        m_Nova.pid0.setP(0.00003);

        lc = new LaserCan(10);
        try {
              lc.setRangingMode(LaserCan.RangingMode.SHORT);
              lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
              lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
                 } catch (ConfigurationFailedException e) {  System.out.println("Configuration failed! " + e);
             }

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

    public int getMeasurement() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        return (measurement.distance_mm);
    }

    public boolean haveCoral() {
        return (getMeasurement() < 40);
    }


    // @Override
    public void periodic() {
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        SmartDashboard.putNumber(getName(), measurement.distance_mm);
    }

    SmartDashboard.putNumber("Coral Velo", m_Nova.getVelocity()/42.0);
    SmartDashboard.putNumber("Coral Current", m_Nova.getStatorCurrent());
    }
}