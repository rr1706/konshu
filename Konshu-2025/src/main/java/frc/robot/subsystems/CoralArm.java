package frc.robot.subsystems;

import com.thethriftybot.ThriftyNova;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;



public class CoralArm extends SubsystemBase{
//     public static final ThriftyNova m_Nova = new ThriftyNova(13);
//     private LaserCan lc;

//     public CoralArm(){
//         m_Nova.setInversion(true);
//         lc = new LaserCan(0);
//     //     try {
//     //           lc.setRangingMode(LaserCan.RangingMode.SHORT);
//     //           lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
//     //           lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
//     //              } catch (ConfigurationFailedException e) {  System.out.println("Configuration failed! " + e);
//     //          }
//     }

//     public void runCoral(double speed) {

//         m_Nova.setPercent(speed);
//     }
//     @Override
//     public void periodic() {
//     LaserCan.Measurement measurement = lc.getMeasurement();
//     if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
//         //  System.out.println("The target is " + measurement.distance_mm + "mm away!");
//          if (measurement.distance_mm < 23.0){
//             m_Nova.setPercent(0.0);
//     }   else {
//             m_Nova.setPercent(0.5);
//         }
// }    else {       
//         //  System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
//          m_Nova.setPercent(0.0);

// //             // You can still use distance_mm in here, if you're ok tolerating a clamped
// //             // value or an unreliable measurement.
// }
// }
}

