package frc.robot.constants;

public class ElevatorConstants {

      // We are using Motion Magic position control with voltage countrol output type, so the PID parameters are in volts
    public static final double kPElevator = 2.0;   // Output per unit of error in position (volts/rotation)
    public static final double kIElevator = 0.0;   // Output per unit of integrated error in position (volts/(rotation*s))
    public static final double kDElevator = 0.3;   // Output per unit of error in velocity (volts/rotation)
    public static final double kGElevator = 0.38;   // Output to overcome gravity (volts) - modified by ArmCosine
    public static final double kSElevator = .01;   // Output to overcome static friction (volts)
    public static final double kVElevator = .123;   // Output per unit of target velocity (volts/rps)
    public static final double kAElevator = 0.001;   // Output per unit of target accelleration (volts/rps/rps)

    public static final double kUpAccelerationElevator = 300.0; // Was 400, changed with new elevator 4/3/2025
    public static final double kUpVelocityElevator = 80.0;  // Was 86, changed wth new elevator 4/3/2025
    public static final double kUpJerkElevator = 5000;

    public static final double kDownAccelerationElevator = 200.0;
    public static final double kDownVelocityElevator = 50.0;
    public static final double kDownJerkElevator = 5000;
    
    public static final double kFElevator = 1.0;
    public static final double kElevatorGearRatio = 3.33; 
    public static final double kElevatorPositionFactor = 1.0; 
    public static final double kElevatorNeutral = 1.0; 
    public static final boolean elevatorConfigforwardSoftLimitEnabled = true;
    public static final double kElevatorL1 = 8.6;       // These are in inches
    public static final double kElevatorL2 = 22.0; 
    public static final double kElevatorL3 = 37.5; 
    public static final double kElevatorL4 = 71.1; 
    public static final double kElevatorLoadingStation = 0.2; 
    public static final double kElevatorProcessor = 5.7;
    public static final double kElevatorBarge = 69.61; 
    public static final double kElevatorZero = 0.0;
    public static final double kElelvatorGroundAlgae = 15;  
    public static final double kl1SafeToMoveMax = 2;
    public static final double kl2SafeToMoveMin = 18.65;
    public static final double kl3SafeToMoveMin = 34.5;
    public static final double kl4SafeToMoveMin = 60;
    public static final double kAlgeaIntakeTransfer = 13.21;
    public static final double kLowAlgeaGrab = 19.5;
    public static final double kHighAlgeaGrab = kLowAlgeaGrab+15.75;

    public static final double kUpperLimitElevator = 72.7;    // Inches
    public static final double kULowerLimitElevator = 0;

    public static final double kPulleyTeeth = 36.0;
    public static final double kPulleyPitch = .196850394;//in inches
    public static final double kGearRatio = 5.0;
    public static final double kInchPerRotation = (kPulleyTeeth*kPulleyPitch)/kGearRatio; 
    
    // These are the constants for the arm stayout zones
    // kElevatorHighDanger - elevator must be higher than this to safely move arm above kArmHighDanger
    // KElevatorLowDanter - elevator must be higher than this to safely move arm below kArmLowDanger
    public static final double kElevatorHighDanger = 23.0;
    public static final double kElevatorLowDanger = 4.2;

    public static class ElevatorCurrents {
        public static final double kElevatorNeutral = 1.0; 
        public static final double kStatorCurrent = 80.0;  // Was 50, changed for new elevator 4/3/2025
        public static final double kSupplyCurrent = 40.0; 
}
}
