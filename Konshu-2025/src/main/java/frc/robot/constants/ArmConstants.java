package frc.robot.constants;

public class ArmConstants {
  // We are using Motion Magic position control with voltage countrol output type, so the PID parameters are in volts
  public static final double kArmP = 200.0;   // Output per unit of error in position (volts/rotation)
  public static final double kArmI = 0.0;   // Output per unit of integrated error in position (volts/(rotation*s))
  public static final double kArmD = 10.0;   // Output per unit of error in velocity (volts/rotation)
  public static final double kArmG = -0.375;   // Output to overcome gravity (volts) - modified by ArmCosine
  public static final double kArmS = 0.2;   // Output to overcome static friction (volts)
  public static final double kArmV = 5.49;   // Output per unit of target velocity (volts/rps)
  public static final double kArmA = 0.001;   // Output per unit of target accelleration (volts/rps/rps)

  public static final double kArmCruiseVelocity = 1.0;
  public static final double kArmAcceleration = 6.0;
  // public static final double kArmJerk = 0.4;

  public static final double kArmGearRatio = (50.0/8.0)*(48.0/20.0)*(48.0/14.0);   // alpha was (44.0/8.0)*(40.0/20.0)*(48.0/14.0);
  public static final double kArmRotorToSensor = 22.0/18.0;

  // These are the constants for the arm stayout zones
  // kElevatorHighDanger - elevator must be higher than this to safely move arm above kArmHighDanger
  // KElevatorLowDanter - elevator must be higher than this to safely move arm below kArmLowDanger
  public static final double kArmHighDanger = 10.0-75.4+17.0; 
  public static final double kArmLowDanger = 32.0-75.4+17.0;

  public static final double kArmL1 = 12.1-75.4+17.0;      // These are in degrees
  public static final double kArmL1Interim = kArmL1;         // Waypoint to arm inside the elevator
  public static final double kArmL1IN = -60.0;               // Inside the elevator
  public static final double kArmL2 = 26.8-75.4+17.0;
  public static final double kArmL3 = 26.8-75.4+17.0;
  public static final double kArmL4 = -10.0;
  public static final double kArmProcessor = 6.0;
  public static final double kArmBarge = -25.0-75.4+17.0;
  public static final double kArmFeeder = -41.0;
  public static final double kArmGroundAlgae = 15.0-75.4+17.0;
  public static final double kArmAlgeaGrab = 32.5-75.4+17.0;

  public static final double kArmLowerLimit = kArmProcessor;   // Degrees from arm horizontal
  public static final double kArmUpperLimit = -65.0-75.4+17.0;       // Degrees from arm horizontal

  public static final double kMangentOffSet = 0.358224; // Rotations

  public static class ArmCurrents {
    public static final double kSupplyCurrent = 30.0;   // Amps
    public static final double kStatorCurrent = 50.0;
    }

}


//hi