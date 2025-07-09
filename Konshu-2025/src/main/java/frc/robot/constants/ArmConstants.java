package frc.robot.constants;

public class ArmConstants {
  // We are using Motion Magic position control with voltage countrol output type,
  // so the PID parameters are in volts
  public static final double kArmP = 50.0; // Output per unit of error in position (volts/rotation)
  public static final double kArmI = 0.0; // Output per unit of integrated error in position (volts/(rotation*s))
  public static final double kArmD = 2.0; // Output per unit of error in velocity (volts/rotation)
  public static final double kArmG = -0.375; // Output to overcome gravity (volts) - modified by ArmCosine
  public static final double kArmS = 0.2; // Output to overcome static friction (volts)
  public static final double kArmV = 5.49; // Output per unit of target velocity (volts/rps)
  public static final double kArmA = 0.001; // Output per unit of target accelleration (volts/rps/rps)

  public static final double kArmCruiseVelocity = 1.0;
  public static final double kArmAcceleration = 5.5;
  public static final double kSlowArmCruiseVelocity = 1.0;
  public static final double kSlowArmAcceleration = 5.5;
  // public static final double kArmJerk = 0.4;

  public static final double kArmGearRatio = (50.0 / 8.0) * (48.0 / 20.0) * (48.0 / 14.0); // alpha was
                                                                                           // (44.0/8.0)*(40.0/20.0)*(48.0/14.0);
  public static final double kArmRotorToSensor = 22.0 / 18.0;

  // These are the constants for the arm stayout zones
  // kElevatorHighDanger - elevator must be higher than this to safely move arm
  // above kArmHighDanger
  // KElevatorLowDanter - elevator must be higher than this to safely move arm
  // below kArmLowDanger
  public static final double kArmHighDanger = -48.4;
  public static final double kArmLowDanger = -26.4;

  public static final double kArmL1 = -44.3; // These are in degrees
  public static final double kArmL1Interim = kArmL1; // Waypoint to arm inside the elevator
  public static final double kArmL1IN = -78.0; // Inside the elevator
  public static final double kArmL2 = -29.6;
  public static final double kArmL3 = -29.6;
  public static final double kArmL4 = -6.0;
  public static final double kArmProcessor = 8.0;
  public static final double kArmBarge = -107.5;
  public static final double kToss = -107.5;

  public static final double kArmLoadingStation = -44.0;
  public static final double kArmGroundAlgae = -41.4;
  public static final double kArmAlgeaGrab = -23.9;

  public static final double kArmLowerLimit = kArmProcessor; // Degrees from arm horizontal
  public static final double kArmUpperLimit = -120.0; // Degrees from arm horizontal

  public static final double kMangentOffSet = 0.358224; // Rotations

  public static class ArmCurrents {
    public static final double kSupplyCurrent = 30.0; // Amps
    public static final double kStatorCurrent = 50.0;
  }
}