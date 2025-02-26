package frc.robot.constants;

public class ArmConstants {
  // We are using Motion Magic position control with voltage countrol output type, so the PID parameters are in volts
  public static final double kArmP = 60.0;   // Output per unit of error in position (volts/rotation)
  public static final double kArmI = 0.0;   // Output per unit of integrated error in position (volts/(rotation*s))
  public static final double kArmD = 3.0;   // Output per unit of error in velocity (volts/rotation)
  public static final double kArmG = 0.45;   // Output to overcome gravity (volts) - modified by ArmCosine
  public static final double kArmS = 0.01;   // Output to overcome static friction (volts)
  public static final double kArmV = 4.25;   // Output per unit of target velocity (volts/rps)
  public static final double kArmA = 0.0;   // Output per unit of target accelleration (volts/rps/rps)

  public static final double kArmCruiseVelocity = 1.1;//1.1
  public static final double kArmAcceleration = 4.0; //10
  // public static final double kArmJerk = 8000.0;

  public static final double kArmGearRatio = (44.0/8.0)*(40.0/20.0)*(48.0/14.0);
  public static final double kArmRotorToSensor = 22.0/18.0;

  public static final double kArmL1 = 124.0;      // Degrees from arm hortizontal
  public static final double kArmL2 = 124.0;
  public static final double kArmL3 = 124.0;
  public static final double kArmL4 = 131.0;
  public static final double kArmProcessor = -52.0;
  public static final double kArmBarge = 115.0; 
  public static final double kArmFeeder = -56.0;
  public static final double kArmGroundAlgae = -47.0;

  public static final double kArmLowerLimit = kArmFeeder;   // Degrees from arm horizontal
  public static final double kArmUpperLimit = kArmL4;       // Degrees from arm horizontal

  public static final double kMangentOffSet = 0.0947; // Rotations

  // These are the constants for the arm stayout zones
  // kElevatorHighDanger - elevator must be higher than this to safely move arm above kArmHighDanger
  // KElevatorLowDanter - elevator must be higher than this to safely move arm below kArmLowDanger
  public static final double kArmHighDanger = 30.0;  // 40 was number from CAD, but add margin to give motor time to stop
  public static final double kArmLowDanger = -21.0;  // -23 was number from CAD, but add margin to give motor time to stop

  public static class ArmCurrents {
    public static final double kSupplyCurrent = 70.0;   // Amps
    public static final double kStatorCurrent = 120.0;
  }

}


