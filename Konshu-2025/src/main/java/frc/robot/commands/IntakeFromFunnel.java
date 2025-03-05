// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralArm;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This command polls the LaserCan sensor and sets the motor output on the CoralArm subsystem.
 * <p>
 * It mirrors the following periodic behavior:
 * <pre>
 *   LaserCan.Measurement measurement = lc.getMeasurement();
 *   if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
 *       if (measurement.distance_mm < 23.0) {
 *           m_Nova.setPercent(0);
 *       } else {
 *           m_Nova.setPercent(-0.15);
 *       }
 *   } else {
 *       m_Nova.setPercent(0.0);
 *   }
 * </pre>
 */
public class IntakeFromFunnel extends Command {
  
  private final CoralArm coralArm;
  
  /**
   * Creates a new command instance.
   *
   * @param coralArm The subsystem that contains the LaserCan sensor and motor controller.
   */
  public IntakeFromFunnel(CoralArm coralArm) {
    this.coralArm = coralArm;
    // Declare subsystem dependencies.
    addRequirements(coralArm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Retrieve the latest measurement from the LaserCan sensor.
    int Measurement = coralArm.getMeasurement();
    

      if (Measurement < 23) {
        coralArm.runCoral(0);
      } else {
        coralArm.runCoral(-25.0);
      }
      // If the measurement is not valid, stop the motor.
      coralArm.runCoral(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Ensure that the motor is stopped when the command ends.
    //coralArm.runCoral(0.0);
  }

  // This command never finishes on its own.
  @Override
  public boolean isFinished() {
    return false;
  }
}
