// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CoralArm;
import edu.wpi.first.wpilibj2.command.Command;

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
    
      if (coralArm.haveCoral()) {
        coralArm.runCoral(0);
      } else {
        coralArm.runCoral(-.22);
      }
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
