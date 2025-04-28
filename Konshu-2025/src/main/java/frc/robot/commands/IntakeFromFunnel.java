// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.CoralArm;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeFromFunnel extends Command {
  private final CoralArm coralArm;

  public IntakeFromFunnel(CoralArm coralArm) {
    this.coralArm = coralArm;
    // Declare subsystem dependencies.
    addRequirements(coralArm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // One sensor version...
      // if (coralArm.haveCoral()) {
      //   coralArm.runCoral(0);
      // } else {
      //   coralArm.runCoral(-.22);
      // }

      // Two sensor version...
      // Run fast until coral is first detected, then slow down.  Stop at second detector.
      if (coralArm.haveCoral()) {
        coralArm.runCoral(0);         // Second detector - stop coral
      } else if (coralArm.startCoral()) {
        coralArm.runCoral(-0.15);            // First but not second - run slow
      } else {
        coralArm.runCoral(-0.28);            // Neither - run fast
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // This command never finishes on its own.
  @Override
  public boolean isFinished() {
    return false;
  }
}
