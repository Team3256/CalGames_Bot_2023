// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;

public class LatchCube extends CommandBase {
  private final Intake intakeSubsystem;

  public LatchCube(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    intakeSubsystem.intakeCube(0.2);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
