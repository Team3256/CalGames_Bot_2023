// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import frc.robot.helpers.DebugCommandBase;
import frc.robot.helpers.TimedBoolean;
import frc.robot.intake.Intake;

public class IntakeConeOrCube extends DebugCommandBase {
  private final Intake intakeSubsystem;
  private final TimedBoolean isCurrentSpiking;
  private final boolean manual;
  private final boolean isCone;

  public IntakeConeOrCube(Intake intakeSubsystem, boolean manual, boolean isCone) {
    this.intakeSubsystem = intakeSubsystem;
    this.isCurrentSpiking = new TimedBoolean(intakeSubsystem::isCurrentSpiking, 0.3);
    this.manual = manual;
    this.isCone = isCone;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    if (!manual) {
      isCurrentSpiking.initialize();
      intakeSubsystem.configIntakeCurrentLimit();
    }
    if (isCone) {
      intakeSubsystem.intakeCone();
    } else {
      intakeSubsystem.intakeCube();
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSubsystem.off();
    if (!manual) {
      if (!interrupted) {
        new LatchGamePiece(intakeSubsystem, isCone).schedule();
      }
    }
  }

  @Override
  public boolean isFinished() {
    if (!manual) {
      isCurrentSpiking.update();
      return isCurrentSpiking.hasBeenTrueForThreshold();
    } else {
      return false;
    }
  }
}
