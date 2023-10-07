// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.helpers.DebugCommandBase;
import frc.robot.intake.Intake;

public class OuttakeConeOrCube extends DebugCommandBase {
  private final Intake intakeSubsystem;
  private final Timer timer;
  private final boolean manual;
  private final boolean isCone;

  public OuttakeConeOrCube(Intake intakeSubsystem, boolean manual, boolean isCone) {
    this.intakeSubsystem = intakeSubsystem;
    this.timer = new Timer();
    addRequirements(intakeSubsystem);
    this.manual = manual;
    this.isCone = isCone;
  }

  @Override
  public void initialize() {
    super.initialize();
    if (isCone) {
      intakeSubsystem.outtakeCone();
    } else {
      intakeSubsystem.outtakeCube();
    }
    timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    if (!manual) {
      double outtakeTime = 1;
      return timer.hasElapsed(outtakeTime);
    } else {
      return false;
    }
  }
}
