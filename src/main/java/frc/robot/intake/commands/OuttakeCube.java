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
import frc.robot.led.LED;

public class OuttakeCube extends DebugCommandBase {
  private Intake intakeSubsystem;
  private LED ledSubsystem;
  private Timer timer;

  public OuttakeCube(Intake intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.timer = new Timer();
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    intakeSubsystem.outtakeCube();
    timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intakeSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.0);
  }
}
