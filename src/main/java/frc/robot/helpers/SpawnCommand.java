// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.wpilibj2.command.Command;

public class SpawnCommand extends DebugCommandBase {
  private Command childCommand;

  protected void setChildCommand(Command command) {
    childCommand = command;
  }

  @Override
  public void initialize() {
    childCommand.schedule();
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println("Parent killing child " + childCommand.getName());
    childCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return childCommand.isFinished();
  }
}
