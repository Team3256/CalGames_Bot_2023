// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class SpawnCommand extends DebugCommandBase {
  private Command childCommand;
  private int m_currentCommandIndex = -1;

  public SpawnCommand() {
    setChildCommand(new PrintCommand("NO COMMAND SELECTED!"));
  }

  protected void setChildCommand(Command command) {
    if (m_currentCommandIndex != -1) {
      throw new IllegalStateException(
          "Commands cannot be added to a composition while it's running");
    }

    CommandScheduler.getInstance().registerComposedCommands(command);

    childCommand = command;
    m_requirements.addAll(childCommand.getRequirements());
  }

  @Override
  public void initialize() {
    m_currentCommandIndex = 0;
    childCommand.initialize();
    super.initialize();
  }

  @Override
  public void execute() {
    childCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) childCommand.end(true);
    m_currentCommandIndex = -1;
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return childCommand.isFinished();
  }
}
