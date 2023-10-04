// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.Arm;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetEndEffectorState.EndEffectorPreset;
import frc.robot.helpers.DebugCommandBase;

public class StowEndEffector extends DebugCommandBase {
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;

  public StowEndEffector(Elevator elevatorSubsystem, Arm armSubsystem) {
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;

    addRequirements(elevatorSubsystem, armSubsystem);
  }

  @Override
  public void initialize() {
    Commands.sequence(
            new SetEndEffectorState(elevatorSubsystem, armSubsystem, EndEffectorPreset.STOW, true),
            new ZeroElevator(elevatorSubsystem))
        .schedule();

    super.initialize();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
