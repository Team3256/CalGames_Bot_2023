// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmConstants;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.helpers.ParentCommand;

public class StowEndEffector extends ParentCommand {
  private final Elevator elevatorSubsystem;
  private final Arm armSubsystem;

  public StowEndEffector(Elevator elevatorSubsystem, Arm armSubsystem) {
    super();
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(armSubsystem, elevatorSubsystem);
  }

  @Override
  public void initialize() {
    addChildCommands(
        Commands.sequence(
            new SetArmAngle(armSubsystem, Arm.ArmPreset.STOW),
            new ZeroElevator(elevatorSubsystem)));

    super.initialize();
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.getArmAngleElevatorRelative() == ArmConstants.kStowRotation.getRadians();
  }
}
