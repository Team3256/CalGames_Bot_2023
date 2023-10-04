// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.KeepArm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.helpers.SpawnCommand;

public class SetEndEffectorState extends SpawnCommand {
  private Arm armSubsystem;
  private Elevator elevatorSubsystem;
  private Rotation2d armAngle;
  private double elevatorExtension;
  private boolean moveArmFirst;

  public enum EndEffectorPreset {
    // scoring
    SCORE_CONE_HIGH(Arm.ArmPreset.CONE_HIGH, Elevator.ElevatorPreset.CONE_HIGH),
    SCORE_CONE_MID(Arm.ArmPreset.CONE_MID, Elevator.ElevatorPreset.ANY_PIECE_MID),
    SCORE_CUBE_HIGH(Arm.ArmPreset.CUBE_HIGH, Elevator.ElevatorPreset.CUBE_HIGH),
    SCORE_CUBE_MID(Arm.ArmPreset.CUBE_MID, Elevator.ElevatorPreset.ANY_PIECE_MID),
    SCORE_ANY_LOW_BACK(
        Arm.ArmPreset.ANY_PIECE_LOW_BACK, Elevator.ElevatorPreset.ANY_PIECE_LOW_BACK),
    SCORE_ANY_LOW_FRONT(
        Arm.ArmPreset.ANY_PIECE_LOW_FRONT, Elevator.ElevatorPreset.ANY_PIECE_LOW_FRONT),
    // substation
    DOUBLE_SUBSTATION_CONE(
        Arm.ArmPreset.DOUBLE_SUBSTATION_CONE, Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE),
    DOUBLE_SUBSTATION_CUBE(
        Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE, Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE),
    // ground intake
    CUBE_GROUND_INTAKE(Arm.ArmPreset.CUBE_GROUND_INTAKE, Elevator.ElevatorPreset.GROUND_INTAKE),
    SITTING_CONE_GROUND_INTAKE(
        Arm.ArmPreset.SITTING_CONE_GROUND_INTAKE, Elevator.ElevatorPreset.GROUND_INTAKE),
    STANDING_CONE_GROUND_INTAKE(
        Arm.ArmPreset.STANDING_CONE_GROUND_INTAKE, Elevator.ElevatorPreset.GROUND_INTAKE),
    // stow
    STOW(Arm.ArmPreset.STOW, Elevator.ElevatorPreset.STOW);

    public final Arm.ArmPreset armPreset;
    public final Elevator.ElevatorPreset elevatorPreset;

    EndEffectorPreset(Arm.ArmPreset armPreset, Elevator.ElevatorPreset elevatorPreset) {
      this.armPreset = armPreset;
      this.elevatorPreset = elevatorPreset;
    }
  }

  public SetEndEffectorState(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      EndEffectorPreset endEffectorPreset,
      boolean moveArmFirst) {
    super();
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorExtension = endEffectorPreset.elevatorPreset.position;
    this.armAngle = endEffectorPreset.armPreset.rotation;
    this.moveArmFirst = moveArmFirst;
  }

  public SetEndEffectorState(
      Elevator elevatorSubsystem, Arm armSubsystem, EndEffectorPreset endEffectorPreset) {
    this(elevatorSubsystem, armSubsystem, endEffectorPreset, false);
  }

  @Override
  public void initialize() {
    if (moveArmFirst) {
      setChildCommand(
          Commands.sequence(
              new SetArmAngle(armSubsystem, armAngle).asProxy(),
              Commands.parallel(
                  new InstantCommand(() -> new KeepArm(armSubsystem).schedule()),
                  new SetElevatorExtension(elevatorSubsystem, elevatorExtension))));
    } else {
      setChildCommand(
          Commands.sequence(
              new SetElevatorExtension(elevatorSubsystem, elevatorExtension),
              new SetArmAngle(armSubsystem, armAngle).asProxy(),
              new InstantCommand(() -> new KeepArm(armSubsystem).schedule())));
    }
    super.initialize();
  }
}
