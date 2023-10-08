// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.elevator.Elevator;
import frc.robot.helpers.ParentCommand;

public class SetEndEffectorState extends ParentCommand {
  private Arm armSubsystem;
  private Elevator elevatorSubsystem;
  private Rotation2d armAngle;
  private double elevatorExtension;
  boolean accurate;

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
      Elevator elevatorSubsystem, Arm armSubsystem, EndEffectorPreset endEffectorPreset) {
    this(elevatorSubsystem, armSubsystem, endEffectorPreset, false);
  }

  public SetEndEffectorState(
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      EndEffectorPreset endEffectorPreset,
      boolean accurate) {
    super();
    this.armSubsystem = armSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorExtension = endEffectorPreset.elevatorPreset.position;
    this.armAngle = endEffectorPreset.armPreset.rotation;
    this.accurate = accurate;
    addRequirements(armSubsystem, elevatorSubsystem);
  }

  @Override
  public void initialize() {
      addChildCommands(
          Commands.sequence(
              new SetElevatorPosition(elevatorSubsystem, elevatorExtension, accurate),
              Commands.deadline(
                  new SetArmAngle(armSubsystem, armAngle),
                  new KeepElevatorAtPosition(
                      elevatorSubsystem, elevatorExtension)))); // may not work to add like this

    super.initialize();
  }
}
