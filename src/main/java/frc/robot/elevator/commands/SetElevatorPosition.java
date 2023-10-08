// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.Elevator.ElevatorPreset;

public class SetElevatorPosition extends ProfiledPIDCommand {
  private double setpointPosition;
  private final Elevator elevatorSubsystem;
  private ElevatorPreset elevatorPreset;

  /** Constructor for setting the elevator to a setpoint in the parameters */
  public SetElevatorPosition(Elevator elevatorSubsystem, double setpointPosition) {
    this(elevatorSubsystem, setpointPosition, false);
  }

  public SetElevatorPosition(
      Elevator elevatorSubsystem, double setpointPosition, boolean accurate) {
    super(
        new ProfiledPIDController(kElevatorP, kElevatorI, kElevatorD, kElevatorConstraints),
        elevatorSubsystem::getElevatorPosition,
        setpointPosition,
        (output, setpoint) ->
            elevatorSubsystem.setInputVoltage(
                output + elevatorSubsystem.calculateFeedForward(setpoint.velocity)),
        elevatorSubsystem);

    this.setpointPosition = setpointPosition;
    this.elevatorSubsystem = elevatorSubsystem;

    getController()
        .setTolerance(
            accurate ? kAccurateTolerancePosition : kTolerancePosition, kToleranceVelocity);

    addRequirements(elevatorSubsystem);
  }

  /**
   * Constructor for setting elevator height for the levels specified in the elevator preferences
   * hash map
   */
  public SetElevatorPosition(Elevator elevatorSubsystem, ElevatorPreset elevatorPreset) {
    this(elevatorSubsystem, elevatorSubsystem.getElevatorSetpoint(elevatorPreset));
    this.elevatorPreset = elevatorPreset;
  }

  @Override
  public void initialize() {
    super.initialize();

    // update at runtime in case robot prefs changed
    if (elevatorPreset != null) {
      setpointPosition = elevatorSubsystem.getElevatorSetpoint(elevatorPreset);
      getController().setGoal(setpointPosition);
    }

    System.out.println(
        this.getName()
            + " started (preset: "
            + elevatorPreset
            + ", height: "
            + setpointPosition
            + " meters)");
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber(
        "Elevator desired position", Units.metersToInches(getController().getSetpoint().position));
    SmartDashboard.putNumber(
        "Elevator desired velocity", Units.metersToInches(getController().getSetpoint().velocity));
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    System.out.println(
        this.getName()
            + " finished (preset: "
            + elevatorPreset
            + ", height: "
            + ", current height: "
            + elevatorSubsystem.getElevatorPosition()
            + " meters, "
            + setpointPosition
            + " meters)");
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
