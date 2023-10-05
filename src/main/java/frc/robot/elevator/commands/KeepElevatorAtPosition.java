// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.elevator.Elevator;

public class KeepElevatorAtPosition extends ProfiledPIDCommand {
  private final Elevator elevatorSubsystem;
  private double elevatorPosition;

  public KeepElevatorAtPosition(Elevator elevatorSubsystem) {
    super(
        new ProfiledPIDController(kElevatorP, kElevatorI, kElevatorD, kElevatorConstraints),
        elevatorSubsystem::getElevatorPosition,
        elevatorSubsystem.getElevatorPosition(),
        (output, setpoint) ->
            elevatorSubsystem.setInputVoltage(
                output + elevatorSubsystem.calculateFeedForward(setpoint.velocity)));

    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    elevatorPosition = elevatorSubsystem.getElevatorPosition();
    getController().setGoal(elevatorPosition);

    if (Constants.kDebugEnabled) {
      System.out.println(
          "Keeping elevator at position (position: " + elevatorPosition + " meters)");
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (Constants.kDebugEnabled) {
      System.out.println(
          "Keeping elevator at position ended (position: "
              + elevatorSubsystem.getElevatorPosition()
              + " meters)");
    }
    super.end(interrupted);
    elevatorSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
