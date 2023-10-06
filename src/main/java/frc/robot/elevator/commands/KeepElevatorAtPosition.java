// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator.commands;

import static frc.robot.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.elevator.Elevator;

public class KeepElevatorAtPosition extends PIDCommand {
  private final Elevator elevatorSubsystem;

  public KeepElevatorAtPosition(Elevator elevatorSubsystem, double position) {
    super(
        new PIDController(kElevatorP, kElevatorI, kElevatorD),
        elevatorSubsystem::getElevatorPosition,
        position,
        output ->
            elevatorSubsystem.setInputVoltage(output + elevatorSubsystem.calculateFeedForward(0)));

    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  public KeepElevatorAtPosition(Elevator elevatorSubsystem) {
    super(
        new PIDController(kElevatorP, kElevatorI, kElevatorD),
        elevatorSubsystem::getElevatorPosition,
        elevatorSubsystem.getElevatorPosition(),
        output ->
            elevatorSubsystem.setInputVoltage(output + elevatorSubsystem.calculateFeedForward(0)));

    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
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
    return false;
  }
}
