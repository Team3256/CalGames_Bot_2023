// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.helpers.ParentCommand;
import frc.robot.intake.Intake;
import java.util.function.BooleanSupplier;

public class IntakeOrOuttakeConeOrCube extends ParentCommand {
  private final Intake intakeSubsystem;
  private final boolean isIntake;
  private final BooleanSupplier isCone;

  public IntakeOrOuttakeConeOrCube(
      Intake intakeSubsystem, boolean isIntake, BooleanSupplier isCone) {
    super();
    this.intakeSubsystem = intakeSubsystem;
    this.isIntake = isIntake;
    this.isCone = isCone;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    if (isIntake) {
      addChildCommands(
          new ConditionalCommand(
              new IntakeCone(intakeSubsystem), new IntakeCube(intakeSubsystem), isCone));

    } else {
      addChildCommands(
          new ConditionalCommand(
              new OuttakeCone(intakeSubsystem), new OuttakeCube(intakeSubsystem), isCone));
    }
    super.initialize();
  }
}
