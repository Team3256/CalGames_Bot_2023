// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import static frc.robot.arm.ArmConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.arm.Arm;

public class KeepArmAtAngle extends PIDCommand {
  private final Arm armSubsystem;

  public KeepArmAtAngle(Arm armSubsystem, double angle) {
    super(
        new PIDController(kArmP, kArmI, kArmD),
        armSubsystem::getArmAngleGroundRelative,
        angle,
        output ->
            armSubsystem.setInputVoltage(output + (armSubsystem.calculateFeedForward(angle, 0))*1.2));

    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  public KeepArmAtAngle(Arm armSubsystem) {
    super(
        new PIDController(kArmP, kArmI, kArmD),
        armSubsystem::getArmAngleGroundRelative,
        armSubsystem.getArmAngleGroundRelative(),
        output ->
            armSubsystem.setInputVoltage(
                output
                    + armSubsystem.calculateFeedForward(
                        armSubsystem.getArmAngleElevatorRelative(), 0)));

    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  @Override
  public void end(boolean interrupted) {
    if (Constants.kDebugEnabled) {
      System.out.println(
          "Keeping arm at position ended (position: "
              + Units.radiansToDegrees(armSubsystem.getArmAngleGroundRelative())
              + " deg)");
    }
    super.end(interrupted);
    armSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
