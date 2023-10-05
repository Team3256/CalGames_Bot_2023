// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import static frc.robot.arm.ArmConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.arm.Arm;

public class KeepArmAtAngle extends ProfiledPIDCommand {
  private Arm armSubsystem;
  private double armAngle;

  /**
   * Constructor for setting arm to arbitrary angle in radians. This command is RELATIVE to the
   * Elevator Angle!
   *
   * @param armSubsystem
   */
  public KeepArmAtAngle(Arm armSubsystem) {
    super(
        new ProfiledPIDController(kArmP, kArmI, kArmD, kArmProfileContraints),
        armSubsystem::getArmAngleGroundRelative,
        armSubsystem.getArmAngleGroundRelative(),
        (output, setpoint) ->
            armSubsystem.setInputVoltage(
                output + armSubsystem.calculateFeedForward(setpoint.position, setpoint.velocity)));

    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    super.initialize();
    armAngle = armSubsystem.getArmAngleGroundRelative();
    getController().setGoal(armAngle);

    if (Constants.kDebugEnabled) {
      System.out.println(
          "Keeping arm at position (position: " + Units.radiansToDegrees(armAngle) + " deg)");
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (Constants.kDebugEnabled) {
      System.out.println(
          "Keeping arm at position ended (position: " + Units.radiansToDegrees(armAngle) + " deg)");
    }
    super.end(interrupted);
    armSubsystem.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
