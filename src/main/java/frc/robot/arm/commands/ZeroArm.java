// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm.commands;

import static frc.robot.arm.ArmConstants.kZeroArmStatorCurrentThreshold;
import static frc.robot.arm.ArmConstants.kZeroArmVoltage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.Arm;

public class ZeroArm extends CommandBase {
  private final Arm armSubsystem;

  public ZeroArm(Arm armSubsystem) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
  }

  @Override
  public void initialize() {
    armSubsystem.setInputVoltage(kZeroArmVoltage);
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) armSubsystem.zeroThroughboreEncoder();
  }

  @Override
  public boolean isFinished() {
    return armSubsystem.getStatorCurrent() > kZeroArmStatorCurrentThreshold;
  }
}
