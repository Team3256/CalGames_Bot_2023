// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.tests;

import static frc.robot.intake.IntakeConstants.*;
import static org.junit.jupiter.api.Assertions.*;

import frc.robot.UnitTestBase;
import frc.robot.intake.Intake;
import org.junit.jupiter.api.BeforeAll;

public class IntakeTests extends UnitTestBase {
  public final double DELTA = 0.05;

  private static Intake intakeSubsystem;

  @BeforeAll
  public static void setup() {
    UnitTestBase.setup();
    intakeSubsystem = new Intake();
  }

  // @Test
  // public void testIntakeCube() {
  // IntakeCube command = new IntakeConeOrCube(intakeSubsystem, false, false);
  // command.initialize();
  // runScheduler(1, command, intakeSubsystem);

  // double velocity = intakeSubsystem.getIntakeSpeed();
  // assertEquals(kIntakeCubeSpeed, velocity, DELTA, "testing Intake Cube");
  // }

  // @Test
  // public void testIntakeCone() {
  // IntakeConeOrCube command = new IntakeConeOrCube(intakeSubsystem, true, true);
  // command.initialize();
  // runScheduler(1, command, intakeSubsystem);

  // double velocity = intakeSubsystem.getIntakeSpeed();
  // assertEquals(kIntakeConeSpeed, velocity, DELTA, "testing Intake Cone");
  // }
}
