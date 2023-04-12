// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import static frc.robot.auto.AutoConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmPreset;
import frc.robot.arm.commands.SetArmAngle;
import frc.robot.arm.commands.StowArmElevator;
import frc.robot.auto.helpers.AutoBuilder;
import frc.robot.auto.helpers.AutoChooser;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorExtension;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.IntakeOff;
import frc.robot.intake.commands.OutakeCone;
import frc.robot.intake.commands.OutakeCube;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.AutoBalance;
import frc.robot.swerve.commands.LockSwerveX;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoPaths {
  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private BooleanSupplier isCurrentPieceCone;
  private HashMap<String, Supplier<Command>> autoEventMap = new HashMap<>();

  public AutoPaths(
      SwerveDrive swerveSubsystem,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      BooleanSupplier isCurrentPieceCone) {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.isCurrentPieceCone = isCurrentPieceCone;
  }

  public void sendCommandsToChooser() {
    AutoChooser.createSingleDefaultPath("Do Nothing", new InstantCommand());

    Supplier<Command> scorePreloadCone = () -> new InstantCommand();
    Supplier<Command> scorePreloadCube = () -> new InstantCommand();

    autoEventMap.put(
        "engage",
        () -> new AutoBalance(swerveSubsystem)
            .andThen(new LockSwerveX(swerveSubsystem))
            .asProxy()
            .withName("engage"));

    if (swerveSubsystem != null
        && intakeSubsystem != null
        && armSubsystem != null
        && elevatorSubsystem != null) {

      autoEventMap.put("outtakeCube", () -> new OutakeCube(intakeSubsystem).asProxy());

      autoEventMap.put("outtakeCone", () -> new OutakeCone(intakeSubsystem).asProxy());

      autoEventMap.put(
          "coneHigh",
          () -> Commands.parallel(
              new SetElevatorExtension(
                  elevatorSubsystem, Elevator.ElevatorPreset.CONE_HIGH),
              new SetArmAngle(armSubsystem, ArmPreset.CONE_HIGH))
              .asProxy()
              .withName("coneHigh"));

      autoEventMap.put(
          "coneMid",
          () -> Commands.parallel(
              new SetElevatorExtension(
                  elevatorSubsystem, Elevator.ElevatorPreset.ANY_PIECE_MID),
              new SetArmAngle(armSubsystem, ArmPreset.CONE_MID))
              .asProxy()
              .withName("coneMid"));

      autoEventMap.put(
          "coneLow",
          () -> Commands.parallel(
              new SetElevatorExtension(
                  elevatorSubsystem, Elevator.ElevatorPreset.ANY_PIECE_LOW),
              new SetArmAngle(armSubsystem, ArmPreset.ANY_PIECE_LOW))
              .asProxy()
              .withName("coneLow"));

      autoEventMap.put(
          "cubeHigh",
          () -> Commands.parallel(
              new SetElevatorExtension(
                  elevatorSubsystem, Elevator.ElevatorPreset.CUBE_HIGH),
              new SetArmAngle(armSubsystem, ArmPreset.CUBE_HIGH))
              .asProxy()
              .withName("cubeHigh"));

      autoEventMap.put(
          "cubeMid",
          () -> Commands.parallel(
              new SetElevatorExtension(
                  elevatorSubsystem, Elevator.ElevatorPreset.ANY_PIECE_MID),
              new SetArmAngle(armSubsystem, ArmPreset.CUBE_MID))
              .asProxy()
              .withName("cubeMid"));

      autoEventMap.put(
          "cubeLow",
          () -> Commands.parallel(
              new SetElevatorExtension(
                  elevatorSubsystem, Elevator.ElevatorPreset.ANY_PIECE_LOW),
              new SetArmAngle(armSubsystem, ArmPreset.ANY_PIECE_LOW))
              .asProxy()
              .withName("cubeLow"));

      autoEventMap.put(
          "defaultPosition",
          () -> runParallelWithPath(
              Commands.parallel(
                  new StowArmElevator(elevatorSubsystem, armSubsystem, isCurrentPieceCone),
                  new IntakeOff(intakeSubsystem))
                  .asProxy()
                  .withName("stow")));

      autoEventMap.put(
          "stow",
          () -> runParallelWithPath(
              Commands.parallel(
                  new StowArmElevator(elevatorSubsystem, armSubsystem, isCurrentPieceCone),
                  new IntakeOff(intakeSubsystem))
                  .asProxy()
                  .withName("stow")));

      autoEventMap.put(
          "intakeCone",
          () -> runParallelWithPath(
              Commands.deadline(
                  new IntakeCone(intakeSubsystem),
                  new SetElevatorExtension(
                      elevatorSubsystem, Elevator.ElevatorPreset.GROUND_INTAKE),
                  new SetArmAngle(armSubsystem, ArmPreset.CONE_GROUND_INTAKE)))
              .asProxy()
              .withName("intakeCone"));

      autoEventMap.put(
          "intakeCube",
          () -> runParallelWithPath(
              Commands.deadline(
                  new IntakeCube(intakeSubsystem),
                  new SetElevatorExtension(
                      elevatorSubsystem, Elevator.ElevatorPreset.GROUND_INTAKE),
                  new SetArmAngle(armSubsystem, ArmPreset.CUBE_GROUND_INTAKE)))
              .asProxy()
              .withName("intakeCube"));
    }

    AutoBuilder autoBuilder = new AutoBuilder(swerveSubsystem, autoEventMap);

    // NODE 1
    Command node1x3 = autoBuilder.createPath("Node1x3", kFastPathConstraints, true);
    AutoChooser.createSinglePath("Node1x3", node1x3);

    Command node1x25Engage = autoBuilder.createPath("Node1x2.5-Engage", kFastPathConstraints, true);
    AutoChooser.createSinglePath("Node1x2.5-Engage", node1x25Engage);

    Command node1x2 = autoBuilder.createPath("Node1x2", kFastPathConstraints, true);
    AutoChooser.createSinglePath("Node1x2", node1x2);

    Command node1Mobility = autoBuilder.createPath("Node1-Mobility", kFastPathConstraints, true);
    AutoChooser.createSinglePath("Node1-Mobility", node1Mobility);

    // NODE 9
    Command node9x3 = autoBuilder.createPath("Node9x3", kFastPathConstraints, true);
    AutoChooser.createSinglePath("Node9x3", node9x3);

    Command node9x25Engage = autoBuilder.createPath("Node9x2.5-Engage", kFastPathConstraints, true);
    AutoChooser.createSinglePath("Node9x2.5-Engage", node9x25Engage);

    Command node9x2 = autoBuilder.createPath("Node9x2", kFastPathConstraints, true);
    AutoChooser.createSinglePath("Node9x2", node9x2);

    Command node9Mobility = autoBuilder.createPath("Node9-Mobility", kFastPathConstraints, true);
    AutoChooser.createSinglePath("Node9-Mobility", node9Mobility);

    // NODE 5
    Command node5Engage = autoBuilder.createPath("Node5-Engage", kFastPathConstraints, true);
    AutoChooser.createSinglePath("Node5-Engage", node5Engage);

    Command node5x2EngageBottom = autoBuilder.createPath("Node5x2-Engage-Bottom", kEngagePathConstraints, true);
    AutoChooser.createSinglePath("Node5x2-Engage-Bottom", node5x2EngageBottom);

    Command node5x2EngageTop = autoBuilder.createPath("Node5x2-Engage-Top", kEngagePathConstraints, true);
    AutoChooser.createSinglePath("Node5x2-Engage-Top", node5x2EngageTop);

    Command node5MobilityEngage = autoBuilder.createPath("Node5-Mobility-Engage", kEngagePathConstraints, true);
    AutoChooser.createSinglePath("Node5-Mobility-Engage", node5MobilityEngage);

    AutoChooser.sendChooserToDashboard("Auto Chooser");
  }

  public Command runParallelWithPath(Command command) {
    return new InstantCommand(command::schedule);
  }

  public Command getSelectedPath() {
    return AutoChooser.getCommand();
  }
}
