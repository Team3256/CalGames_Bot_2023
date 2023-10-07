// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.pathgeneration.commands;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;
import static frc.robot.led.LEDConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.auto.dynamicpathgeneration.DynamicPathGenerator;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.pathgeneration.PathGeneration;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetEndEffectorState;
import frc.robot.elevator.commands.StowEndEffector;
import frc.robot.helpers.ParentCommand;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeConeOrCube;
import frc.robot.led.LED;
import frc.robot.led.commands.SetAllBlink;
import frc.robot.led.commands.SetAllColor;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;

public class AutoIntakeAtDoubleSubstation extends ParentCommand {
  public enum SubstationLocation {
    // From driver's POV
    RIGHT_SIDE,
    LEFT_SIDE
  }

  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private LED ledSubsystem;
  private BooleanSupplier cancelCommand;
  private BooleanSupplier isAutoScoreMode;
  private BooleanSupplier isCurrentPieceCone;
  private int guiStation;

  public AutoIntakeAtDoubleSubstation(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      BooleanSupplier cancelCommand,
      BooleanSupplier isAutoScoreMode,
      BooleanSupplier isCurrentPieceCone) {
    new AutoIntakeAtDoubleSubstation(
        swerveDrive,
        intakeSubsystem,
        elevatorSubsystem,
        armSubsystem,
        ledSubsystem,
        cancelCommand,
        isAutoScoreMode,
        isCurrentPieceCone,
        -1);
  }

  public AutoIntakeAtDoubleSubstation(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      LED ledSubsystem,
      BooleanSupplier cancelCommand,
      BooleanSupplier isAutoScoreMode,
      BooleanSupplier isCurrentPieceCone,
      int guiStation) {

    this.swerveSubsystem = swerveDrive;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.isAutoScoreMode = isAutoScoreMode;
    this.cancelCommand = cancelCommand;
    this.isCurrentPieceCone = isCurrentPieceCone;
    this.guiStation = guiStation;

    addRequirements(swerveDrive, intakeSubsystem, elevatorSubsystem, armSubsystem, ledSubsystem);
  }

  @Override
  public void initialize() {

    Command DoubleSubstationIntake;

    Command runIntake =
        new ConditionalCommand(
            new IntakeConeOrCube(intakeSubsystem),
            new IntakeCube(intakeSubsystem),
            isCurrentPieceCone);

    Command stowArmElevator = new StowEndEffector(elevatorSubsystem, armSubsystem);

    Command runningLEDs =
        new ConditionalCommand(
            new SetAllBlink(ledSubsystem, kCone),
            new SetAllBlink(ledSubsystem, kCube),
            isCurrentPieceCone);
    Command successLEDs = new SetAllColor(ledSubsystem, kSuccess);
    Command errorLEDs = new SetAllColor(ledSubsystem, kError);

    Command moveArmElevatorToPreset =
        new ParallelCommandGroup(
            new ConditionalCommand(
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.DOUBLE_SUBSTATION_CONE),
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.DOUBLE_SUBSTATION_CUBE),
                isCurrentPieceCone));

    System.out.println(
        "Is running auto intake instead of presets: " + isAutoScoreMode.getAsBoolean());
    if (!isAutoScoreMode.getAsBoolean()) {
      System.out.println(
          "Running intake preset at double substation for cone? "
              + isCurrentPieceCone.getAsBoolean());
      DoubleSubstationIntake =
          Commands.sequence(
                  Commands.deadline(runIntake, moveArmElevatorToPreset), stowArmElevator.asProxy())
              .deadlineWith(runningLEDs.asProxy())
              .until(cancelCommand)
              .handleInterrupt(errorLEDs::schedule)
              .finallyDo(
                  (interrupted) -> {
                    if (!interrupted) successLEDs.schedule();
                  });
    } else {

      System.out.println("Running: Go to substation from " + swerveSubsystem.getPose());
      Alliance alliance = DriverStation.getAlliance();

      Pose2d end;
      SubstationLocation substationLocation;
      int guiStation = (int) SmartDashboard.getNumber("guiStation", -1);
      if (guiStation < 0 || guiStation > 1) {
        System.out.println("guiStation was invalid (" + guiStation + ")");
        new SetAllColor(ledSubsystem, kError).withTimeout(2.5).schedule();
        return;
      }
      switch (guiStation) {
        case 0:
          substationLocation = SubstationLocation.LEFT_SIDE;
          break;
        case 1:
          substationLocation = SubstationLocation.RIGHT_SIDE;
          break;
        default:
          substationLocation = SubstationLocation.LEFT_SIDE;
          break;
      }
      if (substationLocation.equals(SubstationLocation.RIGHT_SIDE)) {
        // Left and right are different depending on alliance
        if (alliance == Alliance.Red) {
          end = kBlueOuterDoubleSubstationPose;
        } else {
          end = kBlueInnerDoubleSubstationPose;
        }
      } else {
        if (alliance == Alliance.Red) {
          end = kBlueInnerDoubleSubstationPose;
        } else {
          end = kBlueOuterDoubleSubstationPose;
        }
      }

      Pose2d substationWaypoint =
          new Pose2d(
              end.getX() - kSubstationWaypointOffset,
              end.getY(),
              end.getRotation().plus(kElevatorFckConstant));

      if (alliance == Alliance.Red) {
        end = PathUtil.flip(end);
        substationWaypoint = PathUtil.flip(substationWaypoint);
      }

      // commands that will be run sequentially
      Command moveToWaypoint;
      if (kDynamicPathGenerationDebug) {
        DynamicPathGenerator dpg =
            new DynamicPathGenerator(
                swerveSubsystem.getPose(), substationWaypoint, swerveSubsystem);
        moveToWaypoint = dpg.getCommand();
      } else {
        moveToWaypoint =
            PathGeneration.createDynamicAbsolutePath(
                swerveSubsystem.getPose(),
                substationWaypoint,
                swerveSubsystem,
                kWaypointPathConstraints);
      }

      Command moveToSubstation =
          PathGeneration.createDynamicAbsolutePath(
              substationWaypoint, end, swerveSubsystem, kPathToDestinationConstraints);

      // Automatically intake at the double substation
      DoubleSubstationIntake =
          Commands.sequence(
                  moveToWaypoint,
                  Commands.deadline(
                      runIntake.withTimeout(6), moveArmElevatorToPreset, moveToSubstation),
                  stowArmElevator.asProxy())
              .deadlineWith(runningLEDs.asProxy())
              .until(cancelCommand)
              .handleInterrupt(errorLEDs::schedule)
              .finallyDo(
                  (interrupted) -> {
                    if (!interrupted) successLEDs.schedule();
                  });
    }
    addChildCommands(DoubleSubstationIntake);
    super.initialize();
  }
}
