// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.pathgeneration.commands;

import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;

public class AutoIntakePrep extends ParentCommand {
  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private BooleanSupplier isCurrentPieceCone;
  private BooleanSupplier cancelCommand;

  public AutoIntakePrep(
      SwerveDrive swerveDrive,
      Intake intakeSubsystem,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      BooleanSupplier isCurrentPieceCone,
      BooleanSupplier cancelCommand) {

    this.swerveSubsystem = swerveDrive;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.isCurrentPieceCone = isCurrentPieceCone;
    this.cancelCommand = cancelCommand;

    addRequirements(swerveDrive, intakeSubsystem, elevatorSubsystem, armSubsystem);
  }

  @Override
  public void initialize() {
    // get guiStation
    int guiStation = (int) SmartDashboard.getNumber("guiStation", -1);
    if (guiStation < 0 || guiStation > 1) {
      System.out.println("guiStation was invalid (" + guiStation + ")");
      return;
    }

    // drive to waypoint and then substation
    Pose2d end;
    if (guiStation == 1) {
      if (DriverStation.getAlliance().equals(Alliance.Red)) {
        end = kBlueOuterDoubleSubstationPose;
      } else {
        end = kBlueInnerDoubleSubstationPose;
      }
    } else {
      if (DriverStation.getAlliance().equals(Alliance.Red)) {
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

    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      end = PathUtil.flip(end);
      substationWaypoint = PathUtil.flip(substationWaypoint);
    }
    Command moveToWaypoint;
    if (kDynamicPathGenerationDebug) {
      DynamicPathGenerator dpg =
          new DynamicPathGenerator(swerveSubsystem.getPose(), substationWaypoint, swerveSubsystem);
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

    // Intake
    Command runIntake =
        new ConditionalCommand(
            new IntakeConeOrCube(intakeSubsystem),
            new IntakeCube(intakeSubsystem),
            isCurrentPieceCone);

    // preset
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

    // stow
    Command stowArmElevator = new StowEndEffector(elevatorSubsystem, armSubsystem);

    // fin
    //    Command cmd =
    //        Commands.sequence(
    //                moveToWaypoint,
    //                Commands.deadline(
    //                    runIntake.withTimeout(6), moveArmElevatorToPreset.asProxy(),
    // moveToSubstation),
    //                stowArmElevator.asProxy())
    //            .until(cancelCommand);
    Command cmd = moveArmElevatorToPreset;
    addChildCommands(cmd);
    super.initialize();
  }
}
