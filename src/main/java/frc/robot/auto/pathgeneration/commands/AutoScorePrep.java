// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto.pathgeneration.commands;

import static frc.robot.Constants.FeatureFlags.kDynamicPathGenEnabled;
import static frc.robot.auto.dynamicpathgeneration.DynamicPathConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.arm.Arm;
import frc.robot.auto.dynamicpathgeneration.DynamicPathGenerator;
import frc.robot.auto.dynamicpathgeneration.helpers.PathUtil;
import frc.robot.auto.pathgeneration.PathGeneration;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetEndEffectorState;
import frc.robot.helpers.ParentCommand;
import frc.robot.swerve.SwerveDrive;
import java.util.function.BooleanSupplier;

public class AutoScorePrep extends ParentCommand {
  private SwerveDrive swerveSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private BooleanSupplier isOperatorSelectingCone;
  private BooleanSupplier cancelCommand;

  public AutoScorePrep(
      SwerveDrive swerveDrive,
      Elevator elevatorSubsystem,
      Arm armSubsystem,
      BooleanSupplier isOperatorSelectingCone,
      BooleanSupplier cancelCommand) {
    this.swerveSubsystem = swerveDrive;
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.isOperatorSelectingCone = isOperatorSelectingCone;
    this.cancelCommand = cancelCommand;
    addRequirements(swerveDrive, elevatorSubsystem, armSubsystem);
  }

  @Override
  public void initialize() {
    // Get scoring location id from SmartDashboard
    int guiColumn = (int) SmartDashboard.getNumber("guiColumn", -1);
    if (0 > guiColumn || guiColumn > 8) {
      System.out.println("guiColumn was invalid (" + guiColumn + ")");
      return;
    }
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Blue)) {
      guiColumn = 8 - guiColumn;
    }

    // Get scoring height from SmartDashboard
    int guiRow = (int) SmartDashboard.getNumber("guiRow", -1);
    if (guiRow < 0 || guiRow > 2) {
      System.out.println("guiRow was invalid (" + guiRow + ")");
      return;
    }

    // Preset Effector
    Command moveArmElevatorToPreset;
    switch (guiRow) {
      case 0:
        moveArmElevatorToPreset =
            new ConditionalCommand(
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.SCORE_CONE_HIGH),
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.SCORE_CUBE_HIGH),
                isOperatorSelectingCone);
        break;
      case 1:
        moveArmElevatorToPreset =
            new ConditionalCommand(
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.SCORE_CONE_MID),
                new SetEndEffectorState(
                    elevatorSubsystem,
                    armSubsystem,
                    SetEndEffectorState.EndEffectorPreset.SCORE_CUBE_MID),
                isOperatorSelectingCone);
        break;
      default:
        moveArmElevatorToPreset =
            new SetEndEffectorState(
                elevatorSubsystem,
                armSubsystem,
                SetEndEffectorState.EndEffectorPreset.SCORE_ANY_LOW_FRONT);
        break;
    }

    // Move to node waypoint
    Pose2d start = swerveSubsystem.getPose();
    Pose2d scoringWaypoint = kBlueScoreWaypointPoses[guiColumn];
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
      scoringWaypoint = PathUtil.flip(scoringWaypoint);
    }
    Command moveToScoringWaypoint;
    if (kDynamicPathGenEnabled) {
      DynamicPathGenerator gen = new DynamicPathGenerator(start, scoringWaypoint, swerveSubsystem);
      moveToScoringWaypoint = gen.getCommand();
    } else {
      moveToScoringWaypoint =
          PathGeneration.createDynamicAbsolutePath(
              start, scoringWaypoint, swerveSubsystem, kWaypointPathConstraints);
    }

    // Move to node
    Pose2d scoringLocation = kHighBlueScoringPoses[guiColumn];
    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
      scoringLocation = PathUtil.flip(scoringLocation);
    }
    Command moveToScoringLocation =
        PathGeneration.createDynamicAbsolutePath(
            scoringWaypoint, scoringLocation, swerveSubsystem, kPathToDestinationConstraints);

    // Fin
    System.out.println("Running: Go to grid (id: " + guiColumn + ") from " + start);
    //    Command autoScore =
    //        Commands.sequence(
    //                moveToScoringWaypoint, moveToScoringLocation,
    // moveArmElevatorToPreset.asProxy())
    //            .until(cancelCommand);
    Command autoScore = moveArmElevatorToPreset;
    addChildCommands(autoScore);
    super.initialize();
  }
}
