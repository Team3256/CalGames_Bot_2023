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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.dynamicpathgeneration.DynamicPathGenerator;
import frc.robot.auto.pathgeneration.PathGeneration;
import frc.robot.helpers.ParentCommand;
import frc.robot.swerve.SwerveDrive;

public class AutoAlign extends ParentCommand {
  private SwerveDrive swerveSubsystem;
  private int guiColumn;

  public AutoAlign(SwerveDrive swerveDrive, int guiColumn) {
    this.swerveSubsystem = swerveDrive;
    this.guiColumn = guiColumn;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    Pose2d start = swerveSubsystem.getPose();
    // Get scoring location id from SD
    guiColumn = 8 - guiColumn; // ASSUME BLUE

    // Move to scoring waypoint
    Pose2d scoringWaypoint = kBlueScoreWaypointPoses[guiColumn];

    System.out.println("Running: Go to grid (id: " + guiColumn + ") from " + start);

    Command moveToScoringWaypoint;
    if (kDynamicPathGenEnabled) {
      DynamicPathGenerator gen = new DynamicPathGenerator(start, scoringWaypoint, swerveSubsystem);
      moveToScoringWaypoint = gen.getCommand();
    } else {
      moveToScoringWaypoint =
          PathGeneration.createDynamicAbsolutePath(
              start, scoringWaypoint, swerveSubsystem, kWaypointPathConstraints);
    }

    Pose2d scoringLocation = kHighBlueScoringPoses[guiColumn];
    Command moveToScoringLocation =
        PathGeneration.createDynamicAbsolutePath(
            scoringWaypoint, scoringLocation, swerveSubsystem, kPathToDestinationConstraints);

    Command autoScore = Commands.sequence(moveToScoringWaypoint, moveToScoringLocation);

    addChildCommands(autoScore);
    super.initialize();
  }
}
