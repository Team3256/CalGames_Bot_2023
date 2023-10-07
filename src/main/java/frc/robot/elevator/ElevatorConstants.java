// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.drivers.CanDeviceId;
import java.util.Map;

public final class ElevatorConstants {
  public static final int kElevatorMasterID = 5;
  public static final int kElevatorFollowerID = 14;

  public static final double kElevatorStartingPosition = 0.5;
  public static final double kElevatorAngleOffset = Units.degreesToRadians(35.4);

  public static final String kElevatorCANBus = "rio";
  public static final CanDeviceId kElevatorCANDevice =
      new CanDeviceId(kElevatorMasterID, kElevatorCANBus);
  public static final CanDeviceId kElevatorFollowerCANDevice =
      new CanDeviceId(kElevatorFollowerID, kElevatorCANBus);

  public static final int kNumElevatorMotors = 2;

  // scoring
  public static final double kConeHighPosition = 1.36;
  public static final double kCubeHighPosition = 1.36;
  public static final double kAnyPieceMidPosition = 0.89;
  public static final double kAnyPieceLowBackPosition = 0;
  public static final double kAnyPieceLowFrontPosition = 0.39; // needs tuning
  // substation
  public static final double kCubeDoubleSubstationPosition = 1.12;
  public static final double kConeDoubleSubstationPosition = 1.1;
  // ground intake
  public static final double kGroundIntakePosition = 0;
  // stow
  public static final double kStowPosition = 0;

  public static final double kElevatorS = 0;
  public static final double kElevatorV = 8.869;
  public static final double kElevatorA = 21.64;
  public static final double kElevatorG = 0;
  public static final double kElevatorP = 11.399;
  public static final double kElevatorI = 0;
  public static final double kElevatorD = 0;

  // Comp: 3.5, 2
  // Tests: 2, 1.5
  public static final TrapezoidProfile.Constraints kElevatorConstraints =
      new TrapezoidProfile.Constraints(1, 1.5);

  public static final double kDownSpeedVolts = -6; // comp: -8
  public static final double kElevatorCurrentThreshold = 20; // amps
  public static final double kElevatorDownCurrentThreshold = 20; // amps
  public static final double kElevatorUpCurrentThreshold = 40; // amps

  public static final double kDrumRadius = Units.inchesToMeters(0.75);
  public static final double kMinExtension = 0;
  public static final double kMaxExtension = 1.5;
  public static final double kElevatorGearing = 25.0 / 3;
  public static final double kCarriageMass = 9; // kg

  public static final double kTolerancePosition = Units.inchesToMeters(2.5); // used to be 2.5
  public static final double kToleranceVelocity = Units.inchesToMeters(2.5); // used to be 2.5
  public static final double kRateLimiting = 0.05;

  public static class ElevatorPreferencesKeys {
    public static final Map<Elevator.ElevatorPreset, String> kElevatorPositionKeys =
        Map.of(
            Elevator.ElevatorPreset.CUBE_HIGH,
            "kCubeHighPositionMeters",
            Elevator.ElevatorPreset.CONE_HIGH,
            "kConeHighPositionMeters",
            Elevator.ElevatorPreset.ANY_PIECE_LOW_BACK,
            "kAnyPieceBackLowPositionMeters",
            Elevator.ElevatorPreset.ANY_PIECE_LOW_FRONT,
            "kAnyPieceFrontLowPositionMeters",
            Elevator.ElevatorPreset.ANY_PIECE_MID,
            "kAnyPieceMidPositionMeters",
            Elevator.ElevatorPreset.GROUND_INTAKE,
            "kGroundIntakePositionMeters",
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE,
            "kDoubleSubstationPositionConeMeters",
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE,
            "kDoubleSubstationPositionCubeMeters",
            Elevator.ElevatorPreset.STOW,
            "kStowPositionMeters");

    public static final Map<Elevator.ElevatorPreset, Double> kElevatorPositionDefaults =
        Map.of(
            Elevator.ElevatorPreset.CUBE_HIGH,
            kCubeHighPosition,
            Elevator.ElevatorPreset.CONE_HIGH,
            kConeHighPosition,
            Elevator.ElevatorPreset.ANY_PIECE_LOW_BACK,
            kAnyPieceLowBackPosition,
            Elevator.ElevatorPreset.ANY_PIECE_MID,
            kAnyPieceMidPosition,
            Elevator.ElevatorPreset.GROUND_INTAKE,
            kGroundIntakePosition,
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CONE,
            kConeDoubleSubstationPosition,
            Elevator.ElevatorPreset.DOUBLE_SUBSTATION_CUBE,
            kCubeDoubleSubstationPosition,
            Elevator.ElevatorPreset.STOW,
            kStowPosition);

    public static final String kPKey = "ElevatorkP";
    public static final String kIKey = "ElevatorkI";
    public static final String kDKey = "ElevatorkD";
  }
}
