// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.drivers.CanDeviceId;
import java.util.Map;

public final class ArmConstants {
  // --CAN--
  public static final int kArmMotorID = 6;
  public static final String kArmCanBus = "rio";
  public static final CanDeviceId kArmCANDevice = new CanDeviceId(kArmMotorID, kArmCanBus);
  public static final int kArmSimulationID = 16;

  // --Character--
  public static final double kArmGearing = 80;
  public static final double kArmLength = 0.569075;
  public static final double kArmInertia = 0.410;
  public static final double kArmMassKg = 7.5;
  public static final int kNumArmMotors = 1;

  // --SysID--
  public static final double kArmS = 1.0785;
  public static final double kArmV = 0.65977;
  public static final double kArmA = 0.023807;
  public static final double kArmG = 0.52783;
  public static final double kArmP = 3.5;
  public static final double kArmI = 0;
  public static final double kArmD = 0;

  // --Constraints--
  public static final TrapezoidProfile.Constraints kArmProfileContraints =
      new TrapezoidProfile.Constraints(16, 16);
  public static final Rotation2d kArmAngleMinConstraint = Rotation2d.fromDegrees(22.3);
  public static final Rotation2d kArmAngleMaxConstraint = Rotation2d.fromDegrees(217);
  public static final double kArmCurrentThreshold = 18.5; // change as necessary 30

  // --Tolerance--
  public static final Rotation2d kArmToleranceAngle = Rotation2d.fromDegrees(5); // used to be 5
  public static final Rotation2d kArmToleranceAngularVelocity =
      Rotation2d.fromDegrees(5); // used to be 5

  // --Presets--
  public static final double kArmAngleInit = Units.degreesToRadians(217);
  // Stow
  public static final Rotation2d kStowRotation = Rotation2d.fromDegrees(60);
  // DoubleSub
  public static final Rotation2d kDoubleSubstationRotationCube = Rotation2d.fromDegrees(22.3);
  public static final Rotation2d kDoubleSubstationRotationCone = Rotation2d.fromDegrees(17.94);
  // Score
  public static final Rotation2d kCubeMidRotation = Rotation2d.fromDegrees(22.3);
  public static final Rotation2d kConeMidRotation = Rotation2d.fromDegrees(22.3);
  public static final Rotation2d kCubeHighRotation = Rotation2d.fromDegrees(22.3);
  public static final Rotation2d kConeHighRotation = Rotation2d.fromDegrees(22.3);
  public static final Rotation2d kAnyPieceLowBackRotation = Rotation2d.fromDegrees(180);
  public static final Rotation2d kAnyPieceLowFrontRotation = Rotation2d.fromDegrees(22.3);
  // Ground
  public static final Rotation2d kStandingConeGroundIntakeRotation = Rotation2d.fromDegrees(172.5);
  public static final Rotation2d kTippedConeGroundIntakeRotation = Rotation2d.fromDegrees(180);
  public static final Rotation2d kCubeGroundIntakeRotation = Rotation2d.fromDegrees(217);

  // --Preferences--
  public static class ArmPreferencesKeys {
    public static final Map<Arm.ArmPreset, String> kArmPositionKeys =
        Map.ofEntries(
            Map.entry(Arm.ArmPreset.STOW, "kStowRotation"),
            Map.entry(Arm.ArmPreset.ANY_PIECE_LOW_BACK, "kAnyPieceLowBackRotation"),
            Map.entry(Arm.ArmPreset.ANY_PIECE_LOW_FRONT, "kAnyPieceLowFrontRotation"),
            Map.entry(Arm.ArmPreset.CUBE_MID, "kCubeMidRotation"),
            Map.entry(Arm.ArmPreset.CONE_MID, "kConeMidRotation"),
            Map.entry(Arm.ArmPreset.CUBE_HIGH, "kCubeHighRotation"),
            Map.entry(Arm.ArmPreset.CONE_HIGH, "kConeHighRotation"),
            Map.entry(Arm.ArmPreset.CUBE_GROUND_INTAKE, "kCubeGroundIntakeRotation"),
            Map.entry(
                Arm.ArmPreset.STANDING_CONE_GROUND_INTAKE, "kStandingConeGroundIntakeRotation"),
            Map.entry(Arm.ArmPreset.SITTING_CONE_GROUND_INTAKE, "kTippedConeGroundIntakeRotation"),
            Map.entry(Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE, "kDoubleSubstationCubeRotation"),
            Map.entry(Arm.ArmPreset.DOUBLE_SUBSTATION_CONE, "kDoubleSubstationConeRotation"));

    public static final Map<Arm.ArmPreset, Rotation2d> kArmPositionDefaults =
        Map.ofEntries(
            Map.entry(Arm.ArmPreset.STOW, kStowRotation),
            Map.entry(Arm.ArmPreset.ANY_PIECE_LOW_BACK, kAnyPieceLowBackRotation),
            Map.entry(Arm.ArmPreset.CUBE_MID, kCubeMidRotation),
            Map.entry(Arm.ArmPreset.CONE_MID, kConeMidRotation),
            Map.entry(Arm.ArmPreset.CUBE_HIGH, kCubeHighRotation),
            Map.entry(Arm.ArmPreset.CONE_HIGH, kConeHighRotation),
            Map.entry(Arm.ArmPreset.CUBE_GROUND_INTAKE, kCubeGroundIntakeRotation),
            Map.entry(Arm.ArmPreset.STANDING_CONE_GROUND_INTAKE, kStandingConeGroundIntakeRotation),
            Map.entry(Arm.ArmPreset.SITTING_CONE_GROUND_INTAKE, kTippedConeGroundIntakeRotation),
            Map.entry(Arm.ArmPreset.DOUBLE_SUBSTATION_CUBE, kDoubleSubstationRotationCube),
            Map.entry(Arm.ArmPreset.DOUBLE_SUBSTATION_CONE, kDoubleSubstationRotationCone));

    public static final String kPKey = "ArmkP";
    public static final String kIKey = "ArmkI";
    public static final String kDKey = "ArmkD";
  }
}
