// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.led.LEDConstants.*;
import static frc.robot.swerve.SwerveConstants.*;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FeatureFlags;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmVoltage;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.auto.pathgeneration.commands.AutoIntakeAtDoubleSubstation;
import frc.robot.auto.pathgeneration.commands.AutoIntakeAtDoubleSubstation.SubstationLocation;
import frc.robot.auto.pathgeneration.commands.AutoScore.*;
import frc.robot.drivers.CANTestable;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetElevatorVolts;
import frc.robot.elevator.commands.StowEndEffector;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.*;
import frc.robot.led.LED;
import frc.robot.led.commands.ColorFlowPattern;
import frc.robot.led.commands.SetAllColor;
import frc.robot.logging.Loggable;
import frc.robot.simulation.RobotSimulation;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.*;
import java.util.ArrayList;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements CANTestable, Loggable {
  public enum GamePiece {
    CUBE,
    CONE
  }

  public enum Mode {
    AUTO_SCORE,
    PRESET
  }

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController tester = new CommandXboxController(2);

  private SwerveDrive swerveSubsystem;
  private Intake intakeSubsystem;
  private Elevator elevatorSubsystem;
  private Arm armSubsystem;
  private LED ledSubsystem;

  private GamePiece currentPiece = GamePiece.CONE;
  private GridScoreHeight currentScoringPreset = GridScoreHeight.LOW;
  private SubstationLocation doubleSubstationLocation = SubstationLocation.RIGHT_SIDE;

  private RobotSimulation robotSimulation;
  private SendableChooser<Mode> modeChooser;
  private AutoPaths autoPaths;

  private final ArrayList<CANTestable> canBusTestables = new ArrayList<>();
  private final ArrayList<Loggable> loggables = new ArrayList<>();

  public RobotContainer() {
    if (kArmEnabled) armSubsystem = new Arm();
    if (kIntakeEnabled) intakeSubsystem = new Intake();
    if (kElevatorEnabled) elevatorSubsystem = new Elevator();
    if (kSwerveEnabled) swerveSubsystem = new SwerveDrive();
    if (kLedStripEnabled) ledSubsystem = new LED();

    if (kLedStripEnabled) {
      System.out.println("LED Subsystem Enabled");
      configureLEDStrip();
    } else System.out.println("LED Subsystem NOT Enabled");
    if (kIntakeEnabled) {
      System.out.println("Intake Subsystem Enabled");
      configureIntake();
      canBusTestables.add(intakeSubsystem);
      loggables.add(intakeSubsystem);
    } else System.out.println("Intake Subsystem NOT Enabled");
    if (kArmEnabled) {
      System.out.println("Arm Subsystem Enabled");
      configureArm();
      canBusTestables.add(armSubsystem);
      loggables.add(armSubsystem);
    } else System.out.println("Arm Subsystem NOT Enabled");
    if (kElevatorEnabled) {
      System.out.println("Elevator Subsystem Enabled");
      configureElevator();
      canBusTestables.add(elevatorSubsystem);
      loggables.add(elevatorSubsystem);
    } else System.out.println("Elevator Subsystem NOT Enabled");
    if (kSwerveEnabled) {
      System.out.println("Swerve Subsystem Enabled");
      configureSwerve();
      canBusTestables.add(swerveSubsystem);
      loggables.add(swerveSubsystem);
    } else System.out.println("Swerve Subsystem NOT Enabled");

    modeChooser = new SendableChooser<>();
    if (FeatureFlags.kAutoScoreEnabled) {
      modeChooser.setDefaultOption("Auto Score", Mode.AUTO_SCORE);
      modeChooser.addOption("Presets", Mode.PRESET);
    } else {
      modeChooser.setDefaultOption("Presets", Mode.PRESET);
      modeChooser.addOption("Auto Score", Mode.AUTO_SCORE);
    }
    SmartDashboard.putData("Auto Score Toggle", modeChooser);
    SmartDashboard.putString(
        "Current Double Substation Location", doubleSubstationLocation.toString());

    autoPaths =
        new AutoPaths(
            swerveSubsystem,
            intakeSubsystem,
            elevatorSubsystem,
            armSubsystem,
            this::setGamePiece,
            this::isCurrentPieceCone);
    autoPaths.sendCommandsToChooser();

    if (AutoConstants.kAutoDebug) {
      PathPlannerServer.startServer(5811);
    }

    if (Constants.kDebugEnabled && FeatureFlags.kShuffleboardLayoutEnabled) {
      for (Loggable loggable : loggables) {
        loggable.logInit();
      }
    }

    if (RobotBase.isSimulation()) {
      robotSimulation =
          new RobotSimulation(swerveSubsystem, intakeSubsystem, armSubsystem, elevatorSubsystem);
      robotSimulation.initializeRobot();
      // robotSimulation.addDoubleSubstation(GamePiece.CONE);
    }
  }

  private void configureSwerve() {
    swerveSubsystem.setDefaultCommand(
        new TeleopSwerve(
            swerveSubsystem,
            driver::getLeftY,
            driver::getLeftX,
            driver::getRightX,
            kFieldRelative,
            kOpenLoop));

    driver
        .povUp()
        .onTrue(
            new TeleopSwerveWithAzimuth(
                swerveSubsystem,
                driver::getLeftY,
                driver::getLeftX,
                () -> 0,
                () -> -1,
                () -> isMovingJoystick(driver),
                kFieldRelative,
                kOpenLoop));

    driver
        .povDown()
        .onTrue(
            new TeleopSwerveWithAzimuth(
                swerveSubsystem,
                driver::getLeftY,
                driver::getLeftX,
                () -> 0,
                () -> 1,
                () -> isMovingJoystick(driver),
                kFieldRelative,
                kOpenLoop));

    driver
        .povRight()
        .onTrue(
            new TeleopSwerveWithAzimuth(
                swerveSubsystem,
                driver::getLeftY,
                driver::getLeftX,
                () -> 1,
                () -> 0,
                () -> isMovingJoystick(driver),
                kFieldRelative,
                kOpenLoop));

    driver
        .povLeft()
        .onTrue(
            new TeleopSwerveWithAzimuth(
                swerveSubsystem,
                driver::getLeftY,
                driver::getLeftX,
                () -> -1,
                () -> 0,
                () -> isMovingJoystick(driver),
                kFieldRelative,
                kOpenLoop));

    driver.a().onTrue(new InstantCommand(swerveSubsystem::zeroGyroYaw));

    //
    //    driver
    //        .leftBumper()
    //        .toggleOnTrue(
    //            new TeleopSwerveLimited(
    //                    swerveSubsystem,
    //                    driver::getLeftY,
    //                    driver::getLeftX,
    //                    driver::getRightX,
    //                    kFieldRelative,
    //                    kOpenLoop)
    //                .deadlineWith(new LimitedSwervePattern(ledSubsystem,
    // this::isCurrentPieceCone)));

    driver
        .x()
        .onTrue(
            new LockSwerveX(swerveSubsystem)
                .andThen(new SetAllColor(ledSubsystem, kLockSwerve))
                .until(() -> isMovingJoystick(driver)));

    //    if (kElevatorEnabled && kArmEnabled && kLedStripEnabled) {
    //      new Trigger(this::scoreTriggered)
    //          .onTrue(
    //              new AutoScore(
    //                  swerveSubsystem,
    //                  intakeSubsystem,
    //                  elevatorSubsystem,
    //                  armSubsystem,
    //                  ledSubsystem,
    //                  this::isCurrentPieceCone,
    //                  () -> true,
    //                  () -> isMovingJoystick(driver),
    //                  true));
    //
    //      new Trigger(this::intakeTriggered)
    //          .onTrue(
    //              new AutoIntakeAtDoubleSubstation(
    //                  swerveSubsystem,
    //                  intakeSubsystem,
    //                  elevatorSubsystem,
    //                  armSubsystem,
    //                  ledSubsystem,
    //                  () -> isMovingJoystick(driver),
    //                  () -> modeChooser.getSelected().equals(Mode.AUTO_SCORE),
    //                  this::isCurrentPieceCone));
    //    }
  }

  // Checks if Button Board sent an intake command
  private boolean intakeTriggered() {
    return SmartDashboard.getBoolean("intakeTriggered", false);
  }

  // Checks if Button Board sent a score command
  private boolean scoreTriggered() {
    return SmartDashboard.getBoolean("scoreTriggered", false);
  }

  private void configureIntake() {
    //    operator
    //        .rightTrigger()
    //        .whileTrue(
    //            new ConditionalCommand(
    //                new OuttakeCone(intakeSubsystem),
    //                new OuttakeCube(intakeSubsystem),
    //                this::isCurrentPieceCone))
    //        .onFalse(
    //            new StowEndEffector(elevatorSubsystem, armSubsystem, this::isCurrentPieceCone)
    //                .asProxy());

    if (kArmEnabled && kElevatorEnabled) {
      //      driver
      //          .rightTrigger()
      //          .onTrue(
      //              new GroundIntake(
      //                  elevatorSubsystem,
      //                  armSubsystem,
      //                  intakeSubsystem,
      //                  ConeOrientation.STANDING_CONE,
      //                  this::isCurrentPieceCone));

      //      driver
      //          .rightBumper()
      //          .onTrue(
      //              new GroundIntake(
      //                  elevatorSubsystem,
      //                  armSubsystem,
      //                  intakeSubsystem,
      //                  ConeOrientation.SITTING_CONE,
      //                  this::isCurrentPieceCone));
    }
  }

  public void configureElevator() {
    //    if (kArmEnabled) {
    //      operator
    //          .leftTrigger()
    //          .onTrue(new StowEndEffector(elevatorSubsystem, armSubsystem,
    // this::isCurrentPieceCone));
    //    }
  }

  private void configureArm() {
    // operator.start().onTrue(new ZeroArm(armSubsystem).withTimeout(4));
    tester.povUp().whileTrue(new SetElevatorVolts(elevatorSubsystem, 6));
    tester.povDown().whileTrue(new SetElevatorVolts(elevatorSubsystem, -6));
    tester.povLeft().whileTrue(new SetArmVoltage(armSubsystem, 3));
    tester.povRight().whileTrue(new SetArmVoltage(armSubsystem, -3));
    //    tester.leftBumper().onTrue(new SetElevatorExtension(elevatorSubsystem, 0.5));
    //    tester.leftTrigger().onTrue(new SetElevatorExtension(elevatorSubsystem, 1));
    //    tester.rightBumper().onTrue(new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(0)));
    //    tester.rightTrigger().onTrue(new SetArmAngle(armSubsystem, Rotation2d.fromDegrees(90)));
    tester.leftBumper().onTrue(new IntakeCone(intakeSubsystem));
    tester.leftTrigger().onTrue(new OuttakeCone(intakeSubsystem));
    tester.rightBumper().onTrue(new IntakeCube(intakeSubsystem));
    tester.rightTrigger().onTrue(new OuttakeCube(intakeSubsystem));

    //    tester.a().onTrue(new AutoAlign(swerveSubsystem, 5));
    //    tester
    //        .y()
    //        .onTrue(
    //            new SequentialCommandGroup(
    //                new SetEndEffectorState(
    //                    elevatorSubsystem,
    //                    armSubsystem,
    //                    SetEndEffectorState.EndEffectorPreset.SCORE_CONE_MID),
    //                new OuttakeCube(intakeSubsystem)));
    //    tester.x().onTrue(new StowEndEffector(elevatorSubsystem, armSubsystem));
    //    tester
    //        .y()
    //        .onTrue(
    //            new SetEndEffectorState(
    //                elevatorSubsystem,
    //                armSubsystem,
    //                SetEndEffectorState.EndEffectorPreset.SCORE_CONE_HIGH));
    //    tester
    //        .b()
    //        .onTrue(
    //            new SetEndEffectorState(
    //                elevatorSubsystem,
    //                armSubsystem,
    //                SetEndEffectorState.EndEffectorPreset.CUBE_GROUND_INTAKE));
    tester
        .x()
        .onTrue(
            new AutoIntakeAtDoubleSubstation(
                swerveSubsystem,
                intakeSubsystem,
                elevatorSubsystem,
                armSubsystem,
                ledSubsystem,
                () -> isMovingJoystick(driver),
                () -> modeChooser.getSelected().equals(Mode.AUTO_SCORE),
                this::isCurrentPieceCone));

    //    if (kIntakeEnabled && FeatureFlags.kOperatorManualArmControlEnabled) {
    //      operator.povUp().whileTrue(new SetArmVoltage(armSubsystem,
    // ArmConstants.kManualArmVoltage));
    //      operator
    //          .povDown()
    //          .whileTrue(new SetArmVoltage(armSubsystem, -ArmConstants.kManualArmVoltage));
    //      operator
    //          .povUp()
    //          .or(operator.povDown())
    //          .whileTrue(
    //              new ConditionalCommand(
    //                  new IntakeCone(intakeSubsystem),
    //                  new IntakeCube(intakeSubsystem),
    //                  this::isCurrentPieceCone));
    //    }
  }

  public void configureLEDStrip() {
    ledSubsystem.setDefaultCommand(new ColorFlowPattern(ledSubsystem));
    //    operator.leftBumper().onTrue(new InstantCommand(this::toggleGamePiece));
  }

  // --MISC--
  public Command getAutonomousCommand() {
    Command autoPath = autoPaths.getSelectedPath();
    if (kElevatorEnabled && kArmEnabled) {
      return Commands.sequence(
          new StowEndEffector(elevatorSubsystem, armSubsystem),
          autoPath,
          Commands.parallel(
              new StowEndEffector(elevatorSubsystem, armSubsystem).asProxy(),
              new LockSwerveX(swerveSubsystem)
                  .andThen(new SetAllColor(ledSubsystem, kLockSwerve))
                  .until(() -> isMovingJoystick(driver))));
    } else {
      return autoPath;
    }
  }

  public boolean isMovingJoystick(CommandXboxController controller) {
    return Math.abs(controller.getLeftX()) > kStickCancelDeadband
        || Math.abs(controller.getLeftY()) > kStickCancelDeadband
        || Math.abs(controller.getRightX()) > kStickCancelDeadband
        || Math.abs(controller.getRightY()) > kStickCancelDeadband;
  }

  public void startPitRoutine() {
    PitTestRoutine pitSubsystemRoutine =
        new PitTestRoutine(elevatorSubsystem, intakeSubsystem, swerveSubsystem, armSubsystem);
    pitSubsystemRoutine.runPitRoutine();
  }

  // --TOGGLE STATE--
  // NOTICE: ONLY FOR AUTO TO CHANGE LATCH
  public void setGamePiece(GamePiece piece) {
    currentPiece = piece;
  }

  public boolean isCurrentPieceCone() {
    return GamePiece.CONE.equals(currentPiece);
  }

  public void toggleGamePiece() {
    if (currentPiece == GamePiece.CONE) {
      currentPiece = GamePiece.CUBE;
      new SetAllColor(ledSubsystem, kCube).schedule();
    } else {
      currentPiece = GamePiece.CONE;
      new SetAllColor(ledSubsystem, kCone).schedule();
    }
    System.out.println("Current game piece: " + currentPiece);
  }

  // --LOG N SIM--
  public void updateSimulation() {
    robotSimulation.updateSubsystemPositions();
  }

  @Override
  public ShuffleboardLayout getLayout(String tab) {
    return null;
  }

  @Override
  public boolean CANTest() {
    System.out.println("Testing CAN connections:");
    boolean result = true;
    for (CANTestable subsystem : canBusTestables) result &= subsystem.CANTest();
    System.out.println("CAN fully connected: " + result);
    return result;
  }

  @Override
  public void logInit() {
    SmartDashboard.putData("trajectoryViewer", trajectoryViewer);
    SmartDashboard.putData("waypointViewer", waypointViewer);
    SmartDashboard.putData("swerveViewer", swerveViewer);
  }
}
