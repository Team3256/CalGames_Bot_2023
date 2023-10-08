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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeatureFlags;
import frc.robot.arm.Arm;
import frc.robot.arm.commands.SetArmVoltage;
import frc.robot.arm.commands.ZeroArmSensor;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.auto.pathgeneration.commands.AutoIntakePrep;
import frc.robot.auto.pathgeneration.commands.AutoScorePrep;
import frc.robot.drivers.CANTestable;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.*;
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

  private final SwerveDrive swerveSubsystem;
  private final Intake intakeSubsystem;
  private final Elevator elevatorSubsystem;
  private final Arm armSubsystem;
  private final LED ledSubsystem;

  private GamePiece currentPiece = GamePiece.CONE;

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

    driver
        .leftTrigger()
        .toggleOnTrue(
            new TeleopSwerveLimited(
                swerveSubsystem,
                driver::getLeftY,
                driver::getLeftX,
                driver::getRightX,
                kFieldRelative,
                kOpenLoop));

    driver
        .x()
        .onTrue(
            new LockSwerveX(swerveSubsystem)
                .andThen(new SetAllColor(ledSubsystem, kLockSwerve))
                .until(() -> isMovingJoystick(driver)));

    if (kElevatorEnabled && kArmEnabled && kLedStripEnabled && kIntakeEnabled) {
      new Trigger(this::scoreTriggered)
          .onTrue(
              new AutoScorePrep(
                  swerveSubsystem,
                  elevatorSubsystem,
                  armSubsystem,
                  this::isCurrentPieceCone,
                  () -> isMovingJoystick(driver)));

      new Trigger(this::intakeTriggered)
          .onTrue(
              new AutoIntakePrep(
                  swerveSubsystem,
                  intakeSubsystem,
                  elevatorSubsystem,
                  armSubsystem,
                  this::isCurrentPieceCone,
                  () -> isMovingJoystick(driver)));
    }
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
    operator
        .povLeft()
        .whileTrue(
            new ConditionalCommand(
                new OuttakeConeOrCube(intakeSubsystem, true, true),
                new OuttakeConeOrCube(intakeSubsystem, true, false),
                this::isCurrentPieceCone));
    operator
        .povRight()
        .whileTrue(
            new ConditionalCommand(
                new IntakeConeOrCube(intakeSubsystem, true, true),
                new IntakeConeOrCube(intakeSubsystem, true, false),
                this::isCurrentPieceCone));
    operator.povDown().onTrue(new InstantCommand(this::toggleGamePiece)); // toggle

    if (kArmEnabled && kElevatorEnabled) {
      driver // intake
          .leftBumper()
          .whileTrue(
              new ConditionalCommand(
                  new IntakeConeOrCube(intakeSubsystem, true, true),
                  new IntakeConeOrCube(intakeSubsystem, true, false),
                  this::isCurrentPieceCone));
      driver // outtake
          .rightBumper()
          .whileTrue(
              new ConditionalCommand(
                  new OuttakeConeOrCube(intakeSubsystem, true, true),
                  new OuttakeConeOrCube(intakeSubsystem, true, false),
                  this::isCurrentPieceCone));
    }
  }

  private void configureArm() {
    //    armSubsystem.setDefaultCommand(new KeepArm(armSubsystem));
    operator.rightBumper().whileTrue(new SetArmVoltage(armSubsystem, 4.5));
    operator.leftBumper().whileTrue(new SetArmVoltage(armSubsystem, -4.5));
    tester.povUp().whileTrue(new SetArmVoltage(armSubsystem, 4.5));
    tester.povDown().whileTrue(new SetArmVoltage(armSubsystem, -4.5));
  }

  public void configureElevator() {
    //    elevatorSubsystem.setDefaultCommand(new KeepElevator(elevatorSubsystem));
    operator.rightTrigger().whileTrue(new SetElevatorVolts(elevatorSubsystem, 6)); // manual +
    operator.leftTrigger().whileTrue(new SetElevatorVolts(elevatorSubsystem, -6)); // manual -
    tester.povRight().whileTrue(new SetElevatorVolts(elevatorSubsystem, 6)); // manual +
    tester.povLeft().whileTrue(new SetElevatorVolts(elevatorSubsystem, -6)); // manual -
    if (kArmEnabled) {
      driver.y().onTrue(new StowEndEffector(elevatorSubsystem, armSubsystem)); // stow
      operator.y().onTrue(new StowEndEffector(elevatorSubsystem, armSubsystem)); // stow
      tester.leftBumper().onTrue(new StowEndEffector(elevatorSubsystem, armSubsystem)); // stow
      operator // double sub preset
          .povUp()
          .onTrue(
              new ConditionalCommand(
                  new SetEndEffectorState(
                      elevatorSubsystem,
                      armSubsystem,
                      SetEndEffectorState.EndEffectorPreset.DOUBLE_SUBSTATION_CONE,false),
                  new SetEndEffectorState(
                      elevatorSubsystem,
                      armSubsystem,
                      SetEndEffectorState.EndEffectorPreset.DOUBLE_SUBSTATION_CUBE, false),
                  this::isCurrentPieceCone));
      operator.a().onTrue(new ZeroArmSensor(armSubsystem)); // zero
      driver // zero/cube/fallen cone
          .leftBumper()
          .onTrue(new ZeroEndEffector(elevatorSubsystem, armSubsystem));

      driver // zero/cube/standing cone
          .rightBumper()
          .onTrue(
              new ConditionalCommand(
                  new SetEndEffectorState(
                      elevatorSubsystem,
                      armSubsystem,
                      SetEndEffectorState.EndEffectorPreset.STANDING_CONE_GROUND_INTAKE),
                  new ZeroEndEffector(elevatorSubsystem, armSubsystem),
                  this::isCurrentPieceCone));

      operator // zero/cube/standing cone
          .b()
          .onTrue(
              new ConditionalCommand(
                  new SetEndEffectorState(
                      elevatorSubsystem,
                      armSubsystem,
                      SetEndEffectorState.EndEffectorPreset.STANDING_CONE_GROUND_INTAKE),
                  new ZeroEndEffector(elevatorSubsystem, armSubsystem),
                  this::isCurrentPieceCone));
    }
  }

  public void configureLEDStrip() {
    ledSubsystem.setDefaultCommand(new ColorFlowPattern(ledSubsystem));
  }

  // --MISC--
  public Command getAutonomousCommand() {
    Command autoPath = autoPaths.getSelectedPath();
    //    if (kElevatorEnabled && kArmEnabled) {
    //      return Commands.sequence(
    //          new StowEndEffector(elevatorSubsystem, armSubsystem),
    //          autoPath,
    //          Commands.parallel(
    //              new StowEndEffector(elevatorSubsystem, armSubsystem).asProxy(),
    //              new LockSwerveX(swerveSubsystem)
    //                  .andThen(new SetAllColor(ledSubsystem, kLockSwerve))
    //                  .until(() -> isMovingJoystick(driver))));
    //    } else {
    return autoPath;
    //    }
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

  // --GamePiece?--
  public void setGamePiece(GamePiece piece) {
    currentPiece = piece;
    if (piece.equals(GamePiece.CUBE)) new SetAllColor(ledSubsystem, kCube).schedule();
    else new SetAllColor(ledSubsystem, kCone).schedule();
  }

  public void toggleGamePiece() {
    if (currentPiece.equals(GamePiece.CUBE)) {
      setGamePiece(GamePiece.CONE);
    } else {
      setGamePiece(GamePiece.CUBE);
    }
  }

  public boolean isCurrentPieceCone() {
    return GamePiece.CONE.equals(currentPiece);
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
