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
import frc.robot.arm.ArmConstants;
import frc.robot.arm.commands.KeepArm;
import frc.robot.arm.commands.SetArmVoltage;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoPaths;
import frc.robot.auto.pathgeneration.commands.*;
import frc.robot.auto.pathgeneration.commands.AutoIntakeAtDoubleSubstation.SubstationLocation;
import frc.robot.drivers.CANTestable;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.SetEndEffectorState;
import frc.robot.elevator.commands.SetEndEffectorState.EndEffectorPreset;
import frc.robot.elevator.commands.StowEndEffector;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.*;
import frc.robot.intake.commands.GroundIntake.ConeOrientation;
import frc.robot.led.LED;
import frc.robot.led.commands.LimitedSwervePattern;
import frc.robot.led.commands.SetAllBlink;
import frc.robot.led.commands.SetAllColor;
import frc.robot.logging.Loggable;
import frc.robot.simulation.RobotSimulation;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.LockSwerveX;
import frc.robot.swerve.commands.TeleopSwerve;
import frc.robot.swerve.commands.TeleopSwerveLimited;
import frc.robot.swerve.commands.TeleopSwerveWithAzimuth;
import java.util.ArrayList;
import frc.robot.auto.pathgeneration.commands.AutoScore.*;

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

  private final ArrayList<CANTestable> canBusTestables = new ArrayList<CANTestable>();
  private final ArrayList<Loggable> loggables = new ArrayList<Loggable>();

  public RobotContainer() {
    if (kArmEnabled) armSubsystem = new Arm();
    if (kIntakeEnabled) intakeSubsystem = new Intake();
    if (kElevatorEnabled) elevatorSubsystem = new Elevator();
    if (kSwerveEnabled) swerveSubsystem = new SwerveDrive();
    if (kLedStripEnabled) ledSubsystem = new LED();

    if (kLedStripEnabled) {
      configureLEDStrip();
    }
    if (kIntakeEnabled) {
      configureIntake();
      canBusTestables.add(intakeSubsystem);
      loggables.add(intakeSubsystem);
    }
    if (kArmEnabled) {
      configureArm();
      canBusTestables.add(armSubsystem);
      loggables.add(armSubsystem);
    }
    if (kElevatorEnabled) {
      configureElevator();
      canBusTestables.add(elevatorSubsystem);
      loggables.add(elevatorSubsystem);
    }
    if (kSwerveEnabled) {
      configureSwerve();
      canBusTestables.add(swerveSubsystem);
      loggables.add(swerveSubsystem);
    }

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
      robotSimulation.addDoubleSubstation(GamePiece.CONE);
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
        .leftBumper()
        .toggleOnTrue(
            new TeleopSwerveLimited(
                    swerveSubsystem,
                    driver::getLeftY,
                    driver::getLeftX,
                    driver::getRightX,
                    kFieldRelative,
                    kOpenLoop)
                .deadlineWith(new LimitedSwervePattern(ledSubsystem, this::isCurrentPieceCone)));

    driver
        .x()
        .onTrue(
            new LockSwerveX(swerveSubsystem)
                .andThen(new SetAllColor(ledSubsystem, kLockSwerve))
                .until(() -> isMovingJoystick(driver)));

    if (kElevatorEnabled && kArmEnabled && kLedStripEnabled) {

       driver
       .leftTrigger()
       .onTrue(
       new AutoIntakeAtDoubleSubstation(
       swerveSubsystem,
       intakeSubsystem,
       elevatorSubsystem,
       armSubsystem,
       ledSubsystem,
       () -> doubleSubstationLocation,
       () -> isMovingJoystick(driver),
       () -> modeChooser.getSelected().equals(Mode.AUTO_SCORE),
       this::isCurrentPieceCone));

       driver
       .rightTrigger()
       .onTrue(
               new GroundIntake(elevatorSubsystem, armSubsystem, intakeSubsystem, ConeOrientation.STANDING_CONE, this::isCurrentPieceCone)
       );

      driver
              .rightBumper()
              .onTrue(
                      new GroundIntake(elevatorSubsystem, armSubsystem, intakeSubsystem, ConeOrientation.SITTING_CONE, this::isCurrentPieceCone)
              );


       driver
       .y()
       .onTrue(
       new AutoScore(
       swerveSubsystem,
       intakeSubsystem,
       elevatorSubsystem,
       armSubsystem,
       ledSubsystem,
       AutoScore.GridScoreHeight.LOW,
       this::isCurrentPieceCone,
       () -> false,
       () -> false));

       //B Auto Score

       driver
       .b()
       .onTrue(
       new AutoScore(
       swerveSubsystem,
       intakeSubsystem,
       elevatorSubsystem,
       armSubsystem,
       ledSubsystem,
       currentScoringPreset,
       this::isCurrentPieceCone,
       () -> modeChooser.getSelected().equals(Mode.AUTO_SCORE),
       () -> isMovingJoystick(driver)));

      operator.rightBumper().onTrue(new InstantCommand(() -> setScoreLocation(GridScoreHeight.HIGH)));
      operator.povUp().onTrue(new InstantCommand(() -> setScoreLocation(GridScoreHeight.MID)));
      operator.povDown().onTrue(new InstantCommand(() -> setScoreLocation(GridScoreHeight.LOW)));
    }

    operator.a().toggleOnTrue(new InstantCommand(this::toggleSubstationLocation));
  }

  private void configureIntake() {
    driver
        .rightBumper()
        .onTrue(
            new GroundIntake(
                elevatorSubsystem,
                armSubsystem,
                intakeSubsystem,
                ConeOrientation.SITTING_CONE,
                () -> true));
    operator.rightTrigger()
        .whileTrue(
            new ConditionalCommand(
                new OuttakeCone(intakeSubsystem),
                new OuttakeCube(intakeSubsystem),
                this::isCurrentPieceCone))
        .onFalse(new StowEndEffector(elevatorSubsystem, armSubsystem, this::isCurrentPieceCone));
  }

  public void configureElevator() {
    if (kArmEnabled) {
      operator.leftTrigger().onTrue(new StowEndEffector(elevatorSubsystem, armSubsystem, this::isCurrentPieceCone));
    }
  }

  private void configureArm() {
    armSubsystem.setDefaultCommand(new KeepArm(armSubsystem));
    if (kIntakeEnabled && FeatureFlags.kOperatorManualArmControlEnabled) {
      operator.povUp().whileTrue(new SetArmVoltage(armSubsystem, ArmConstants.kManualArmVoltage));
      operator
          .povDown()
          .whileTrue(new SetArmVoltage(armSubsystem, -ArmConstants.kManualArmVoltage));
      operator
          .povUp()
          .or(operator.povDown())
          .whileTrue(
              new ConditionalCommand(
                  new IntakeCone(intakeSubsystem),
                  new IntakeCube(intakeSubsystem),
                  this::isCurrentPieceCone));
    }
  }

  public void configureLEDStrip() {
    ledSubsystem.setDefaultCommand(new SetAllColor(ledSubsystem, kCube));
    operator
        .leftBumper()
        .toggleOnTrue(
            new InstantCommand(this::toggleGamePiece)
                .andThen(new InstantCommand(this::updateGamePieceLEDs)));
  }

  public Command getAutonomousCommand() {
    if (true) {
      return new SetEndEffectorState(
          elevatorSubsystem, armSubsystem, EndEffectorPreset.DOUBLE_SUBSTATION_CONE);
      // return new GroundIntake(
      // elevatorSubsystem,
      // armSubsystem,
      // intakeSubsystem,
      // ConeOrientation.STANDING_CONE,
      // () -> false);
    }

    Command autoPath = autoPaths.getSelectedPath();
    if (kElevatorEnabled && kArmEnabled) {
      return Commands.sequence(
          autoPath,
          Commands.parallel(
              new StowEndEffector(elevatorSubsystem, armSubsystem, this::isCurrentPieceCone)
                  .asProxy(),
              new LockSwerveX(swerveSubsystem)
                  .andThen(new SetAllColor(ledSubsystem, kLockSwerve))
                  .until(() -> isMovingJoystick(driver))));
    } else {
      return autoPath;
    }
  }

  @Override
  public void logInit() {
    SmartDashboard.putData("trajectoryViewer", trajectoryViewer);
    SmartDashboard.putData("waypointViewer", waypointViewer);
    SmartDashboard.putData("swerveViewer", swerveViewer);
  }

  public boolean isMovingJoystick(CommandXboxController controller) {
    return Math.abs(controller.getLeftX()) > kStickCancelDeadband
        || Math.abs(controller.getLeftY()) > kStickCancelDeadband
        || Math.abs(controller.getRightX()) > kStickCancelDeadband
        || Math.abs(controller.getRightY()) > kStickCancelDeadband;
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

  public void startPitRoutine() {
    PitTestRoutine pitSubsystemRoutine =
        new PitTestRoutine(elevatorSubsystem, intakeSubsystem, swerveSubsystem, armSubsystem);
    pitSubsystemRoutine.runPitRoutine();
  }

  public boolean isCurrentPieceCone() {
    return GamePiece.CONE.equals(currentPiece);
  }

  public void toggleGamePiece() {
    if (currentPiece == GamePiece.CONE) {
      currentPiece = GamePiece.CUBE;
    } else {
      currentPiece = GamePiece.CONE;
    }
  }

  public void updateGamePieceLEDs() {
    if (currentPiece == GamePiece.CONE) {
      new SetAllBlink(ledSubsystem, kCone).schedule();
    } else {
      new SetAllBlink(ledSubsystem, kCube).schedule();
    }
  }

  public GamePiece getCurrentPiece() {
    return currentPiece;
  }

  public void toggleSubstationLocation() {
    if (doubleSubstationLocation == SubstationLocation.LEFT_SIDE) {
      doubleSubstationLocation = SubstationLocation.RIGHT_SIDE;
    } else {
      doubleSubstationLocation = SubstationLocation.LEFT_SIDE;
    }

    SmartDashboard.putString(
        "Current Double Substation Location", doubleSubstationLocation.toString());
  }


  public void setScoreLocation(GridScoreHeight Preset) {
    currentScoringPreset = Preset;
    SmartDashboard.putString(
            "Current Scoring Preset", currentScoringPreset.toString());
  }

  public void updateSimulation() {
    robotSimulation.updateSubsystemPositions();
  }

  public SubstationLocation getSubstationLocation() {
    return this.doubleSubstationLocation;
  }
}
