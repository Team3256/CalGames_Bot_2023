// Copyright (c) 2023 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.intake;

import static frc.robot.Constants.ShuffleboardConstants.kDriverTabName;
import static frc.robot.Constants.ShuffleboardConstants.kIntakeLayoutName;
import static frc.robot.Constants.kDebugEnabled;
import static frc.robot.arm.Arm.getArmPositionRads;
import static frc.robot.intake.IntakeConstants.*;
import static frc.robot.simulation.SimulationConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.logging.Loggable;

public class Intake extends SubsystemBase implements Loggable, CANTestable {
  private WPI_TalonFX intakeMotor;

  public Intake() {
    if (RobotBase.isReal()) {
      configureRealHardware();
    } else {
      configureSimHardware();
    }
    off();
    System.out.println("Intake initialized");
  }

  private void configureRealHardware() {
    intakeMotor = TalonFXFactory.createDefaultTalon(kIntakeCANDevice);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public double getIntakeSpeed() {
    return intakeMotor.getMotorOutputPercent();
  }

  public void latchCone() {
    intakeMotor.set(ControlMode.PercentOutput, kLatchConeSpeed);
  }

  public void latchCube() {
    intakeMotor.set(ControlMode.PercentOutput, kLatchCubeSpeed);
  }

  public void configureCurrentLimit(boolean enabled) {
    if (kDebugEnabled) System.out.println("Setting Current Limit Configuration: " + enabled);
    intakeMotor.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(
            enabled, kGamePieceMaxCurrent, kIntakeMaxCurrent, kTriggerThresholdTime));
  }

  public void intakeCone() {
    intakeMotor.set(ControlMode.PercentOutput, kIntakeConeSpeed);
    intakeMotor.getSimCollection().setBusVoltage(kIntakeConeSpeed);
  }

  public void outakeCone() {
    System.out.println("Outake cone");
    intakeMotor.set(ControlMode.PercentOutput, kOutakeConeSpeed);
  }

  public void intakeCube() {
    intakeMotor.set(ControlMode.PercentOutput, kIntakeCubeSpeed);
    intakeMotor.getSimCollection().setBusVoltage(kIntakeCubeSpeed);
  }

  public void outakeCube() {
    System.out.println("Outake cube");
    intakeMotor.set(ControlMode.PercentOutput, kOutakeCubeSpeed);
  }

  public boolean isCurrentSpiking() {
    return intakeMotor.getStatorCurrent() > kIntakeMaxCurrent;
  }

  public void off() {
    intakeMotor.neutralOutput();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake supply current", intakeMotor.getSupplyCurrent());
    SmartDashboard.putNumber("Intake stator current", intakeMotor.getStatorCurrent());
  }

  public void logInit() {
    getLayout(kDriverTabName).add(this);
    getLayout(kDriverTabName).add(new IntakeCube(this));
    getLayout(kDriverTabName).add(new IntakeCone(this));
    getLayout(kDriverTabName).add(intakeMotor);
  }

  public ShuffleboardLayout getLayout(String tab) {
    return Shuffleboard.getTab(tab)
        .getLayout(kIntakeLayoutName, BuiltInLayouts.kList)
        .withSize(2, 4);
  }

  public boolean CANTest() {
    System.out.println("Testing intake CAN:");
    boolean result = CANDeviceTester.testTalonFX(intakeMotor);
    System.out.println("Intake CAN connected: " + result);
    SmartDashboard.putBoolean("Intake CAN connected", result);
    return result;
  }

  // sim
  private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 1);
  private MechanismLigament2d intakePivot;

  public MechanismLigament2d getWrist() {
    return intakePivot;
  }

  private void configureSimHardware() {
    intakeMotor = new WPI_TalonFX(kIntakeMotorID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);

    intakePivot =
        new MechanismLigament2d(
            "Intake Wrist",
            Units.inchesToMeters(2.059),
            Units.radiansToDegrees(getArmPositionRads()) * (86.058 / 180) - 90,
            0,
            new Color8Bit(Color.kBlack));
  }

  @Override
  public void simulationPeriodic() {
    intakeSim.setInput(intakeMotor.getMotorOutputPercent() * kVoltage);
    intakeSim.update(kSimulateDelta);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(intakeSim.getCurrentDrawAmps()));
    simulationOutputToDashboard();

    // TODO what are these numbers?
    intakePivot.setAngle(Units.radiansToDegrees(getArmPositionRads()) * (86.058 / 180) - 90);
  }

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber(
        "Intake angle deg", Units.radiansToDegrees(intakeSim.getAngularPositionRad()));
    SmartDashboard.putNumber("Intake current draw", intakeMotor.getStatorCurrent());
    SmartDashboard.putNumber("Intake sim voltage", intakeMotor.getMotorOutputPercent());
  }
}
