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
import static frc.robot.intake.IntakeConstants.*;
import static frc.robot.simulation.SimulationConstants.*;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
import frc.robot.Constants;
import frc.robot.drivers.CANDeviceTester;
import frc.robot.drivers.CANTestable;
import frc.robot.drivers.TalonFXFactory;
import frc.robot.logging.Loggable;

public class Intake extends SubsystemBase implements Loggable, CANTestable {
  private WPI_TalonFX intakeMotor;
  private double simIntakeAngle = 0;

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
    intakeMotor.setInverted(TalonFXInvertType.Clockwise);
    configIntakeCurrentLimit();
  }

  public void configIntakeCurrentLimit() {
    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 60, 0.2));
  }

  public double getIntakeSpeed() {
    return intakeMotor.getMotorOutputPercent();
  }

  public void updateSimIntakePosition() {
    simIntakeAngle += Math.signum(getIntakeSpeed()) * 3;
  }

  public double getSimIntakePosition() {
    return simIntakeAngle;
  }

  public void latchCone() {
    intakeMotor.set(ControlMode.PercentOutput, kLatchConeSpeed);
  }

  public void latchCube() {
    intakeMotor.set(ControlMode.PercentOutput, kLatchCubeSpeed);
  }

  public void configureLatchCurrentLimit(boolean enabled) {
    if (kDebugEnabled) System.out.println("Setting Current Limit Configuration: " + enabled);
    intakeMotor.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(
            enabled, kGamePieceMaxCurrent, kIntakeMaxCurrent, kTriggerThresholdTime));
  }

  public void intakeCone() {
    intakeMotor.set(ControlMode.PercentOutput, kIntakeConeSpeed);
    intakeMotor.getSimCollection().setBusVoltage(kIntakeConeSpeed);
  }

  public void outtakeCone() {
    intakeMotor.set(ControlMode.PercentOutput, kOuttakeConeSpeed);
  }

  public void intakeCube() {
    intakeMotor.set(ControlMode.PercentOutput, kIntakeCubeSpeed);
    intakeMotor.getSimCollection().setBusVoltage(kIntakeCubeSpeed);
  }

  public void intakeCube(double factor) {
    intakeMotor.set(ControlMode.PercentOutput, kIntakeCubeSpeed * factor);
    intakeMotor.getSimCollection().setBusVoltage(kIntakeCubeSpeed);
  }

  public void outtakeCube() {
    intakeMotor.set(ControlMode.PercentOutput, kOuttakeCubeSpeed);
  }

  public boolean isCurrentSpiking() {
    return intakeMotor.getSupplyCurrent() > kIntakeMaxCurrent;
  }

  public void off() {
    intakeMotor.neutralOutput();
  }

  @Override
  public void periodic() {
    if (Constants.kDebugEnabled) {
      SmartDashboard.putNumber("Intake supply current", intakeMotor.getSupplyCurrent());
      SmartDashboard.putNumber("Intake stator current", intakeMotor.getStatorCurrent());
    }
  }

  public void logInit() {
    getLayout(kDriverTabName).add(this);
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

  private MechanismLigament2d intakePivot;
  private MechanismLigament2d intakeWheel;

  public MechanismLigament2d getWrist() {
    return intakePivot;
  }

  public MechanismLigament2d getIntakeWheel() {
    return intakeWheel;
  }

  private void configureSimHardware() {
    intakeMotor = new WPI_TalonFX(kIntakeMotorID);
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(TalonFXInvertType.Clockwise);

    intakePivot =
        new MechanismLigament2d(
            "Intake Wrist", Units.inchesToMeters(2.059), -90, 0, new Color8Bit(Color.kBlack));
    intakeWheel = new MechanismLigament2d("Intake Wheel", 0.1, 0, 10, new Color8Bit(Color.kViolet));
  }

  @Override
  public void simulationPeriodic() {
    updateSimIntakePosition();
    simulationOutputToDashboard();
  }

  private void simulationOutputToDashboard() {
    SmartDashboard.putNumber("Intake current draw", intakeMotor.getStatorCurrent());
    SmartDashboard.putNumber("Intake sim voltage", intakeMotor.getMotorOutputPercent());
  }
}
