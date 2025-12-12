//BEYONCEEEEEE
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.GamepieceMode;

public class Elevator extends SubsystemBase {
  public enum ElevatorPosition {
    L1(5),
    L2(9),
    L3(25.5),
    L4(48),
    Stow(0.5);

    public final double setpoint;

    private ElevatorPosition(double setpoint) {
      this.setpoint = setpoint;
    }
  }

  private static final double CALIBRATION_SPEED = -0.1;
  private static final double HARD_STOP_CURRENT_LIMIT = 50;

  private static final double INCREMENT_CONSTANT = 1;
  private static final double DECREMENT_CONSTANT = 1;

  private static final double ELEVATOR_MOTOR_KP = 0.75;
  private static final double ELEVATOR_MOTOR_KI = 0.15; 
  private static final double ELEVATOR_MOTOR_KD = 0;
  private static final double ELEVATOR_PID_VEL = 220;
  private static final double ELEVATOR_PID_ACC = 215;

  private static final double ELEVATOR_MAX_INCHES = 48;
  private static final double ELEVATOR_MAX_ROTATIONS = 36.4;

  private static final double MOTOR_CONVERSION = ELEVATOR_MAX_INCHES / ELEVATOR_MAX_ROTATIONS;

  // In newtons
  private static final double ELEVATOR_STAGE1_WEIGHT_N = 2.053 * 9.81;
  private static final double ELEVATOR_STAGE2_WEIGHT_N = 7.554 * 9.81;
  private static final double ELEVATOR_TOTAL_WEIGHT_N = ELEVATOR_STAGE1_WEIGHT_N + (ELEVATOR_STAGE2_WEIGHT_N);

  // In m
  private static final double SPOOL_DIAMETER = 0.0527;

  private static final double ELEVATOR_STALL_TORQUE_LB_IN = 3.6;
  private static final double ELEVATOR_STALL_CURRENT = 211;
  private static final double ELEVATOR_KT = ELEVATOR_STALL_TORQUE_LB_IN / ELEVATOR_STALL_CURRENT;
  private static final double GEAR_RATIO = 7.75;
  private static final double NUMBER_OF_MOTORS = 2;
  private static final double EFFECTIVE_KT = ELEVATOR_KT * NUMBER_OF_MOTORS * GEAR_RATIO;
  private static final double ELEVATOR_RESISTANCE = 0.057;

  private static final double FEEDFORWARD_CONSTANT = ((ELEVATOR_TOTAL_WEIGHT_N * SPOOL_DIAMETER * ELEVATOR_RESISTANCE) / (EFFECTIVE_KT)) - 0.65;

  private boolean _isCalibrated;
  private ElevatorPosition _currentPos;
  private ElevatorPosition _desiredPos;
  private MultiSubsystemCommands.GamepieceMode _currentMode;

  private ProfiledPIDController _elevatorMotorPID;

  private ElevatorIO _io;
  private ElevatorIOInputsAutoLogged _inputs;

  

  SysIdRoutine routine = new SysIdRoutine(new Config(),
      new SysIdRoutine.Mechanism(this::setElevatorVoltage, this::populateLog, this));

  public Elevator(ElevatorIO io) {
    _io = io;
    _inputs = new ElevatorIOInputsAutoLogged();

    _elevatorMotorPID = new ProfiledPIDController(ELEVATOR_MOTOR_KP, ELEVATOR_MOTOR_KI, ELEVATOR_MOTOR_KD,
        new Constraints(ELEVATOR_PID_VEL, ELEVATOR_PID_ACC));
    _elevatorMotorPID.setTolerance(0.5);

    _currentPos = ElevatorPosition.Stow;
    _desiredPos = ElevatorPosition.Stow;
  }

  // Runs the motors down at the calibration speed
  public void runMotorsDown() {
    _io.setElevatorSpeed(CALIBRATION_SPEED);
  }

  public void stopMotors() {
    _io.setElevatorSpeed(0);
  }

  public void setElevatorVoltage(Voltage voltage) {
    _io.setElevatorVoltage(voltage);
  }

  public void setSetpoint(ElevatorPosition setpoint) {
    setSetpoint(setpoint.setpoint);
    _desiredPos = setpoint;
  }

  // Sets the setpoint of the PID
  public void setSetpoint(double setpoint) {
    
  } 
  
  // Checks if it is at the setpoint
  public boolean isAtSetpoint() {
    boolean atSetpoint = Math.abs(_elevatorMotorPID.getGoal().position - getCurrentPosInches()) <= 0.5;
    if (atSetpoint)
      _currentPos = _desiredPos;
    return atSetpoint;
  }

  // Increases elevator position
  public void incrementElevatorPosition() {
    setSetpoint(_elevatorMotorPID.getGoal().position + INCREMENT_CONSTANT);
  }

  // Decreases elevator position
  public void decrementElevatorPosition() {
    setSetpoint(_elevatorMotorPID.getGoal().position - DECREMENT_CONSTANT);
  }

  // Checks if above limit
  public boolean isAboveCurrentLimit() {
    return _inputs._elevatorMotorCurrent > HARD_STOP_CURRENT_LIMIT;
  }

  // Sets the encoder values to 0
  public void resetEncoders() {
    _io.resetEncoder();
    _elevatorMotorPID.reset(getCurrentPosInches()); // TODO: Make it so this runs when auto and teleop init
  }

  // Runs motors with PID
  public void runMotorsWithPID() {
  // _io.setElevatorSpeed();
    _io.setElevatorVoltage(Voltage.ofBaseUnits(_elevatorMotorPID.calculate(getCurrentPosInches()) + FEEDFORWARD_CONSTANT, Volt));
  }

  public boolean isCalibrated() {
    return _isCalibrated;
  }

  public void setIsCalibrated(boolean isCalibrated) {
    _isCalibrated = isCalibrated;
  }

  // Converts the motors position from weird units to normal people inches
  public double getCurrentPosInches() {
    return (_inputs._elevatorPosition * MOTOR_CONVERSION) - 0.25;
  }

  public ElevatorPosition getCurrentPos() {
    return _currentPos;
  }

  public GamepieceMode getCurrentMode() {
    return _currentMode;
  }

  public void setCurrentMode(GamepieceMode mode) {
    _currentMode = mode;
  }

  public double getElevatorMaxHeight() {
    return ELEVATOR_MAX_INCHES;
  }

  public void populateLog(SysIdRoutineLog log) {
    log.motor("elevator_primary")
        .voltage(Voltage.ofBaseUnits(_inputs._elevatorMotorVoltage, Volt))
        .linearPosition(Distance.ofBaseUnits(Units.inchesToMeters(getCurrentPosInches()), Meters))
        .linearVelocity(
            LinearVelocity.ofBaseUnits(_inputs._elevatorVelocity * SPOOL_DIAMETER * Math.PI / 60, MetersPerSecond));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  @Override
  public void periodic() {
    _io.updateInputs(_inputs);
    Logger.processInputs("Elevator", _inputs);

    // This method will be called once per scheduler run
    Logger.recordOutput("Elevator/currentPos", getCurrentPosInches());
    Logger.recordOutput("Elevator/targetPos", _elevatorMotorPID.getGoal().position);
    Logger.recordOutput("Elevator/isAboveCurrentLimit", isAboveCurrentLimit());
    Logger.recordOutput("Elevator/Current", _inputs._elevatorMotorCurrent);
    Logger.recordOutput("Elevator/motorSpeed", _inputs._elevatorSpeed);
    Logger.recordOutput("Elevator/CurrentSetpoint", _elevatorMotorPID.getSetpoint().position);
    Logger.recordOutput("Elevator/atSetpoint", isAtSetpoint());
    Logger.recordOutput("Elevator/currentPosEnum", _currentPos);
    Logger.recordOutput("Elevator/desiredPosEnum", _desiredPos);
  }

}
