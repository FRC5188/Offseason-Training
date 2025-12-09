package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.HardwareConstants.CAN;
import frc.robot.HardwareConstants.DIO;

public class RealArmIO implements ArmIO {

    private static final double POS_AT_90 = 0.447;
    private static final double POS_AT_0 = 0.701;
    private static final double ENCODER_CONVERSION = (POS_AT_90 - POS_AT_0) / 90.0;
    
    private double INTAKE_ROTATION_CONVERSION = 1; 

    private SparkFlex _armMotor;
    private DigitalInput _lightSensor;
    private SparkFlex _intakeMotor;
    private SparkAbsoluteEncoder _armEncoder;
    private SparkMax _feederMotor;

    public RealArmIO() {
        _armMotor = new SparkFlex(CAN.ARM_MTR_ID, MotorType.kBrushless);
        _intakeMotor = new SparkFlex(CAN.INTAKE_MTR_ID, MotorType.kBrushless);
        _lightSensor = new DigitalInput(DIO.LIGHT_SENSOR_CHANNEL);
        _armEncoder = _armMotor.getAbsoluteEncoder();

        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.idleMode(IdleMode.kBrake);
        armConfig.inverted(true);
        armConfig.voltageCompensation(12);
        _armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(ArmIOInputs inputs) {
        inputs._armMotorSpeed = _armMotor.get();
        inputs._armMotorCurrent = _armMotor.getOutputCurrent();
        inputs._armMotorVoltage = _armMotor.getAppliedOutput() * _armMotor.getBusVoltage();

        inputs._feederMotorSpeed = _feederMotor.get();
        inputs._feederMotorCurrent = _feederMotor.getOutputCurrent();
        inputs._feederMotorVoltage = _feederMotor.getAppliedOutput() * _feederMotor.getBusVoltage();

        inputs._lightSensorState = !_lightSensor.get();
        inputs._intakeMotorVelocityRotationsPerMin = _intakeMotor.get();
        inputs._intakeMotorCurrent = _intakeMotor.getOutputCurrent();
        inputs._intakeMotorVoltage = _intakeMotor.getAppliedOutput() * _armMotor.getBusVoltage();
        inputs._intakeMotorPositionRotations = _intakeMotor.getEncoder().getPosition() * INTAKE_ROTATION_CONVERSION; 
        
        inputs._armEncoderPositionDegrees = (_armEncoder.getPosition() - POS_AT_0) / ENCODER_CONVERSION;
        inputs._armEncoderVelocity = _armEncoder.getVelocity();
    }

    public void setArmMotorSpeed(double speed) {
        _armMotor.set(speed);
    }

    public void setIntakeMotorSpeed() {
        // Write the code here.
    }
    
    public void setFeederMotorSpeed(double speed){
        _feederMotor.set(speed);   
    }
    public void resetIntakeEncoders() {
        _intakeMotor.getEncoder().setPosition(0);
    }

    public void setArmMotorVoltage(Voltage voltage) {
        _armMotor.setVoltage(voltage);
    }

    public void setFeederMotorVoltage(Voltage voltage) {
        _feederMotor.setVoltage(voltage);
    }
}
