// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlternateEncoderTester extends SubsystemBase
{
    private final TalonFX motor = new TalonFX(1);
    private final CANcoder canCoder = new CANcoder(0);
    private final PositionVoltage motorPositionRequest = new PositionVoltage(0).withSlot(0).with;

    private double targetPosition;
    private double targetTolerance;
    private boolean atTargetPosition;
    private boolean isPositioningStarted;
    private double lastPosition;
    public final double minPosition = 0;
    public final double maxPosition = 20;

    /** Creates a new ExampleSubsystem. */
    public AlternateEncoderTester()
    {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.Slot0.kP = 0.15;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.MotorOutput.PeakForwardDutyCycle = 0.1;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -0.1;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxPosition;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minPosition;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motor.getConfigurator().apply(motorConfig);

        var canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoder.getConfigurator().apply(canCoderConfig);

        motor.clearStickyFaults();
        resetPosition();
    }

    public void setMotorSpeed(double speed)
    {
        motor.set(speed);
    }

    public double getPosition()
    {
        return canCoder.getPosition().getValueAsDouble();
    }

    public void resetPosition()
    {
        canCoder.setPosition(0);
    }

    public void setPosition(double position, double tolerance)
    {
        if (position > maxPosition)
        {
            position = maxPosition;
        }
        else if (position < minPosition)
        {
            position = minPosition;
        }
        
        targetPosition = position;
        targetTolerance = tolerance;
        lastPosition = getPosition();

        motor.setControl(motorPositionRequest.withPosition(targetPosition));
        atTargetPosition = false;
        isPositioningStarted = true;
    }

    public boolean atTargetPosition()
    {
        return atTargetPosition;
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        var position = getPosition();

        SmartDashboard.putNumber("Alt Pos", position);
        SmartDashboard.putNumber("Alt RPM", canCoder.getVelocity().getValueAsDouble() / 60.0);
        SmartDashboard.putNumber("Alt Spd", motor.getClosedLoopOutput().getValueAsDouble());

        if (isPositioningStarted)
        {
            if ((Math.abs(position - targetPosition) < targetTolerance) && Math.abs(position - lastPosition) < targetTolerance)
            {
                atTargetPosition = true;
                isPositioningStarted = false;
            }
            else
            {
                lastPosition = position;
            }
        }
    }
}
