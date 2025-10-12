// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrimaryEncoderTester extends SubsystemBase
{
    private final TalonFX motor = new TalonFX(1);

    private double targetPosition;
    private double targetTolerance;
    private boolean atTargetPosition;
    private boolean isPositioningStarted;
    private double lastPosition;
    public final double minPosition = 0;
    public final double maxPosition = 4096;

    /** Creates a new ExampleSubsystem. */
    public PrimaryEncoderTester()
    {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = 0.15;
        motorConfig.Slot0.kI = 0.0;
        motorConfig.Slot0.kD = 0.0;
        motorConfig.MotorOutput.PeakForwardDutyCycle = 0.5;
        motorConfig.MotorOutput.PeakReverseDutyCycle = -0.5;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxPosition;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minPosition;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motor.getConfigurator().apply(motorConfig);
        
        motor.clearStickyFaults();
        resetPosition();
    }

    public void setMotorSpeed(double speed)
    {
        motor.set(speed);
    }

    public double getPosition()
    {
        return motor.getPosition().getValueAsDouble();
    }

    public void resetPosition()
    {
        motor.setPosition(0);
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
        motor.setControl(new PositionDutyCycle(targetPosition).withSlot(0));
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

        SmartDashboard.putNumber("Pri Pos", position);
        SmartDashboard.putNumber("Pri RPM", motor.getVelocity().getValueAsDouble() / 60.0);
        SmartDashboard.putNumber("Pri Spd", motor.getClosedLoopOutput().getValueAsDouble());

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
