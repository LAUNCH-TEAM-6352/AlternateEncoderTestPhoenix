// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrimaryEncoderTester extends SubsystemBase
{
    private final SparkMax motor = new SparkMax(1, MotorType.kBrushless);

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
        EncoderConfig encoderConfig =
            new EncoderConfig()
                .positionConversionFactor(1)
                .velocityConversionFactor(1); // velocity will be measured in hex shaft rotations per minuite

        SoftLimitConfig softLimitConfig = 
            new SoftLimitConfig()
                .forwardSoftLimit(maxPosition)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(minPosition)
                .reverseSoftLimitEnabled(true);

        ClosedLoopConfig closedLoopConfig =
            new ClosedLoopConfig()
                .pidf(0.15, 0.0, 0.0, 0.0)
                .iZone(0.0)
                .outputRange(-0.5, +0.5)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        config
            .apply(encoderConfig)
            .apply(softLimitConfig)
            .apply(closedLoopConfig);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.clearFaults();
        resetPosition();
    }

    public void setMotorSpeed(double speed)
    {
        motor.set(speed);
    }

    public double getPosition()
    {
        return motor.getEncoder().getPosition();
    }

    public void resetPosition()
    {
        motor.getEncoder().setPosition(0);
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
        motor.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
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
        SmartDashboard.putNumber("Pri RPM", motor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Pri Spd", motor.getAppliedOutput());

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
