// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlternateEncoderTester extends SubsystemBase
{
    private final SparkMax motor = new SparkMax(1, MotorType.kBrushless);

    private double targetPosition;
    private double targetTolerance;
    private boolean atTargetPosition;
    private boolean isPositioningStarted;
    private double lastPosition;
    private final double minPosition = 0;
    private final double maxPosition = 100;

    /** Creates a new ExampleSubsystem. */
    public AlternateEncoderTester()
    {
        AlternateEncoderConfig encoderConfig =
            new AlternateEncoderConfig()
                .averageDepth(64)
                .countsPerRevolution(8192)
                .inverted(true) // by default, cw rotation is negative
                .measurementPeriod(100)
                .positionConversionFactor(12) // approx 1 count per inch?
                .setSparkMaxDataPortConfig()
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
                .outputRange(-0.1, +0.1)
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);

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

    private double getPosition()
    {
        return motor.getAlternateEncoder().getPosition();
    }

    public void resetPosition()
    {
        motor.getAlternateEncoder().setPosition(0);
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

        if (isPositioningStarted)
        {
            double position = getPosition();
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
        SmartDashboard.putNumber("Position", motor.getAlternateEncoder().getPosition());
        SmartDashboard.putNumber("Velocity", motor.getAlternateEncoder().getVelocity());
    }
}
