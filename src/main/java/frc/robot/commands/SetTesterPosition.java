// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlternateEncoderTester;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetTesterPosition extends Command

{
    private final AlternateEncoderTester alternateEncoderTester;
    private final double position;

    /** Creates a new SetTesterPosition. */
    public SetTesterPosition(AlternateEncoderTester alternateEncoderTester, double position)
    {
        this.alternateEncoderTester = alternateEncoderTester;
        this.position = position;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(alternateEncoderTester);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        alternateEncoderTester.setPosition(position, 0.2);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return alternateEncoderTester.atTargetPosition();
    }
}
