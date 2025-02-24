// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlternateEncoderTester;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlternateEncoderPosition extends Command

{
    private final AlternateEncoderTester alternateEncoderTester;
    private final double position;

    private final HashMap<Integer, Double> positionMap = new HashMap<Integer, Double>()
    {
        {
            put(-1, 0.0);
            put(0, 5.0);
            put(90, 10.0);
            put(190, 15.0);
            put(270, 20.0);
        }
    };

    /** Creates a new SetTesterPosition. */
    public SetAlternateEncoderPosition(AlternateEncoderTester alternateEncoderTester, XboxController gamepad)
    {
        this.alternateEncoderTester = alternateEncoderTester;

        this.position = positionMap.get(gamepad.getPOV());

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
