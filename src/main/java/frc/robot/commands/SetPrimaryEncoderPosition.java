// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PrimaryEncoderTester;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetPrimaryEncoderPosition extends Command

{
    private final PrimaryEncoderTester encoderTester;
    private final XboxController gamepad;

    private final HashMap<Integer, Double> positionMap = new HashMap<Integer, Double>()
    {
        {
            put(-1, 0.0);
            put(0, 1024.0);
            put(90, 2048.0);
            put(180, 3072.0);
            put(270, 4096.0);
        }
    };

    /** Creates a new SetTesterPosition. */
    public SetPrimaryEncoderPosition(PrimaryEncoderTester encoderTester, XboxController gamepad)
    {
        this.encoderTester = encoderTester;
        this.gamepad = gamepad;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(encoderTester);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        encoderTester.setPosition(positionMap.get(gamepad.getPOV()), .2);
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
        return encoderTester.atTargetPosition();
    }
}
