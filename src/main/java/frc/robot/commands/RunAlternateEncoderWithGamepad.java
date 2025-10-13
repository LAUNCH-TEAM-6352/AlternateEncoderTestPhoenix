// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlternateEncoderTester;

/** An example command that uses an example subsystem. */
public class RunAlternateEncoderWithGamepad extends Command
{
    private final AlternateEncoderTester encoderTester;
    private final XboxController gamepad;

    /**
     * Creates a new ExampleCommand.
     *
     * @param encoderTester
     *            The subsystem used by this command.
     */
    public RunAlternateEncoderWithGamepad(AlternateEncoderTester encoderTester, XboxController gamepad)
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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        var speed = -gamepad.getLeftY() / 2.0;
        var position = encoderTester.getPosition();
        if ((speed < 0 && position <= encoderTester.minPosition) ||
            (speed > 0 && position >= encoderTester.maxPosition))
        {
            speed = 0;
            gamepad.setRumble(RumbleType.kBothRumble, 1);
        }
        else
        {
            gamepad.setRumble(RumbleType.kBothRumble, 0);
        }

        encoderTester.setMotorSpeed(speed);
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
        return false;
    }
}
