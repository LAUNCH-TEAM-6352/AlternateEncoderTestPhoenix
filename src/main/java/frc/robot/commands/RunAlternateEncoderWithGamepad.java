// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlternateEncoderTester;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class RunAlternateEncoderWithGamepad extends Command
{
    private final AlternateEncoderTester encoderTester;
    private final CommandXboxController gamepad;

    /**
     * Creates a new ExampleCommand.
     *
     * @param encoderTester
     *            The subsystem used by this command.
     */
    public RunAlternateEncoderWithGamepad(AlternateEncoderTester encoderTester, CommandXboxController gamepad)
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
            // turn on rumble
        }
        else
        {
            // turn off rumble
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
