// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.AlternateEncoderTester;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/** An example command that uses an example subsystem. */
public class RunTestMotorWithJoystick extends Command
{
    private final AlternateEncoderTester alternateEncoderTester;
    private final CommandJoystick joystick;

    /**
     * Creates a new ExampleCommand.
     *
     * @param alternateEncoderTester
     *            The subsystem used by this command.
     */
    public RunTestMotorWithJoystick(AlternateEncoderTester alternateEncoderTester, CommandJoystick joystick)
    {
        this.alternateEncoderTester = alternateEncoderTester;
        this.joystick = joystick;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(alternateEncoderTester);
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
        var speed = -joystick.getThrottle() / 2.0;
        var position = alternateEncoderTester.getPosition();
        if ((speed < 0 && position <= AlternateEncoderTester.minPosition) ||
            (speed > 0 && position >= AlternateEncoderTester.maxPosition))
        {
            speed = 0;
            // turn on rumble
        }
        else
        {
            // turn off rumble
        }

        alternateEncoderTester.setMotorSpeed(speed);
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
