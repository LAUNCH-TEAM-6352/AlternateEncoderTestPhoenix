// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RunTestMotorWithJoystick;
import frc.robot.commands.SetTesterPosition;
import frc.robot.subsystems.AlternateEncoderTester;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final AlternateEncoderTester alternateEncoderTester = new AlternateEncoderTester();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandJoystick joystick = new CommandJoystick(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();

        //Configure default commands:
        configureDefaultCommands();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        joystick.button(2).onTrue(new InstantCommand(() -> alternateEncoderTester.resetPosition()));
        joystick.button(7).onTrue(new SetTesterPosition(alternateEncoderTester, 0));
        joystick.button(8).onTrue(new SetTesterPosition(alternateEncoderTester, 5));
        joystick.button(9).onTrue(new SetTesterPosition(alternateEncoderTester, 10));
        joystick.button(10).onTrue(new SetTesterPosition(alternateEncoderTester, 15));
        joystick.button(11).onTrue(new SetTesterPosition(alternateEncoderTester, 20));
        joystick.button(12).onTrue(new RunTestMotorWithJoystick(alternateEncoderTester, joystick));
    }

    private void configureDefaultCommands()
    {
        //alternateEncoderTester.setDefaultCommand(new RunTestMotorWithJoystick(alternateEncoderTester, joystick));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return null;
    }
}
