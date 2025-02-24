// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RunAlternateEncoderWithGamepad;
import frc.robot.commands.RunPrimaryEncoderWithGamepad;
import frc.robot.commands.SetAlternateEncoderPosition;
import frc.robot.commands.SetPrimaryEncoderPosition;
import frc.robot.subsystems.AlternateEncoderTester;
import frc.robot.subsystems.PrimaryEncoderTester;

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
    private final Optional<AlternateEncoderTester> alternateEncoderTester;
    private final Optional<PrimaryEncoderTester> primaryEncoderTester;

    private final CommandXboxController commandGamepad = new CommandXboxController(0);
    public final XboxController gamepad = new XboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        //alternateEncoderTester = Optional.of(new AlternateEncoderTester())
        alternateEncoderTester = Optional.empty();

        primaryEncoderTester = Optional.of(new PrimaryEncoderTester());
        //primmaryEncoderTester = Optional.empty();

        // Configure the trigger bindings
        configureBindings();

        //Configure default commands:
        configureDefaultCommands();

        SmartDashboard.putData(gamepad);
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
        alternateEncoderTester.ifPresent(this::configureBindings);
        primaryEncoderTester.ifPresent(this::configureBindings);
    }


    private void configureDefaultCommands()
    {
        //alternateEncoderTester.setDefaultCommand(new RunTestMotorWithJoystick(alternateEncoderTester, joystick));
    }

    private void configureBindings(AlternateEncoderTester encoderTester)
    {        
        commandGamepad.x().onTrue(new InstantCommand(() -> encoderTester.resetPosition()));
        commandGamepad.a().onTrue(new SetAlternateEncoderPosition(encoderTester, gamepad));
        commandGamepad.leftStick().onTrue(new RunAlternateEncoderWithGamepad(encoderTester, gamepad));
    }
    
    private void configureBindings(PrimaryEncoderTester encoderTester)
    {        
        commandGamepad.b().onTrue(new InstantCommand(() -> encoderTester.resetPosition()));
        commandGamepad.y().onTrue(new SetPrimaryEncoderPosition(encoderTester, gamepad));
        commandGamepad.rightStick().onTrue(new RunPrimaryEncoderWithGamepad(encoderTester, gamepad));
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
