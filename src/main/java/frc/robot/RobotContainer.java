// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/*
 * DRIVER CONTROLS - two Thrustmaster T.16000M flight sticks
 *
 *   LEFT stick  (Driver Station USB slot 0) = translation
 *     Push stick ............. drive (field-centric)
 *     Trigger (button 1) ..... hold: point wheels where the left stick is aimed
 *     Thumb   (button 2) ..... reset field-centric heading (face away from driver station first!)
 *     Base buttons 5-8 ....... SysId (see bottom of configureBindings)
 *
 *   RIGHT stick (Driver Station USB slot 1) = rotation
 *     Push stick left/right .. rotate robot (or twist - see kUseTwistForRotation)
 *     Trigger (button 1) ..... hold: brake / X-lock wheels
 *
 *   Free for future mechanisms: buttons 3-4 and the hat switch on both sticks,
 *   right-stick base buttons, and left-stick base buttons 9-16.
 *
 * Both sticks report the same name, so the Driver Station can swap them after a
 * reboot or re-plug. Check the USB tab before every match!
 */
public class RobotContainer {
    private double MaxSpeed = .6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Set to true to rotate by TWISTING the right stick instead of pushing it left/right. */
<<<<<<< HEAD
    private static final boolean kUseTwistForRotation = true;
=======
    private static final boolean kUseTwistForRotation = false;
>>>>>>> e6b82073e443e74b5df79038f26a9cb2e27a7f4d

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick leftStick = new CommandJoystick(0);  // translation
<<<<<<< HEAD
    private final CommandJoystick rightStick = new CommandJoystick(0); // rotation
=======
    private final CommandJoystick rightStick = new CommandJoystick(1); // rotation
>>>>>>> e6b82073e443e74b5df79038f26a9cb2e27a7f4d

    /*
     * Driver inputs, each scaled -1 to 1. Everything below reads these instead of the
     * sticks directly, so swapping controllers later only means changing these three lines.
     *
     * Flight sticks read NEGATIVE when pushed forward or left, same as an Xbox stick,
     * so the signs match the original Xbox code.
     */
    private final DoubleSupplier forwardInput = () -> -leftStick.getY();  // push forward = drive forward (+X)
    private final DoubleSupplier strafeInput  = () -> -leftStick.getX();  // push left = drive left (+Y)
    private final DoubleSupplier rotateInput  = () -> kUseTwistForRotation
            ? -rightStick.getTwist()  // twist left = counterclockwise
            : -rightStick.getX();     // push left  = counterclockwise

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(forwardInput.getAsDouble() * MaxSpeed)
                    .withVelocityY(strafeInput.getAsDouble() * MaxSpeed)
                    .withRotationalRate(rotateInput.getAsDouble() * MaxAngularRate)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Right trigger: hold to X-lock the wheels (was Xbox A)
        rightStick.trigger().whileTrue(drivetrain.applyRequest(() -> brake));

        // Left trigger: hold to point the wheels in the direction the left stick is aimed (was Xbox B)
        leftStick.trigger().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(forwardInput.getAsDouble(), strafeInput.getAsDouble()))
        ));

        // Left thumb button: reset the field-centric heading (was Xbox left bumper)
        leftStick.top().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // SysId on the left stick base (was Xbox back/start + Y/X).
        // Hold 5 (dynamic) or 6 (quasistatic), then hold 7 (forward) or 8 (reverse).
        // Note that each routine should be run exactly once in a single log.
        leftStick.button(5).and(leftStick.button(7)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        leftStick.button(5).and(leftStick.button(8)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        leftStick.button(6).and(leftStick.button(7)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        leftStick.button(6).and(leftStick.button(8)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}