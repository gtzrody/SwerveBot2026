// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoLineUp;
import frc.robot.commands.DriveToHub;
import frc.robot.commands.TunerConstants;
import frc.robot.subsystems.HubAlignmentPID;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.hood.Hood;


public class RobotContainer {
    
    //Constants
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    //Subsystems
    private final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Hood m_hood = new Hood();
    private final Shooter m_Shooter = new Shooter();

    //Commands
    private final Command leftHubAutoDrive = new AutoLineUp(drivetrain, 0);
    private final Command rightHubAutoDrive = new AutoLineUp(drivetrain, 1);
    

    private final Telemetry logger = new Telemetry(MaxSpeed);


     private void configureDefaultCommands() {
        // drivetrain.setDefaultCommand(swerveTeleop);
        drivetrain.registerTelemetry(logger::telemeterize);
        m_Shooter.setDefaultCommand(m_Shooter.set(0));
        m_hood.setDefaultCommand(m_hood.set(0));
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //Bindings
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.leftBumper().whileTrue(leftHubAutoDrive);
        joystick.rightBumper().whileTrue(rightHubAutoDrive);
        joystick.leftTrigger().whileTrue(m_Shooter.setVelocity(RPM.of(60)));
        joystick.rightTrigger().whileTrue(m_Shooter.setVelocity(RPM.of(3000)));
        joystick.x().whileTrue(m_hood.set(0.10));
        joystick.y().whileTrue(m_hood.set(-0.10));
        joystick.povLeft().whileTrue(m_hood.setAngle(Degrees.of(-30)));
        joystick.povUp().whileTrue(m_hood.setAngle(Degrees.of(45)));
        joystick.povDown().whileTrue(m_hood.setAngle(Degrees.of(10)));

        
        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();

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
