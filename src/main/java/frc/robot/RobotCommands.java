package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.FieldPosition;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.ShootingLookupTable;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

public class RobotCommands {
    private DriveSubsystem m_robotDrive;
    private LauncherSubsystem m_launcher;
    private SpindexerSubsystem m_spindexer;
    private CollectorSubsystem m_collector;

    public RobotCommands(
        DriveSubsystem m_robotDrive,
        LauncherSubsystem m_launcher,
        SpindexerSubsystem m_spindexer,
        CollectorSubsystem m_collector
    ) {
        this.m_robotDrive = m_robotDrive;
        this.m_launcher = m_launcher;
        this.m_spindexer = m_spindexer;
        this.m_collector = m_collector;
    }

    public Command spindexAndLiftArmCommand() {
        return m_spindexer.spindexCommand().alongWith(
            Commands.waitSeconds(CollectorConstants.kTimeUntilArmLiftsWhileShooting)
            .andThen(m_collector.jostleArmCommand())
        );
    }

    public Command spindexAndShootCommand(double flywheelSpeed, double hoodAngle) {
        return Commands.sequence(
            m_launcher.setHoodAngleAndWaitCommand(hoodAngle),
            m_launcher.runFlywheelCommand(flywheelSpeed),
            spindexAndLiftArmCommand()
        )
        
        .handleInterrupt(() -> CommandScheduler.getInstance().schedule(
            Commands.waitSeconds(LauncherConstants.kFlywheelRunOn).raceWith(m_robotDrive.idle()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .andThen(m_launcher.setHoodAngleCommand(LauncherConstants.kHoodZero))
            .andThen(m_launcher.stopFlywheelCommand())
        ))
        .withName("Spindex and Shoot");
    }

    public Command spindexAndAutoShootCommand() {
        return Commands.sequence(
            m_launcher.autoShootCommand(),
            spindexAndLiftArmCommand()
        )

        .handleInterrupt(() -> CommandScheduler.getInstance().schedule(
            Commands.waitSeconds(LauncherConstants.kFlywheelRunOn).raceWith(m_robotDrive.idle()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
            .andThen(m_launcher.setHoodAngleCommand(LauncherConstants.kHoodZero))
            .andThen(m_launcher.stopFlywheelCommand())
        ))
        .withName("Spindex and Auto Shoot");
    }

    public Command aimAndShootRelativeCommand() {
        return m_robotDrive.aimAtHubRelativeCommand().andThen(spindexAndAutoShootCommand());
    }

    public Command alignAndShootCommand() {
        double distanceFromHub = FieldPosition.HUB.getCurrentAlliance().getDistance(m_robotDrive.getPose().getTranslation());
        Matrix<N2, N1> speedAndAngle = ShootingLookupTable.ShootingMap.get(distanceFromHub);
        double flywheelSpeed = speedAndAngle.get(0, 0);
        double hoodAngle = speedAndAngle.get(1, 0);

        return Commands.sequence(
            m_launcher.setHoodAngleCommand(speedAndAngle.get(1, 0)),
            m_robotDrive.aimAtHubCommand().alongWith(m_launcher.waitForHoodAngleChangeCommand()),
            m_launcher.runFlywheelCommand(speedAndAngle.get(0, 0)),
            spindexAndLiftArmCommand()
        )

        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)

        // After this command is cancelled, make sure that drivetrain
        // doesn't move while flywheel is still shooting
        .handleInterrupt(() -> CommandScheduler.getInstance().schedule(
            Commands.waitSeconds(LauncherConstants.kFlywheelRunOn).raceWith(m_robotDrive.idle().withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
        ));
    }
}
