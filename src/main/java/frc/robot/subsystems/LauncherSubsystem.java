package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;
import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Configs;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
    private final SparkMax m_flywheel = new SparkMax(LauncherConstants.kFlywheelCanId,MotorType.kBrushless);
    private final SparkMax m_hood = new SparkMax(LauncherConstants.kHoodCanId,MotorType.kBrushless);

    private final RelativeEncoder m_hoodEncoder = m_hood.getAlternateEncoder();

    private final SparkClosedLoopController m_flywheelController = m_flywheel.getClosedLoopController();
    private final SparkClosedLoopController m_hoodController = m_hood.getClosedLoopController();

    public LauncherSubsystem() {
        m_flywheel.configure(Configs.LauncherConfigs.flywheelConfig , ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_hood.configure(Configs.LauncherConfigs.hoodConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        setDefaultCommand(this.idle());
    }

    @Override
    public void periodic() {
        
    }

    public Command runFlywheelCommand(Optional<Double> speed) {
        return this.runOnce(
            () -> {
                m_flywheelController.setSetpoint(speed.isPresent() ? speed.get() : LauncherConstants.kFlywheelSpeed, ControlType.kVelocity);
            }
        )
        .andThen(Commands.waitSeconds(LauncherConstants.kFlywheelWindupTime))
        // Flywheel is run for a little longer on end to ensure that all balls
        // in the system have been cleared
        .handleInterrupt(() -> CommandScheduler.getInstance().schedule(stopFlywheelCommand()));
    }

    public Command stopFlywheelCommand() {
        return Commands.waitSeconds(LauncherConstants.kFlywheelRunOn).andThen(
            this.runOnce(() -> m_flywheelController.setSetpoint(0, ControlType.kVelocity))
        );
    }

    public Command setHoodAngleCommand(double angle) {
        return this.runOnce(
            () -> {
                m_hoodController.setSetpoint(angle, ControlType.kPosition);
            }
        );
    }

    public Command waitForHoodAngleChangeCommand() {
        return this.idle().until(
            () -> {
                if (
                    Math.abs(m_hood.getAbsoluteEncoder().getPosition()) <= LauncherConstants.kHoodAngleTolerance &&
                    Math.abs(m_hood.getAbsoluteEncoder().getVelocity()) <= LauncherConstants.kHoodSpeedTolerance
                ) {
                    return true;
                }
                return false;
            }
        );
    }
}