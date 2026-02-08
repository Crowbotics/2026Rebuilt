package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
    private final SparkMax m_flywheel = new SparkMax(LauncherConstants.kFlywheelCanId,MotorType.kBrushless);
    private final SparkMax m_hood = new SparkMax(LauncherConstants.kHoodCanId,MotorType.kBrushless);

    private final SparkClosedLoopController m_flywheelController;
    private final SparkClosedLoopController m_hoodController;

    public LauncherSubsystem() {
        m_flywheel.configure(Configs.LauncherConfigs.flywheelConfig , ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_hood.configure(Configs.LauncherConfigs.hoodConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        
        m_flywheelController = m_flywheel.getClosedLoopController();
        m_hoodController = m_hood.getClosedLoopController();
    }

    public Command launch() {
        return this.startEnd(
            () -> {
                m_flywheelController.setSetpoint(LauncherConstants.kLauncherSpeed, ControlType.kVelocity);
                m_hoodController.setSetpoint(LauncherConstants.kHoodUpSetpoint, ControlType.kPosition);
            },
            () -> {
                m_flywheelController.setSetpoint(0, ControlType.kVelocity);
                m_hoodController.setSetpoint(LauncherConstants.kHoodDownSetpoint, ControlType.kPosition);
            }
        );
    }
}