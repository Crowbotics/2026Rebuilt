package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.LauncherConstants;

public class LauncherSubsystem extends SubsystemBase {
    private final SparkMax m_flywheel = new SparkMax(LauncherConstants.kFlywheelCanId,MotorType.kBrushless);
    private final SparkMax m_hood = new SparkMax(LauncherConstants.kHoodCanId,MotorType.kBrushless);

    public LauncherSubsystem() {
        m_flywheel.configure(Configs.LauncherConfigs.flywheelConfig , ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_hood.configure(Configs.LauncherConfigs.hoodConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }
    public Command launch(){
        return this.startEnd(
            () -> {
                m_flywheel.set(LauncherConstants.kLauncherSpeed);
                m_hood.set(LauncherConstants.kHoodspeed);
            }, 
            () -> {
                m_flywheel.set(0);
                m_hood.set(0);
            }
        );
    }
}