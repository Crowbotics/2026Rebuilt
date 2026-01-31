package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.SpindexerConstants;

public class LauncherSubsystem extends SubsystemBase {
    private final SparkMax m_flywheel = new SparkMax(LauncherConstants.kFlywheelCanId, MotorType.kBrushless);
    private final SparkMax m_hood = new SparkMax(LauncherConstants.kHoodCanId, MotorType.kBrushless);
    
    public LauncherSubsystem() {

    }
}
