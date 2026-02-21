package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {
    private final SparkMax m_spinner = new SparkMax(SpindexerConstants.kSpinnerCanId, MotorType.kBrushless);
    private final SparkMax m_kicker = new SparkMax(SpindexerConstants.kKickerCanId, MotorType.kBrushless);
    
    public SpindexerSubsystem() {
        setDefaultCommand(this.idle());

        SparkMaxConfig spinnerConfig = new SparkMaxConfig();
        spinnerConfig.inverted(true);
        m_spinner.configure(spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command spindexCommand() {
        return this.startEnd(
            () -> {
                m_kicker.set(SpindexerConstants.kKickerSpeed);
                m_spinner.set(SpindexerConstants.kSpinnerSpeed);
            },
            () -> {
                m_kicker.set(0);
                m_spinner.set(0);
            }
        );
    }
}
