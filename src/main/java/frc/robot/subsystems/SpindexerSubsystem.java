package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {
    private final SparkMax m_spinner = new SparkMax(SpindexerConstants.kSpinnerCanId, MotorType.kBrushless);
    private final SparkMax m_kicker = new SparkMax(SpindexerConstants.kKickerCanId, MotorType.kBrushless);
    
    public SpindexerSubsystem() {
        setDefaultCommand(this.idle());
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
