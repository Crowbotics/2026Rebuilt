package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CollectorConstants;

public class CollectorSubsystem extends SubsystemBase {
    private final SparkMax m_arm = new SparkMax(CollectorConstants.kArmCanId, MotorType.kBrushless);
    private final SparkMax m_roller = new SparkMax(CollectorConstants.kRollerCanId, MotorType.kBrushless);

    private final SparkClosedLoopController m_armController;

    public CollectorSubsystem() {
        m_arm.configure(Configs.CollectorConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_armController = m_arm.getClosedLoopController();
    }

    public Command intake() {
        return this.startEnd(
            () -> {
                m_armController.setSetpoint(CollectorConstants.kArmExtendedSetpoint, ControlType.kPosition);
                m_roller.set(CollectorConstants.kRollerSpeed);
            },
            () -> {
                m_roller.set(0);
                m_armController.setSetpoint(CollectorConstants.kArmUnextendedSetpoint, ControlType.kPosition);
            }
        );
    }
}