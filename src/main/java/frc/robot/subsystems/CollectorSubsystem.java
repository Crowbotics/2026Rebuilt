package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Configs.CollectorConfigs;
import frc.robot.Constants.CollectorConstants;

public class CollectorSubsystem extends SubsystemBase {
    private final SparkMax m_arm = new SparkMax(CollectorConstants.kArmCanId, MotorType.kBrushless);
    private final SparkMax m_roller = new SparkMax(CollectorConstants.kRollerCanId, MotorType.kBrushless);

    private final AbsoluteEncoder m_armEncoder = m_arm.getAbsoluteEncoder();

    private final SparkClosedLoopController m_armController;

    public CollectorSubsystem() {
        m_roller.configure(CollectorConfigs.rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_arm.configure(CollectorConfigs.armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_armController = m_arm.getClosedLoopController();



        setDefaultCommand(this.idle());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder", m_armEncoder.getPosition());
    }

    public Command setArmAngleCommand(double angle) {
        return this.runOnce(
            () -> {
                m_armController.setSetpoint(angle, ControlType.kPosition);
            }
        );
    }

    public Command jostleArmCommand() {
        return new SequentialCommandGroup(
            setArmAngleCommand(CollectorConstants.kArmUnextendedSetpoint + 15),
            Commands.waitSeconds(0.6),
            setArmAngleCommand(CollectorConstants.kArmExtendedSetpoint),
            Commands.waitSeconds(0.6)
        ).repeatedly().finallyDo(
            () -> {
                m_armController.setSetpoint(CollectorConstants.kArmExtendedSetpoint, ControlType.kPosition);
                m_roller.set(0);
            }
        ).beforeStarting(startIntakeCommand(CollectorConstants.kRollerJostleSpeed));
    }

    public Command runIntakeCommand() {
        return this.startEnd(
            () -> {
                m_roller.set(CollectorConstants.kRollerSpeed);
            },
            () -> {
                m_roller.set(0);
            }
        );
    }

    public Command reverseIntakeCommand() {
        return this.startEnd(
            () -> {
                m_roller.set(-CollectorConstants.kRollerSpeed);
            },
            () -> {
                m_roller.set(0);
            }
        );
    }

    public Command runIntakeCommand(double speed) {
        return this.startEnd(
            () -> {
                m_roller.set(speed);
            },
            () -> {
                m_roller.set(0);
            }
        );
    }

    public Command startIntakeCommand() {
        return this.runOnce(
            () -> {
                m_roller.set(CollectorConstants.kRollerSpeed);
            }
        );
    }

    public Command startIntakeCommand(double speed) {
        return this.runOnce(
            () -> {
                m_roller.set(speed);
            }
        );
    }

    public Command stopIntakeCommand() {
        return this.runOnce(
            () -> {
                m_roller.set(0);
            }
        );
    }
}