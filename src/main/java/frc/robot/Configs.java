package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(false)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.01, 0.0001, 0.00)
                    //.velocityFF(drivingVelocityFeedForward + 0.003)
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward + 0.003);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(false)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1.3, 0, 0.01)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class CollectorConfigs {
        public static final SparkMaxConfig armConfig = new SparkMaxConfig();

        static {
                armConfig
                        .idleMode(IdleMode.kBrake)
                        .inverted(true);
                /*armConfig.softLimit
                        .reverseSoftLimit(359)
                        .reverseSoftLimitEnabled(true);*/
                armConfig.closedLoop
                        .pid(0.0012, 0, 0)
                        .outputRange(-1, 1)
                        .positionWrappingEnabled(false)
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        ;
                armConfig.absoluteEncoder
                        .velocityConversionFactor(CollectorConstants.kTurningFactor)
                        .positionConversionFactor(CollectorConstants.kTurningFactor)
                        .zeroOffset(133/360.0)
                        .inverted(true);
                        // Uncomment once encoder is confirmed to be working
                        //.setSparkMaxDataPortConfig()
                        //.countsPerRevolution(8192)
                        ; // Through Bore Encoder
                        
        }
    }

    public static final class LauncherConfigs {
        public static final SparkFlexConfig flywheelConfig = new SparkFlexConfig();
        public static final SparkFlexConfig flywheelConfig2 = new SparkFlexConfig();
        public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

        static {
                double shooterVelocityFactor = (LauncherConstants.kShooterWheelRadius * 2 * Math.PI) / 60;

                flywheelConfig
                        .inverted(true)
                        .idleMode(IdleMode.kCoast);
                flywheelConfig.encoder
                        .velocityConversionFactor(shooterVelocityFactor); // convert rotations per minute to meters per second
                flywheelConfig.closedLoop
                        .pid(0.001, 0, 0)
                        .outputRange(-1, 1)
                        .feedForward
                                .kV(0.38); // volts per meters per second
                                //.kA(0.26); // volts per meters per second squared. Not used for velocity control mode

                hoodConfig
                        .idleMode(IdleMode.kCoast);
                hoodConfig.encoder
                        .positionConversionFactor(1.0/12.0)
                        .velocityConversionFactor(1)
                        ;
                hoodConfig.absoluteEncoder
                        .positionConversionFactor(1.0)
                        .velocityConversionFactor(1.0)
                        .inverted(true)
                        .zeroOffset(0.9)
                        ;
                hoodConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.5, 0, 0)
                        .allowedClosedLoopError(0.025, ClosedLoopSlot.kSlot0)
                        ;
        }
    }
}
