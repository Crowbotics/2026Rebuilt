package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
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
                    .inverted(true)
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
                        .idleMode(IdleMode.kBrake);
                armConfig.closedLoop
                        .pid(0, 0, 0)
                        .outputRange(-1, 1)
                        .positionWrappingEnabled(false);
                        // Uncomment once encoder is confirmed to be working
                        //.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
                armConfig.alternateEncoder
                        .velocityConversionFactor(CollectorConstants.kTurningFactor)
                        .positionConversionFactor(CollectorConstants.kTurningFactor)
                        // Uncomment once encoder is confirmed to be working
                        //.setSparkMaxDataPortConfig()
                        //.countsPerRevolution(8192)
                        ; // Through Bore Encoder
                        
        }
    }

    public static final class LauncherConfigs {
        public static final SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

        static {
                double shooterVelocityFactor = (LauncherConstants.kShooterWheelRadius * 2 * Math.PI) / 60;

                flywheelConfig
                        .idleMode(IdleMode.kBrake);
                flywheelConfig.encoder
                        .velocityConversionFactor(shooterVelocityFactor); // convert rotations per minute to meters per second
                flywheelConfig.closedLoop
                        .pid(0, 0, 0)
                        .outputRange(-1, 1)
                        .feedForward
                                .kV(0.38); // volts per meters per second
                                //.kA(0.26); // volts per meters per second squared. Not used for velocity control mode
                
                hoodConfig
                        .idleMode(IdleMode.kBrake);
                hoodConfig.alternateEncoder
                        .positionConversionFactor(LauncherConstants.kTurningFactor)
                        .velocityConversionFactor(LauncherConstants.kTurningFactor)
                        // Uncomment once encoder is confirmed to be working
                        //.setSparkMaxDataPortConfig()
                        //.countsPerRevolution(8192)
                        ; // Through Bore Encoder
                hoodConfig.closedLoop
                        // Uncomment once encoder is confirmed to be working
                        // .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                        .pid(0, 0, 0)
                        .outputRange(-1, 1);
        }
    }
}
