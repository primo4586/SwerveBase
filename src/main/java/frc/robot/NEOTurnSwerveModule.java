package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Swerve;

import javax.naming.ldap.ManageReferralControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class NEOTurnSwerveModule {
    public int moduleNumber;
    private double angleOffset;
    // NOTE: There isn't much of a difference between using TalonFX and WPI_TalonFX, so it's fine to be like this
    private CANSparkMax mAngleMotor; 
    private RelativeEncoder angleMotorEncoder;
    private PIDController anglePID;

    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;

    private SimpleMotorFeedforward feedforward;

    public NEOTurnSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        feedforward = Swerve.driveFeedforward;

        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();
        anglePID = Swerve.anglePID.getController(Constants.loopPeriod);
        anglePID.enableContinuousInput(Math.toDegrees(-Math.PI), Math.toDegrees(Math.PI));

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); 

        // TODO: needs double checking to see if this is how you work with position in SparkMax with WPI's PID
        mAngleMotor.set(anglePID.calculate(getCanCoder().getDegrees(), desiredState.angle.getDegrees()));
        lastAngle = angle;
    }

    private void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset;
        angleMotorEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        
        angleMotorEncoder = mAngleMotor.getEncoder();
        // this probably can be taken to an outside class that just applies all of the constants, similarly to TalonFXConfiguration or SwerveModuleConstants
        angleMotorEncoder.setInverted(Constants.Swerve.angleMotorInvert);
        angleMotorEncoder.setPositionConversionFactor(360.0 * 1 / Swerve.angleGearRatio);

        mAngleMotor.setSmartCurrentLimit(20, 40);
        mAngleMotor.setIdleMode(Constants.Swerve.angleIdleMode);
        mAngleMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
        mAngleMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);
       
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }


    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(angleMotorEncoder.getPosition());
        return new SwerveModuleState(velocity, angle);
    }
    
}