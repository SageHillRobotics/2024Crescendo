package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{
    private static final double HOME_POSITION = 49.5;
    private static final double INTAKE_POSITION = 19;
    private static final double AMP_POSITION = 21;
    private static final double PARALLEL_POSITION = 25;
    private CANSparkMax angleMotor;
    private SparkPIDController wristPID;
    private RelativeEncoder wristIntegratedEncoder;
    private SparkAbsoluteEncoder wristEncoder;
    public Wrist(){
        angleMotor = new CANSparkMax(41, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        
        wristPID = angleMotor.getPIDController();

        wristEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        wristIntegratedEncoder = angleMotor.getEncoder();
        wristEncoder.setPositionConversionFactor(100);

        wristPID.setFeedbackDevice(wristEncoder);
        wristPID.setP(.2);
        wristPID.setI(0);
        wristPID.setD(.2);

        angleMotor.burnFlash();
    }

    public void intakePosition(){
        wristPID.setReference(INTAKE_POSITION, ControlType.kPosition);
    }
    public void ampPosition(){
        wristPID.setReference(AMP_POSITION, ControlType.kPosition);
    }

    public void retract(){
        wristPID.setReference(HOME_POSITION, ControlType.kPosition);
    }
    
    public boolean atIntakePosition(){
        return Math.abs(wristEncoder.getPosition() - INTAKE_POSITION) < 1.5;
    }

    public boolean atHomePosition(){
        return Math.abs(wristEncoder.getPosition() - HOME_POSITION) < 1.5;
    }

    public boolean canRetract(){
        return (Math.abs(wristEncoder.getPosition() - PARALLEL_POSITION) < 4) || wristEncoder.getPosition() > 25;
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("wrist encoder position", wristEncoder.getPosition());
        SmartDashboard.putNumber("wrist integrated encoder position", wristIntegratedEncoder.getPosition());
    }
}
