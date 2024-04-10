package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{
    private static final double HOME_POSITION = 50.5;
    private static final double AUTO_POSITION = 50.5;
    private static final double TRAP_POSITION = 49.75;
    private static final double INTAKE_POSITION = 19.24;
    private static final double AMP_POSITION = 21;
    private static final double PARALLEL_POSITION = 25;
    private static final double CLIMB_POSITION = 44;
    private static final double SOURCE_POSITION = 45.585;
    private CANSparkMax angleMotor;
    private SparkPIDController wristPID;
    private SparkAbsoluteEncoder wristEncoder;
    public Wrist(){
        angleMotor = new CANSparkMax(41, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        
        wristPID = angleMotor.getPIDController();

        wristEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        wristEncoder.setPositionConversionFactor(100);

        wristPID.setFeedbackDevice(wristEncoder);
        wristPID.setP(.2);
        wristPID.setI(0);
        wristPID.setD(.2);

        wristPID.setP(.05, 1);
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
    public void climbPosition(){
        wristPID.setReference(CLIMB_POSITION, ControlType.kPosition);
    }
    public void sourcePosition(){
        wristPID.setReference(SOURCE_POSITION, ControlType.kPosition);
    }
    public void autoPosition(){
        wristPID.setReference(AUTO_POSITION, ControlType.kPosition);
    }
    public void trapPosition(){
        wristPID.setReference(TRAP_POSITION, ControlType.kPosition, 1);
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
    public boolean atClimbPosition(){
        return Math.abs(wristEncoder.getPosition() - CLIMB_POSITION) < 1.5;
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("wrist encoder position", wristEncoder.getPosition());
    }
}
