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
    private static final double INTAKE_POSITION = 80.26;
    private static final double AMP_POSITION = 77;
    private CANSparkMax angleMotor;
    private SparkPIDController wristPID;
    private SparkAbsoluteEncoder wristEncoder;
    public Wrist(){
        angleMotor = new CANSparkMax(41, MotorType.kBrushless);
        wristPID = angleMotor.getPIDController();
        wristPID.setP(1);
        wristEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
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
        return Math.abs(wristEncoder.getPosition() - HOME_POSITION) < 15;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("wrist encoder position", wristEncoder.getPosition());
    }
}
