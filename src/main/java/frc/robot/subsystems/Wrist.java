package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{
    private static final double HOME_POSITION = 0.69;
    private CANSparkMax angleMotor;
    private SparkPIDController wristPID;
    private DutyCycleEncoder wristEncoder;
    public Wrist(){
        angleMotor = new CANSparkMax(69, MotorType.kBrushless);
        wristPID = angleMotor.getPIDController();
        wristPID.setP(.001);
        wristPID.setI(0);
        wristPID.setD(0);
        wristEncoder = new DutyCycleEncoder(6);
        wristEncoder.setDistancePerRotation(0.5);
        
    }

    public void wristPID(double setpoint){
        wristPID.setReference(setpoint, ControlType.kPosition);
    }

    public void retract(){
        wristPID.setReference(HOME_POSITION, ControlType.kPosition);
    }
    


    @Override
    public void periodic(){
        SmartDashboard.putNumber("wrist encoder cycles", wristEncoder.get());
        SmartDashboard.putNumber("wrist encoder distance", wristEncoder.getDistance());
    }
}
