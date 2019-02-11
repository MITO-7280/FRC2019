// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc7280.mecanum_drive_test.subsystems;


import org.usfirst.frc7280.mecanum_drive_test.Constants;
import org.usfirst.frc7280.mecanum_drive_test.RobotMap;
import org.usfirst.frc7280.mecanum_drive_test.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;



/**
 *
 */
public class Base extends Subsystem {

    private TalonSRX leftFrontMotor = new TalonSRX(RobotMap.leftFrontMotor);
    private TalonSRX leftRearMotor = new TalonSRX(RobotMap.leftRearMotor);
    private TalonSRX rightFrontMotor = new TalonSRX(RobotMap.rightFrontMotor);
    private TalonSRX rightRearMotor = new TalonSRX(RobotMap.rightRearMotor);

    public Base() {

        setMotorPID(leftFrontMotor);
        setMotorPID(leftRearMotor);
        setMotorPID(rightFrontMotor);
        setMotorPID(rightRearMotor);

        leftFrontMotor.setInverted(true);
        leftRearMotor.setInverted(true);


    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new Drive());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void drive(double yValue, double xValue, double zValue){
        motorMode(NeutralMode.Coast);

        double frontLeftSpeed = (yValue + xValue - zValue)/4 * 500 * 4096 /600;
        double rearLeftSpeed = (yValue - xValue - zValue)/4 * 500 * 4096 /600;
        double frontRightSpeed = (yValue - xValue + zValue)/4 * 500 * 4096 /600;
        double rearRighttSpeed = (yValue + xValue + zValue)/4 * 500 * 4096 /600;
        
        leftFrontMotor.set(ControlMode.Velocity, frontLeftSpeed);
        leftRearMotor.set(ControlMode.Velocity, rearLeftSpeed);
        rightFrontMotor.set(ControlMode.Velocity, frontRightSpeed);
        rightRearMotor.set(ControlMode.Velocity, rearRighttSpeed);
    }

    public void stop(){
        motorMode(NeutralMode.Brake);

        drive(0,0,0);
    }

    private void motorMode(NeutralMode mode) {
        leftFrontMotor.setNeutralMode(mode);
        leftRearMotor.setNeutralMode(mode);
        rightFrontMotor.setNeutralMode(mode);
        rightRearMotor.setNeutralMode(mode);
    }

    private void setMotorPID(TalonSRX _talon){
    
        /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
        _talon.config_kF(Constants.kPIDLoopIdx, Constants.kElevatorF, Constants.kTimeoutMs);
        _talon.config_kP(Constants.kPIDLoopIdx, Constants.kElevatorP, Constants.kTimeoutMs);
        _talon.config_kI(Constants.kPIDLoopIdx, Constants.kElevatorI, Constants.kTimeoutMs);
        _talon.config_kD(Constants.kPIDLoopIdx, Constants.kElevatorD, Constants.kTimeoutMs);
    
    }

    private void TalonSRXInit(TalonSRX _talon) {

		// set up TalonSRX and closed loop
    // select an encoder and set it
    _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    
    // make sure the sensor gieves the postive value whent the output is positive. 
		_talon.setSensorPhase(true);

		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

  }

}
