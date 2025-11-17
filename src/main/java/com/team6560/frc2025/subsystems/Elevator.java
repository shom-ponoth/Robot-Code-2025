package com.team6560.frc2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6560.frc2025.Constants.ElevatorConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
public class Elevator extends SubsystemBase {

    public enum State {
        STOW,
        L2,
        L3,
        L4,
        BALL,
        S_L2,
        S_L3, 
        S_L4, S_STOW
    };
    
    private final TalonFX m_leftElev;
    private final TalonFX m_rightElev;

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    private double targetPos = 0;

    private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Elevator");
    private final NetworkTableEntry ntHeight = ntTable.getEntry("Height");
    private final NetworkTableEntry ntTargetPos = ntTable.getEntry("Target height");
    private final NetworkTableEntry ntLeftCurrent = ntTable.getEntry("Left Current (A)");
    private final NetworkTableEntry ntRightCurrent = ntTable.getEntry("Right Current (A)");
    private final NetworkTableEntry ntPositionError = ntTable.getEntry("Position Error");
    private final NetworkTableEntry ntVelocity = ntTable.getEntry("Velocity");

    public Elevator() {
        this.m_leftElev = new TalonFX(ElevatorConstants.ELEV_LEFT_ID, "Canivore");
        this.m_rightElev = new TalonFX(ElevatorConstants.ELEV_RIGHT_ID, "Canivore");

        this.topLimitSwitch = new DigitalInput(ElevatorConstants.ELEV_UPPER_LIMIT_SWITCH_ID);
        this.bottomLimitSwitch = new DigitalInput(ElevatorConstants.ELEV_LOWER_LIMIT_SWITCH_ID);

        // Feedforward + Feedback configuration
        Slot0Configs elevatorPID = new Slot0Configs();

        // Feedforward gains (physics-based model)
        elevatorPID.kS = 0.25;  // Overcome static friction
        elevatorPID.kG = 0.4;   // Counteract gravity (already tuned)
        elevatorPID.kV = 0.12;  // Velocity feedforward (adjust if needed)
        elevatorPID.kA = 0.01;  // Acceleration feedforward

        // Feedback gains (reduced - feedforward does most of the work)
        elevatorPID.kP = 0.1;   // Reduced from 0.7 - just for small corrections
        elevatorPID.kI = 0.0;   // Disabled - not needed with good feedforward
        elevatorPID.kD = 0.0;   // Disabled - motion profiling handles damping

        // Motion Magic configuration (smooth trapezoidal motion profile)
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity = 80;   // Max velocity (rotations/sec) - tune as needed
        motionMagicConfig.MotionMagicAcceleration = 160;    // Max acceleration (rotations/sec²) - tune as needed
        motionMagicConfig.MotionMagicJerk = 1600;           // Jerk for smoothness (rotations/sec³)

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Apply both PID and Motion Magic configs
        m_leftElev.getConfigurator().apply(config.withSlot0(elevatorPID).withMotionMagic(motionMagicConfig));
        m_rightElev.getConfigurator().apply(config.withSlot0(elevatorPID).withMotionMagic(motionMagicConfig));

        ntHeight.setDouble(0.0);
        ntTargetPos.setDouble(0.0);
        
    }

    @Override
    public void periodic() {
        updateNTTable();
    }

    public void updateNTTable(){
        ntHeight.setDouble(getElevatorHeight());
        ntTargetPos.setDouble(this.targetPos);
        ntLeftCurrent.setDouble(m_leftElev.getSupplyCurrent().getValueAsDouble());
        ntRightCurrent.setDouble(m_rightElev.getSupplyCurrent().getValueAsDouble());
        ntPositionError.setDouble(Math.abs(this.targetPos - getElevatorHeight()));
        ntVelocity.setDouble(getElevatorVelocity());
    }

    public void setElevatorPosition(double targetrotelev) {
        this.targetPos = targetrotelev;
        // Use Motion Magic for smooth trapezoidal motion profile
        final MotionMagicVoltage m_request = new MotionMagicVoltage(targetrotelev);

        m_leftElev.setControl(m_request);
        m_rightElev.setControl(m_request);
    }


    public void stopMotors() {
        m_rightElev.stopMotor();
        m_leftElev.stopMotor();
    }

    public boolean topLimitSwitchDown() {
        return topLimitSwitch.get();
    }

    public boolean bottomLimitSwitchDown() {
        return bottomLimitSwitch.get();
    }

    public double getElevatorHeight(){
        return m_leftElev.getPosition().getValueAsDouble();// * ElevatorConstants.ELEV_GEAR_RATIO;
    }

    public double getElevatorVelocity(){
        return m_leftElev.getVelocity().getValueAsDouble();
    }

    public void resetEncoderPos(double setposition) {
        m_leftElev.setPosition(setposition);
        m_rightElev.setPosition(setposition);
    }

    // all code below this point is for testing purposes
    // public void turnOnMotors(){
    //     m_leftElev.setControl(new VelocityVoltage(200));
    //     m_rightElev.setControl(new VelocityVoltage(200));
    // }
// 
    // public void turnOnMotorsNoPID(){
    //     m_leftElev.set(0.1);
    //     m_rightElev.set(0.1);
    // }
    // public void revMotorsNoPID(){
    //     m_leftElev.set(-0.1);
    //     m_rightElev.set(-0.1);
    // }
// 
    // public void testMotor(double output){
    //     m_leftElev.set(output);
    //     m_rightElev.set(output);
    // }
}