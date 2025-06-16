package com.stuypulse.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerImpl extends Indexer {
    
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DigitalInput frontBeamBreak;
    private final DigitalInput backBeamBreak;
    private final BStream passedFrontBeam;
    private final BStream passedBackBeam;
    
    public IndexerImpl() {

        leftMotor = new TalonFX(0,"");
        rightMotor = new TalonFX(1, "");
        // motor configs go here
        frontBeamBreak = new DigitalInput(0);
        backBeamBreak = new DigitalInput(1);

        passedFrontBeam = BStream.create(frontBeamBreak).not()
                    .filtered(new BDebounce.Both(0.01));
        passedBackBeam = BStream.create(backBeamBreak).not()
                    .filtered(new BDebounce.Both(0.01));
    }

    public boolean coralInPlace() {
        if (passedFrontBeam.get() && passedBackBeam.get()) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        super.periodic();

        leftMotor.set(getState().getSpeed());
        rightMotor.set(getState().getSpeed());

        SmartDashboard.putBoolean("Shooter/Coral in Place", coralInPlace());

        
    }
}
