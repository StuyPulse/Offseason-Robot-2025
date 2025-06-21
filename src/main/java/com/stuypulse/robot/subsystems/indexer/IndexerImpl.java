package com.stuypulse.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
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

        leftMotor = new TalonFX(Ports.Indexer.LEFT_MOTOR);
        rightMotor = new TalonFX(Ports.Indexer.RIGHT_MOTOR);
        // motor configs go here
        frontBeamBreak = new DigitalInput(Ports.Indexer.FRONT_BEAM_BREAK);
        backBeamBreak = new DigitalInput(Ports.Indexer.BACK_BEAM_BREAK);

        passedFrontBeam = BStream.create(frontBeamBreak).not()
                    .filtered(new BDebounce.Both(Settings.Indexer.CORAL_IN_PLACE_DEBOUNCE));
        passedBackBeam = BStream.create(backBeamBreak).not()
                    .filtered(new BDebounce.Both(Settings.Indexer.CORAL_IN_PLACE_DEBOUNCE));
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
