package com.stuypulse.robot.commands.krakenintake;

import com.stuypulse.robot.subsystems.krakenintake.KrakenIntake.KrakenIntakeRollerState;

public class KrakenIntakeOuttake extends SetKrakenIntakeState {

    public KrakenIntakeOuttake() {
        super(KrakenIntakeRollerState.OUTTAKING);
    }

}