package com.stuypulse.robot.commands.krakenintake;

import com.stuypulse.robot.subsystems.krakenintake.KrakenIntake.KrakenIntakeRollerState;

public class KrakenIntakeStop extends SetKrakenIntakeState {

    public KrakenIntakeStop() {
        super(KrakenIntakeRollerState.STOP);
    }

}