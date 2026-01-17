package com.stuypulse.robot.commands.krakenintake;

import com.stuypulse.robot.subsystems.krakenintake.KrakenIntake.KrakenIntakeRollerState;

public class KrakenIntakeIntake extends SetKrakenIntakeState {

    public KrakenIntakeIntake() {
        super(KrakenIntakeRollerState.INTAKING);
    }

}