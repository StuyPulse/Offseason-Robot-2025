// package com.stuypulse.robot.commands.arm;

// import com.stuypulse.robot.subsystems.double_jointed_arm.Arm;
// import com.stuypulse.robot.subsystems.double_jointed_arm.Arm.ArmState;

// import edu.wpi.first.wpilibj2.command.Command;

// public class ArmTest135 extends Command {
//     private Arm arm;
//     private ArmState state;

//     public ArmUp90(ArmState state) {
//         this.state = state;
//         this.arm = Arm.getInstance();
//         addRequirements(arm);
//     }

//     @Override
//     public void initialize() {
//         if (arm.getState().isFront() != state.isFront()) arm.switchSides();
//         arm.setState(state);
//     }

//     @Override
//     public boolean isFinished() {
//         return arm.isArmAtTarget() && arm.getState() == state;
//     }
// }
