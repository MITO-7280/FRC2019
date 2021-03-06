/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7280.mecanum_drive_test.command_group;

import org.usfirst.frc7280.mecanum_drive_test.Constants;
import org.usfirst.frc7280.mecanum_drive_test.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PutPlate extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PutPlate() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    /*
    1. move z to adjust position 
    2. move x and y to adjust position 
    3. open cylinder 
    4. elevator down a little 
    5. move back 
    6. go to original position 
    
    1.
    3.2.
    4.5.
    6.

    */

    addSequential(new visionMotion());
    addSequential(new SolenoidOut());

    addParallel(new ElevatorDown());
    addSequential(new MoveY(-3000));

    addParallel(new Lift(Constants.kZeroLevel));
    addParallel(new ArmLift());


  }
}
