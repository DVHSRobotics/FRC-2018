/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team6612;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/*
 *
 * getRawButton(4) is top right trigger
 *
 * */
public class Robot extends IterativeRobot {
    private DifferentialDrive myRobot;//"tank drive"
    private Joystick controller;
    private boolean arcadeDrive;
    private boolean compressAir;
    private boolean sole1;
    private boolean sole2;
    private boolean sole3;
    private Thread reader;
    private double mSpeed;
    private Spark eMotor;
    private Compressor c;
    private Solenoid solenoid1;
    private Solenoid solenoid2;

    @Override
    public void robotInit() {

        myRobot = new DifferentialDrive(new Spark(0), new Spark(1));
        controller = new Joystick(0);
        c = new Compressor(0);
        solenoid1 = new Solenoid(0);
        solenoid2 = new Solenoid (1);
    }

    @Override
    public void autonomousInit() {
        //getGameSpecificMessage returns a three char string of either a L or R char in each pos. Ex: "LRR"
        //getLocation returns an int from 1 to 3, representing which slot (starting from the left) the robot has been assigned to.
        String plateColors = DriverStation.getInstance().getGameSpecificMessage();
        int location = DriverStation.getInstance().getLocation();
        switch (location) {
            case 1:
                if (plateColors.charAt(0) == 'L') {
                    //drive to left side of switch past auto line, place power cube on
                } else if (plateColors.charAt(1) == 'L') {
                    //drive to left side of scale, place power cube on
                } else {
                    //drive to right side of scale, place power cube on
                }

                break;

            case 2:
                if (plateColors.charAt(0) == 'L') {
                    //drive to left side of switch past auto line, place power cube on
                } else {
                    //drive to right side of switch past auto line, place power cube on
                }

                break;

            case 3:
                if (plateColors.charAt(0) == 'R') {
                    //drive to right side of switch past auto line, place power cube on
                } else if (plateColors.charAt(1) == 'R') {
                    //drive to right side of the scale, place power cube on
                } else {
                    //drive to left side of the scale, place power cube on
                }

                break;
        }
    }

    @Override
    public void teleopInit() {

        startReaderThread();
        eMotor = new Spark(2);

    }

    @Override
    public void teleopPeriodic() {

        //getRawAxis gives values from -1 to 1
        mSpeed = controller.getRawAxis(6);
        eMotor.setSpeed(mSpeed);


        //if arcadeDrive is true, set drive method to arcade drive; else, tank drive
        if (arcadeDrive)
            myRobot.arcadeDrive(-controller.getX(), controller.getRawAxis(3));
        else
            myRobot.tankDrive(-1 * controller.getY(), -1 * controller.getX(), true);

        compressAir = controller.getRawButton(5);

        if (compressAir)
            c.setClosedLoopControl(true);
        else
            c.setClosedLoopControl(false);

        sole1 = controller.getRawButton(6);
        sole2 = controller.getRawButton(7);
        sole3 = controller.getRawButton(8);

        solenoid1.set(sole1 || sole3);
        solenoid2.set(sole2 || sole3);

    }

    private void startReaderThread() {

        arcadeDrive = controller.getRawButton(4);

        //reader thread prints out arcade drive is active/inactive while enabling/disabling
        reader = new Thread(() -> {
            while (!Thread.interrupted()) {

                if (arcadeDrive != controller.getRawButton(4)) {
                    arcadeDrive = controller.getRawButton(4); //enable/disable
                    if (arcadeDrive)
                        System.out.println("Arcade drive is active!");
                    else
                        System.out.println("Arcade drive is inactive!");

                }

            }
        });

        reader.start();

    }
}
