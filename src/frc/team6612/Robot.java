/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team6612;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//josh was here :-)
public class Robot extends IterativeRobot {

    private DifferentialDrive myRobot; //"tank drive"
    private Joystick controller;
    private boolean arcadeDrive, compressAir, soleOnePowered, soleTwoPowered;
    private Thread reader;
    private double mSpeed, driveSpeed;
    private Spark motorController;
    private Compressor c;
    private Solenoid solenoid1, solenoid2;

    @Override
    public void robotInit() {

        //Objects Initialization :-)
        myRobot = new DifferentialDrive(new Spark(0), new Spark(1));
        controller = new Joystick(0);
        c = new Compressor(0);
        solenoid1 = new Solenoid(0);
        solenoid2 = new Solenoid (1);
        motorController = new Spark(2);

    }

    @Override
    public void autonomousInit() {

        moveToPosition();
        fireCube();

    }

    @Override
    public void teleopInit() {

        startReaderThread();

    }

    @Override
    public void teleopPeriodic() {

        motorController();
        driveControl();
        pistonControl();

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

    private void moveToPosition() {

        //getGameSpecificMessage returns a three char string of either a L or R char in each pos. Ex: "LRR"
        //getLocation returns an int from 1 to 3, representing which slot (starting from the left) the robot has been assigned to.

        String plateColors = DriverStation.getInstance().getGameSpecificMessage();
        int location = DriverStation.getInstance().getLocation();
        switch (location) {
            case 1:
                if (plateColors.charAt(0) == 'L') {
                    System.out.println("drive to left side of switch past auto line, place power cube on");
                } else if (plateColors.charAt(1) == 'L') {
                    System.out.println("drive to left side of scale, place power cube on");
                } else {
                    System.out.println("drive to right side of scale, place power cube on");
                }

                break;

            case 2:
                if (plateColors.charAt(0) == 'L') {
                    System.out.println("drive to left side of switch past auto line, place power cube on");
                } else {
                    System.out.println("drive to right side of switch past auto line, place power cube on");
                }

                break;

            case 3:
                if (plateColors.charAt(0) == 'R') {
                    System.out.println("drive to right side of switch past auto line, place power cube on");
                } else if (plateColors.charAt(1) == 'R') {
                    System.out.println("drive to right side of the scale, place power cube on");
                } else {
                    System.out.println("drive to left side of the scale, place power cube on");
                }

                break;
        }

    }

    private void fireCube(){
        //placeholder
    }

    private void pistonControl() {

        compressAir = controller.getRawButton(5);
        c.setClosedLoopControl(compressAir);


        soleOnePowered = controller.getRawButton(6);
        soleTwoPowered = controller.getRawButton(7);

        solenoid1.set(soleOnePowered);
        solenoid2.set(soleTwoPowered);

    }

    private void driveControl() {

        driveSpeed = controller.getRawAxis(7);
        //if arcadeDrive is true, set drive method to arcade drive; else, tank drive
        if (arcadeDrive)
            myRobot.arcadeDrive(-controller.getX(), controller.getRawAxis(3)*driveSpeed);
        else
            myRobot.tankDrive(-1 * controller.getY() * driveSpeed, -1 * controller.getX() * driveSpeed);

    }

    private void motorController() {

        //getRawAxis gives values from -1 to 1
        mSpeed = controller.getRawAxis(6);
        motorController.setSpeed(mSpeed);

    }

}
