/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team6612;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.kauailabs.navx.frc.*;

//josh was here :-)
public class Robot extends IterativeRobot implements PIDOutput {


    private AHRS sensor; //the sensor pulling data from the robot
    private PIDController pid; //does calculations to make accurate turns
    private Encoder encoder;

    private DifferentialDrive myRobot; //"tank drive"
    private Joystick controller;
    private boolean arcadeDrive, compressAir, soleOnePowered, soleTwoPowered;
    private Thread reader;
    private double driveSpeed, rotation, MIN_ROTATIONSPEED;
    private Spark motorTest;
    private Compressor compressor;
    private Solenoid solenoid1, solenoid2; //pneumatic control for cube launch
    private double kP = 0.03f, kI = 0, kD = 0.05, /*0.04 kD is good*/ kF = 0;

    @Override
    public void robotInit() {

        //Objects Initialization :-)
        motorTest = new Spark(3);
        sensor = new AHRS(I2C.Port.kMXP);
        pid = new PIDController(kP, kI, kD, kF, sensor, this);
        encoder = new Encoder(0,1);
        MIN_ROTATIONSPEED = 0.45 /* min turn speed = 0.45-0.47, found by testing */;

        myRobot = new DifferentialDrive(new Spark(0), new Spark(1));
        controller = new Joystick(0);
        compressor = new Compressor(0);
        solenoid1 = new Solenoid(0);
        solenoid2 = new Solenoid (1);

    }

    @Override
    public void autonomousInit() {

        moveToPosition();
        fireCube();

        //test code
        //PID starts, setpoint is the amount to turn

        sensor.reset();
        pid.enable();
        pid.setSetpoint(90f);

    }

    @Override
    public void autonomousPeriodic() {

        myRobot.arcadeDrive(0, rotation);

        //rotation is set in PIDWrite
        //because the robot is passed in pid constructor as output

    }

    @Override
    public void teleopInit() {

        startReaderThread();
        motorTest.setSafetyEnabled(false);

    }

    @Override
    public void teleopPeriodic() {

        motorController();
        //driveControl();
        //pistonControl();

        //min rotation test
        myRobot.arcadeDrive(0, controller.getRawAxis(4));

    }

    private void startReaderThread() {

        arcadeDrive = controller.getRawButton(4);

        //reader thread prints out arcade drive is active/inactive while enabling/disabling
        reader = new Thread(() -> {
            while (!Thread.interrupted()) {

                //System.out.println(sensor.getAngle());

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
        
    }

    private void pistonControl() {

        compressAir = controller.getRawButton(5);
        compressor.setClosedLoopControl(compressAir);

        soleOnePowered = controller.getRawButton(6);
        soleTwoPowered = controller.getRawButton(7);

        solenoid1.set(soleOnePowered);
        solenoid2.set(soleTwoPowered);

    }

    private void driveControl() {

        //Prints if arcadeDrive is enabled/disabled
        if (arcadeDrive != controller.getRawButton(4)) {
            arcadeDrive = controller.getRawButton(4); //enable/disable
            if (arcadeDrive)
                System.out.println("Arcade drive is active!");
            else
                System.out.println("Arcade drive is inactive!");

        }

        driveSpeed = 1;
        //if arcadeDrive is true, set drive method to arcade drive; else, tank drive
        if (arcadeDrive)
            myRobot.arcadeDrive(-controller.getX(), controller.getRawAxis(3)*driveSpeed);
        else
            myRobot.tankDrive(-1 * controller.getY() * driveSpeed, -1 * controller.getX() * driveSpeed);

    }

    private void motorController() {

        motorTest.setSpeed(controller.getRawAxis(5));
        System.out.println(encoder.get());

    }

    public void pidWrite(double rotation) {

        if(rotation > 0)
            this.rotation = rotation + MIN_ROTATIONSPEED - (rotation * MIN_ROTATIONSPEED);
            //value goes from 0 - 1 to MIN_ROTATIONSPEED - 1
        else
            this.rotation = rotation - MIN_ROTATIONSPEED - (rotation * MIN_ROTATIONSPEED);
            //value goes from -1 - 0 to -1 - -MIN_ROTATIONSPEED
        System.out.println(rotation);

    }

}
