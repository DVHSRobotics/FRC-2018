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
import edu.wpi.first.wpilibj.hal.I2CJNI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;

public class Robot extends IterativeRobot implements PIDOutput {

    private AHRS sensor; //the sensor pulling data from the robot
    private PIDController pid; //does calculations to make accurate turns
    private Encoder lEncoder, rEncoder; //wheel
    private I2C arduino;

    private DifferentialDrive myRobot; //"tank drive"
    private Joystick controller;
    private boolean arcadeDrive, compressAir, soleOnePowered, soleTwoPowered, autonomousEnabled;
    private Thread reader;
    private final double MIN_ROTATIONSPEED = 0.4;
    private double driveSpeed, rotation;
    private Spark winch;
    private Compressor compressor;
    private Solenoid solenoid1, solenoid2; //pneumatic control for cube launch
    private double kP = 0.045, kI = 0.0, kD = 0.1, kF = 0;

    //.045 0 .1 0
    //Old Robot PID Constants: .025, 0, 0.1, 0
    private byte[] distance;
    private final float INCHES_PER_ENCODER_PULSE = 0.942477796f / 10.75f;

    @Override
    public void robotInit() {

        //Objects Initialization :-)
        distance = new byte[32];
        winch = new Spark(1);
        sensor = new AHRS(I2C.Port.kMXP);
        pid = new PIDController(kP, kI, kD, kF, sensor, this);
        lEncoder = new Encoder(4,5);
        rEncoder = new Encoder(2, 3);
        myRobot = new DifferentialDrive(new Spark(0), new Spark(2));
        controller = new Joystick(0);
        compressor = new Compressor(0);
        solenoid1 = new Solenoid(0);
        solenoid2 = new Solenoid(1);
        arduino = new I2C(I2C.Port.kOnboard,8);


        //Variable Settings
        pid.setOutputRange(-0.45, 0.45);
        pid.setInputRange(-1080, 1080);
        pid.setAbsoluteTolerance(1); //min. degree that pid can read. If it's within (1) degree, returns pid.onTarget() as true
        compressor.setClosedLoopControl(true); //compressor auto-regulates its pressure


        liveWindow();

    }

    @Override
    public void autonomousInit() {

        moveToPosition();
        fireCube();


    }

    @Override
    public void autonomousPeriodic() {

        //myRobot.arcadeDrive(0, rotation);

        //rotation is set in PIDWrite
        //because the robot is passed in pid constructor as output

    }

    @Override
    public void teleopInit() {

        winch.setSafetyEnabled(false);
        lEncoder.reset();
        rEncoder.reset();

    }

    @Override
    public void teleopPeriodic() {


        motorController();
        driveControl();
        //System.out.println(lEncoder.getDistance() + " " + rEncoder.getDistance());
        pistonControl();
        //Methods for Forklift, Claw
        printDistance();


    }

    @Override
    public void testInit() {

        /*lEncoder.reset();
        rEncoder.reset();
        myRobot.arcadeDrive(0.6, 0);

        Thread t = new Thread(() -> {
            while(!Thread.interrupted()) {
                System.out.println(lEncoder.getDistance() + " " + rEncoder.getDistance());
                Timer.delay(1);
            }
        });

        t.start();

        Timer.delay(3);
        myRobot.arcadeDrive(0, 0);


        autonomousEnabled = true;
        sensor.reset();
        turnAngle(90, 2);
        System.out.println(sensor.getAngle() + " " + pid.getSetpoint());
        */

        driveDistance(24);

    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledInit() {
        disableAuto();
    }

    private void liveWindow() {

        LiveWindow.addActuator("Turning", "PID", pid);
        SmartDashboard.putNumber("Minimum Speed", MIN_ROTATIONSPEED);
        SmartDashboard.putNumber("Rotation Speed", rotation);

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
                    //start on leftmost side of diamond plate, drive 168 inches forward (middle switch
                } else {
                    System.out.println("drive to right side of switch past auto line, place power cube on");
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
                } else {
                    System.out.println("drive to left side of switch past auto line, place power cube on");
                }

                break;
        }

    }

    private void fireCube(){
        //placeholder
        //Using back launch mechanism
    }

    private void pistonControl() {

        soleOnePowered = controller.getRawButton(7);
        soleTwoPowered = controller.getRawButton(8);

        solenoid1.set(soleOnePowered);
        solenoid2.set(soleTwoPowered);

    }

    private void driveDistance(float distanceInches) {

        double currentDistance = 0;
        double speed = 0;
        double adjustedSpeed = 0;

        while(currentDistance < distanceInches) {

            if(lEncoder.getDistance() <= rEncoder.getDistance()) {
                currentDistance = rEncoder.getDistance() * INCHES_PER_ENCODER_PULSE;
                speed = (distanceInches - currentDistance) / distanceInches;
            }
            else {
                currentDistance = lEncoder.getDistance() * INCHES_PER_ENCODER_PULSE;
                speed = (distanceInches - currentDistance) / distanceInches;
            }

            if(speed > 0)
                adjustedSpeed = speed + MIN_ROTATIONSPEED - (speed * MIN_ROTATIONSPEED);
            else
                adjustedSpeed = speed - MIN_ROTATIONSPEED - (speed * MIN_ROTATIONSPEED);

            myRobot.arcadeDrive(adjustedSpeed, 0);

        }



        myRobot.arcadeDrive(0,0);

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
            myRobot.tankDrive( controller.getX() * driveSpeed,  controller.getY() * driveSpeed);

    }

    private void motorController() {

        winch.setSpeed(controller.getRawAxis(5));

    }

    public void pidWrite(double rotation) {


        if(rotation > 0)
            this.rotation = rotation + MIN_ROTATIONSPEED - (rotation * MIN_ROTATIONSPEED);
            //value goes from 0 - 1 to MIN_ROTATIONSPEED - 1
        else
            this.rotation = rotation - MIN_ROTATIONSPEED - (rotation * MIN_ROTATIONSPEED);
            //value goes from -1 - 0 to -1 - -MIN_ROTATIONSPEED

    }

    private void disableAuto() {
        if(autonomousEnabled)
            autonomousEnabled = !autonomousEnabled;
    }

    private void turnAngle(double angle, double timeout) {

        double lastTime = System.currentTimeMillis()/1000, deltaTime, totalTime = 0; //Time delay for 2 seconds
        pid.enable();
        pid.setSetpoint(angle);

        while((!pid.onTarget() || Math.abs(rotation) > 0.42) && totalTime < timeout && autonomousEnabled) {
            myRobot.arcadeDrive(0, rotation);
            deltaTime = System.currentTimeMillis()/1000 - lastTime;
            lastTime = System.currentTimeMillis()/1000;
            totalTime += deltaTime;
            System.out.println(rotation + " " + sensor.getAngle());
        }

        pid.disable();

    }
    void printDistance(){
        //System.out.println();
        arduino.read(8,1,distance);
        System.out.println(distance[0]);
    }

}
