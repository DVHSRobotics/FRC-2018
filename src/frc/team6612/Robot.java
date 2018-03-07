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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.text.DecimalFormat;

public class Robot extends IterativeRobot implements PIDOutput {

    private AHRS sensor; //the sensor pulling data from the robot
    private PIDController pid; //does calculations to make accurate turns
    private Encoder lEncoder, rEncoder, winchEncoder; //wheel
    private I2C arduino;

    private DifferentialDrive myRobot; //"tank drive"
    private Joystick controller;
    private boolean arcadeDrive, compressAir, soleOnePowered, soleTwoPowered, soleThreePowered, soleFourPowered,autonomousEnabled;
    private Thread reader;
    private final double MIN_ROTATIONSPEED = 0.4, MIN_DRIVESPEED = 0.48;
    private double driveSpeed, rotation;
    private double lastTime, deltaTime, totalTime;
    private Spark winch, ledStrip;
    private Compressor compressor;
    private Solenoid solenoid1, solenoid2, solenoid3, solenoid4; //pneumatic control for cube launch
    private double kP = 0.045, kI = 0.0, kD = 0.085, kF = 0;
    private int switchEncoderTicks= 2000; //~1'11"
    private int scaleEncoderTicks=10200;// ~6'11"
    private double adjustmentConstant = 0.01;//multiplier for distance to determine speed
    private int minDistanceFromWall = 8;//NEEDS TO BE MEASURED

    //Competition Robot PID Constants:  0.045, 0, 0.085, 0
    //Old Robot PID Constants:          0.025, 0, 0.1, 0
    private byte[] distance;
    private final float INCHES_PER_ENCODER_PULSE = 0.942477796f / 10.75f;

    @Override
    public void robotInit() {

        // Objects Initialization :-)
        distance = new byte[32];
        winch = new Spark(1);
        ledStrip = new Spark(3);
        sensor = new AHRS(I2C.Port.kMXP);
        pid = new PIDController(kP, kI, kD, kF, sensor, this);
        lEncoder = new Encoder(4,5);
        rEncoder = new Encoder(2, 3);
        winchEncoder = new Encoder(0,1);
        myRobot = new DifferentialDrive(new Spark(0), new Spark(2));
        controller = new Joystick(0);
        compressor = new Compressor(0);
        solenoid1 = new Solenoid(0);
        solenoid2 = new Solenoid(1);
        solenoid3 = new Solenoid(2);
        solenoid4 = new Solenoid(3);
        arduino = new I2C(I2C.Port.kOnboard,8);

        //Variable Settings
        pid.setOutputRange(-0.45, 0.45);
        pid.setInputRange(-1080, 1080);
        pid.setAbsoluteTolerance(0.5); // Min. degree that pid can read. If it's within (0.5) degrees, returns pid.onTarget() as true
        compressor.setClosedLoopControl(true); // Compressor auto-regulates its pressure

        liveWindow();

        //LED Controls
        ledStrip.set(1);

    }

    @Override
    public void autonomousInit() {



        driveDistance(48);
        //System.out.println(lEncoder.getDistance() * INCHES_PER_ENCODER_PULSE - currentDistance);
        //System.out.println(rEncoder.getDistance() * INCHES_PER_ENCODER_PULSE - currentDistance);
        moveToPosition();
        ledStrip.set(1);

    }

    @Override
    public void autonomousPeriodic() {


    }

    @Override
    public void teleopInit() {

        winch.setSafetyEnabled(false);
        myRobot.setSafetyEnabled(false);
        lEncoder.reset();
        rEncoder.reset();
        winch.setSpeed(0);

    }

    @Override
    public void teleopPeriodic() {

        winchController();
        driveControl();
        pistonControl();
        readDistance();


        ledStrip.set(controller.getRawAxis(4));
        //ledColor 1 is off
        //Cool led options: 0.27 heartbeat

    }

    @Override
    public void testInit() {

        autonomousEnabled = true;
        sensor.reset();
        pid.reset();

    }

    @Override
    public void testPeriodic() {

        winch.setSpeed(controller.getRawAxis(5));
        if(controller.getRawButton(3))
            winchEncoder.reset();

    }

    @Override
    public void disabledInit() {

        disableAuto();

    }

    private void liveWindow() {

        //reports values into the SmartDashboard application as a LiveWindow, which can be adjusted in Test mode.
        LiveWindow.addActuator("Turning", "PID", pid);

        //reports values into the SmartDashboard application
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

    private void pistonControl() {

        // Claw
        soleOnePowered = controller.getRawButton(11);
        soleTwoPowered = controller.getRawButton(12);
        solenoid1.set(soleOnePowered);
        solenoid2.set(soleTwoPowered);

        // Back launch mechanism
        /* soleThreePowered = controller.getRawButton(3);
        soleFourPowered = controller.getRawButton(4);
        solenoid3.set(soleThreePowered);
        solenoid4.set(soleFourPowered); */

    }

    private void raiseToHeight(int pulses) {

        if (winchEncoder.get() < pulses - 5)
            winch.setSpeed(0.4);
        else if(winchEncoder.get() > pulses + 5)
            winch.setSpeed(-0.4);
        else
            winch.setSpeed(0);
    }

    private void driveDistance(float distanceInches) {

        //MOMENTUM NEEDS BE ACCOUNTED FOR IN ORDER TO DRIVE ACCURATELY

        lEncoder.reset();
        rEncoder.reset();

        double speed = 0;
        double adjustedSpeed = 0;
        double currentDistance = 0;
        double time = System.currentTimeMillis();

        while(currentDistance != distanceInches) {

            if(lEncoder.getDistance() <= rEncoder.getDistance()) {
                speed = (distanceInches - currentDistance) / distanceInches;
                currentDistance = rEncoder.getDistance() * INCHES_PER_ENCODER_PULSE;

            }
            else {
                speed = (distanceInches - currentDistance) / distanceInches;
                currentDistance = lEncoder.getDistance() * INCHES_PER_ENCODER_PULSE;

            }

            if(speed > 0.7)
                speed = 0.7;

            if(speed > 0)
                adjustedSpeed = speed + MIN_DRIVESPEED - (speed * MIN_DRIVESPEED);
            else
                adjustedSpeed = speed - MIN_DRIVESPEED - (speed * MIN_DRIVESPEED);

            myRobot.arcadeDrive(adjustedSpeed, 0);

        }

        myRobot.arcadeDrive(0,0);

    }

    private void driveControl() {

        //Prints if arcadeDrive is enabled/disabled
        /*if (arcadeDrive != controller.getRawButton(4)) {
            arcadeDrive = controller.getRawButton(4); //enable/disable
            if (arcadeDrive)
                System.out.println("Arcade drive is active!");
            else
                System.out.println("Arcade drive is inactive!");

        }*/

        driveSpeed = 1;
        arcadeDrive = false;
        //if arcadeDrive is true, set drive method to arcade drive; else, tank drive
        if (arcadeDrive)
            myRobot.arcadeDrive(-controller.getX(), controller.getRawAxis(3)*driveSpeed);
        else
            myRobot.tankDrive( controller.getX() * driveSpeed,  controller.getY() * driveSpeed);

    }

    private void winchController() {

        // Control scheme
        if(controller.getRawButton(6)) {

            winch.setSpeed(controller.getRawAxis(5));

        } else if (controller.getRawButton(5)) {

            if (controller.getRawButton(9))
                raiseToHeight(switchEncoderTicks);
            else if (controller.getRawButton(10))
                raiseToHeight(scaleEncoderTicks);
            else if (controller.getRawButton(8))
                raiseToHeight(0);

        } else if(controller.getRawButton(7))

            winchEncoder.reset();


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

        sensor.reset();
        lastTime = System.currentTimeMillis()/1000;
        deltaTime = 0;
        totalTime = 0; //Time delay for 2 seconds

        pid.enable();
        pid.setSetpoint(angle);

        while(!(pid.onTarget() && Math.abs(rotation) < MIN_ROTATIONSPEED) && totalTime < timeout && autonomousEnabled) {
            myRobot.arcadeDrive(0, rotation);
            deltaTime = System.currentTimeMillis()/1000 - lastTime;
            lastTime = System.currentTimeMillis()/1000;
            totalTime += deltaTime;

        }

        System.out.println(rotation + " " + sensor.getAngle());

        pid.disable();

    }
    private int readDistance(){
        arduino.read(8,1,distance);
        return distance[0];
    }
    private void approachWall(){
        int currentDistance = readDistance();
        if(currentDistance<=minDistanceFromWall) {
            currentDistance = readDistance();
            myRobot.arcadeDrive(currentDistance * adjustmentConstant, 0);
        }
    }
}
