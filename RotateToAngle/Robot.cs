using System;
using System.Collections.Generic;
using System.Linq;

using WPILib;
using WPILib.Extras.NavX;
using WPILib.Interfaces;
using WPILib.LiveWindow;
using WPILib.SmartDashboard;

namespace RotateToAngle
{
    /**

      *  This is a demo program showing the use of the navX MXP to implement
 * a "rotate to angle" feature.
 *
 * This example will automatically rotate the robot to one of four
 * angles (0, 90, 180 and 270 degrees).
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
    public class Robot : SampleRobot, IPIDOutput
    {
        AHRS ahrs;
        RobotDrive myRobot;
        Joystick stick;
        PIDController turnController;

        double rotateToAngleRate;

        /* The following PID Controller coefficients will need to be tuned */
        /* to match the dynamics of your drive system.  Note that the      */
        /* SmartDashboard in Test mode has support for helping you tune    */
        /* controllers by displaying a form where you can enter new P, I,  */
        /* and D constants and test the mechanism.                         */

        const double kP = 0.03;
        const double kI = 0.00;
        const double kD = 0.00;
        const double kF = 0.00;

        const double kToleranceDegrees = 2.0;

        public Robot()
        {
            myRobot = new RobotDrive(0, 1, 2, 3);
            myRobot.Expiration = 0.1;
            stick = new Joystick(0);
            try
            {
                /* Communicate w/navX MXP via the MXP SPI Bus.                                     */
                /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
                /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
                ahrs = new AHRS(SPI.Port.MXP);
            }
            catch (Exception ex)
            {
                DriverStation.ReportError("Error instantiating navX MXP: " + ex.Message, true);
            }
            turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
            turnController.SetInputRange(-180f, 180f);
            turnController.SetOutputRange(-1.0, 1.0);
            turnController.SetAbsoluteTolerance(kToleranceDegrees);
            turnController.Continouous = true;

            /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
            /* tuning of the Turn Controller's P, I and D coefficients.            */
            /* Typically, only the P value needs to be modified.                   */
            LiveWindow.AddActuator("DriveSystem", "RotateController", turnController);
        }
        /**
         * Drive left & right motors for 2 seconds then stop
         */
        public override void Autonomous()
        {
            myRobot.SafetyEnabled = false;
            myRobot.Drive(-0.5, 0.0);	// drive forwards half speed
            Timer.Delay(2.0);		//    for 2 seconds
            myRobot.Drive(0.0, 0.0);	// stop robot
        }

        /**
         * Runs the motors with arcade steering.
         */
        public override void OperatorControl()
        {
            myRobot.SafetyEnabled = true;
            while (IsOperatorControl && IsEnabled)
            {
                bool rotateToAngle = false;
                if (stick.GetRawButton(1))
                {
                    ahrs.Reset();
                }
                if (stick.GetRawButton(2))
                {
                    turnController.Setpoint = (0.0f);
                    rotateToAngle = true;
                }
                else if (stick.GetRawButton(3))
                {
                    turnController.Setpoint = (90.0f);
                    rotateToAngle = true;
                }
                else if (stick.GetRawButton(4))
                {
                    turnController.Setpoint = (179.9f);
                    rotateToAngle = true;
                }
                else if (stick.GetRawButton(5))
                {
                    turnController.Setpoint = (-90.0f);
                    rotateToAngle = true;
                }
                double currentRotationRate;
                if (rotateToAngle)
                {
                    turnController.Enable();
                    currentRotationRate = rotateToAngleRate;
                }
                else {
                    turnController.Disable();
                    currentRotationRate = stick.GetTwist();
                }
                try
                {
                    //Use the joystick X axis for lateral movement,
                    //Y axis for forward movement, and Z axis for rotation.
                    //Use navX MXP yaw angle to define Field-centric transform.
                    myRobot.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(), currentRotationRate, ahrs.GetAngle());
                }
                catch (Exception ex)
                {
                    DriverStation.ReportError("Error communicating with drive syste: " + ex.Message, true);
                }
                Timer.Delay(0.005);		// wait for a motor update time
            }
        }

        public void PidWrite(double value)
        {
            rotateToAngleRate = value;
        }

        /**
         * Runs during test mode
         */
        public override void Test()
        {
        }
    }
}
