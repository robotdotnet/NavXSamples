using System;
using System.Collections.Generic;
using System.Linq;
using WPILib;
using WPILib.Extras.NavX;

namespace FieldCentricDrive
{
    /**
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
    public class Robot : SampleRobot
    {
        AHRS ahrs;
        RobotDrive myRobot;
        Joystick stick;
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
                if (stick.GetRawButton(0))
                {
                    ahrs.Reset();
                }
                try
                {
                    //Use the joystick X axis for lateral movement,
                    //Y axis for forward movement, and Z axis for rotation.
                    //Use navX MXP yaw angle to define Field-centric transform.
                    myRobot.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(), stick.GetTwist(), ahrs.GetAngle());
                }
                catch(Exception ex)
                {
                    DriverStation.ReportError("Error communicating with drive syste: " + ex.Message, true);
                }
                Timer.Delay(0.005);		// wait for a motor update time
            }
        }

        /**
         * Runs during test mode
         */
        public override void Test()
        {
        }
    }
}
