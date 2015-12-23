using System;
using System.Collections.Generic;
using System.Linq;

using WPILib;
using WPILib.Extras.NavX;
using WPILib.SmartDashboard;

namespace DataMonitor
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
        Joystick stick;
        public Robot()
        {
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
            Timer.Delay(2.0);		//    for 2 seconds
        }

        /**
         * Runs the motors with arcade steering.
         */
        public override void OperatorControl()
        {
            while (IsOperatorControl && IsEnabled)
            {
                Timer.Delay(0.020);     /* wait for one motor update time period (50Hz)     */

                bool zero_yaw_pressed = stick.GetTrigger();
                if (zero_yaw_pressed)
                {
                    ahrs.ZeroYaw();
                }

                /* Display 6-axis Processed Angle Data                                      */
                SmartDashboard.PutBoolean("IMU_Connected", ahrs.IsConnected());
                SmartDashboard.PutBoolean("IMU_IsCalibrating", ahrs.IsCalibrating());
                SmartDashboard.PutNumber("IMU_Yaw", ahrs.GetYaw());
                SmartDashboard.PutNumber("IMU_Pitch", ahrs.GetPitch());
                SmartDashboard.PutNumber("IMU_Roll", ahrs.GetRoll());

                /* Display tilt-corrected, Magnetometer-based heading (requires             */
                /* magnetometer calibration to be useful)                                   */

                SmartDashboard.PutNumber("IMU_CompassHeading", ahrs.GetCompassHeading());

                /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
                SmartDashboard.PutNumber("IMU_FusedHeading", ahrs.GetFusedHeading());

                /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
                /* path for upgrading from the Kit-of-Parts gyro to the navx MXP            */

                SmartDashboard.PutNumber("IMU_TotalYaw", ahrs.GetAngle());
                SmartDashboard.PutNumber("IMU_YawRateDPS", ahrs.GetRate());

                /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

                SmartDashboard.PutNumber("IMU_Accel_X", ahrs.GetWorldLinearAccelX());
                SmartDashboard.PutNumber("IMU_Accel_Y", ahrs.GetWorldLinearAccelY());
                SmartDashboard.PutBoolean("IMU_IsMoving", ahrs.IsMoving());
                SmartDashboard.PutBoolean("IMU_IsRotating", ahrs.IsRotating());

                /* Display estimates of velocity/displacement.  Note that these values are  */
                /* not expected to be accurate enough for estimating robot position on a    */
                /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
                /* of these errors due to single (velocity) integration and especially      */
                /* double (displacement) integration.                                       */

                SmartDashboard.PutNumber("Velocity_X", ahrs.GetVelocityX());
                SmartDashboard.PutNumber("Velocity_Y", ahrs.GetVelocityY());
                SmartDashboard.PutNumber("Displacement_X", ahrs.GetDisplacementX());
                SmartDashboard.PutNumber("Displacement_Y", ahrs.GetDisplacementY());

                /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
                /* NOTE:  These values are not normally necessary, but are made available   */
                /* for advanced users.  Before using this data, please consider whether     */
                /* the processed data (see above) will suit your needs.                     */

                SmartDashboard.PutNumber("RawGyro_X", ahrs.GetRawGyroX());
                SmartDashboard.PutNumber("RawGyro_Y", ahrs.GetRawGyroY());
                SmartDashboard.PutNumber("RawGyro_Z", ahrs.GetRawGyroZ());
                SmartDashboard.PutNumber("RawAccel_X", ahrs.GetRawAccelX());
                SmartDashboard.PutNumber("RawAccel_Y", ahrs.GetRawAccelY());
                SmartDashboard.PutNumber("RawAccel_Z", ahrs.GetRawAccelZ());
                SmartDashboard.PutNumber("RawMag_X", ahrs.GetRawMagX());
                SmartDashboard.PutNumber("RawMag_Y", ahrs.GetRawMagY());
                SmartDashboard.PutNumber("RawMag_Z", ahrs.GetRawMagZ());
                SmartDashboard.PutNumber("IMU_Temp_C", ahrs.GetTempC());

                /* Omnimount Yaw Axis Information                                           */
                /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
                AHRS.BoardYawAxis yaw_axis = ahrs.GetBoardYawAxis();
                SmartDashboard.PutString("YawAxisDirection", yaw_axis.Up ? "Up" : "Down");
                SmartDashboard.PutNumber("YawAxis", (int)yaw_axis.BoardAxis);

                /* Sensor Board Information                                                 */
                SmartDashboard.PutString("FirmwareVersion", ahrs.GetFirmwareVersion());

                /* Quaternion Data                                                          */
                /* Quaternions are fascinating, and are the most compact representation of  */
                /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
                /* from the Quaternions.  If interested in motion processing, knowledge of  */
                /* Quaternions is highly recommended.                                       */
                SmartDashboard.PutNumber("QuaternionW", ahrs.GetQuaternionW());
                SmartDashboard.PutNumber("QuaternionX", ahrs.GetQuaternionX());
                SmartDashboard.PutNumber("QuaternionY", ahrs.GetQuaternionY());
                SmartDashboard.PutNumber("QuaternionZ", ahrs.GetQuaternionZ());

                /* Connectivity Debugging Support                                           */
                SmartDashboard.PutNumber("IMU_Byte_Count", ahrs.GetByteCount());
                SmartDashboard.PutNumber("IMU_Update_Count", ahrs.GetUpdateCount());
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
