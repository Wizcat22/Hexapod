/**********************************************************************************************//**
 * @file    accelerometer.cs
 *
 * @brief   Implements the accelerometer class.
 **************************************************************************************************/

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;

namespace HexPi
{

    /**
     * @class   Accelerometer
     *
     * @brief   An accelerometer.
     *
     * @author  Alex
     * @date    13.08.2017
     */

    class Accelerometer
    {
        #region Objects

        /** @brief   The I2C device. */
        I2cDevice device = null;

        #endregion Objects

        #region Fields

        /** @brief   The x coordinate. */
        private double x = 0.0;

        /** @brief   The y coordinate. */
        private double y = 0.0;

        /** @brief   The z coordinate. */
        private double z = 0.0;

        #endregion Fields

        #region Arrays
        /** @brief   Array for the last 3 x-Values */
        double[] xVal = new double[3];
        /** @brief   Array for the last 3 y-Values */
        double[] yVal = new double[3];
        /** @brief   Array for the last 3 z-Values */
        double[] zVal = new double[3];

        /** @brief   Buffer for i2c-write (address of first register). */
        byte[] writeBuffer = new byte[] { 0x28 };

        /** @brief   Buffer for i2c-read. */
        byte[] readBuffer = new byte[6];
        #endregion Arrays

        #region Properties


        /**
         * @property    public double Pitch
         *
         * @brief   Gets the pitch
         *
         * @return  The pitch.
         */

        public double Pitch
        {
            get
            {   
                //calc pitch
                double xz = Math.Round((Math.Acos(x / (Math.Sqrt(x * x + z * z))) - Math.PI / 2) * 100 * Math.PI / 180, 2);

                //check for errors
                if (double.IsNaN(xz) || double.IsInfinity(xz))
                {
                    return 0;
                }
                return xz;
            }
        }


        /**
         * @property    public double Roll
         *
         * @brief   Gets the roll
         *
         * @return  The roll.
         */

        public double Roll
        {
            get
            {
                //calc roll
                double yz = Math.Round((Math.Acos(y / (Math.Sqrt(y * y + z * z))) - Math.PI / 2) * 100 * Math.PI / 180, 2);
                //check for errors
                if (double.IsNaN(yz) || double.IsInfinity(yz))
                {
                    return 0;
                }

                return yz;
            }
        }
        #endregion Properties

        #region Functions

        /**
         * @fn  public Accelerometer()
         *
         * @brief   Default constructor
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        public Accelerometer()
        {
            init();
        }



        /**
         * @fn  public async void init()
         *
         * @brief   Initializes this object
         *
         * @author  Alex
         * @date    13.08.2017
         */

        public async void init()
        {
            try
            {
                I2cConnectionSettings settings = new I2cConnectionSettings(0x6B); // Address
                settings.BusSpeed = I2cBusSpeed.StandardMode;
                settings.SharingMode = I2cSharingMode.Shared;
                string aqs = I2cDevice.GetDeviceSelector("I2C1");
                DeviceInformationCollection dis = await DeviceInformation.FindAllAsync(aqs);
                device = await I2cDevice.FromIdAsync(dis[0].Id, settings);
            }
            catch
            {
                Debug.WriteLine("Error: Accelerometer init failed!");
            }
        }


        /**
         * @fn  public void read()
         *
         * @brief   Read data for each axis.
         *          1. Initialize the sensor with 0x10,0x80.  
         *          2. Wait until the sensor has new data.  
         *          3. Write address of the first value (address increments automaticaly after each read)  
         *          4. Combine the 2 bytes to a 16bit value and add it to the last value  
         *          5. Divide the sum with number of measurements  
         *          6. Save last value in xVal-Array  
         *          7. Calculate x by using a weighted average (20% , 50% , 25%)
         *          
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        public void read()
        {
            try
            {
                if (device != null)
                {
                    //number of measurements 
                    int length = 10;
                    //status flag
                    bool status = false;
                    //read buffer for register status
                    byte[] STATUS_REG = new byte[1];
                    //write buffer with initializing
                    device.Write(new byte[] { 0x10, 0x80 });
                    for (int i = 0; i < length; i++)
                    {
                        //while no new value
                        while (!status)
                        {
                            //set status address
                            device.Write(new byte[] { 0x1E });
                            //read status byte
                            device.Read(STATUS_REG);
                            status = (STATUS_REG[0] & 1) == 1;
                        }
                        status = false;
                        //write address of first value
                        device.Write(writeBuffer);
                        //read 6 bytes
                        device.Read(readBuffer);
                        //combine 2 bytes into a 16bit number and add it
                        x += Math.Round(((Int16)(readBuffer[1] << 8 | readBuffer[0]) / 16383.0), 2);
                        y += Math.Round(((Int16)(readBuffer[3] << 8 | readBuffer[2]) / 16383.0), 2);
                        z += Math.Round(((Int16)(readBuffer[5] << 8 | readBuffer[4]) / 16383.0), 2);

                    }

                    //save new value and remove old value
                    xVal[2] = xVal[1];
                    xVal[1] = xVal[0];
                    xVal[0] = Math.Round(x / length, 2);
                    //weighted average ()25%,50%,25%)
                    x = 0.25 * xVal[0] + 0.5 * xVal[1] + 0.25 * xVal[2];

                    yVal[2] = yVal[1];
                    yVal[1] = yVal[0];
                    yVal[0] = Math.Round(y / length, 2);
                    y = 0.25 * yVal[0] + 0.5 * yVal[1] + 0.25 * yVal[2];

                    zVal[2] = zVal[1];
                    zVal[1] = zVal[0];
                    zVal[0] = Math.Round(z / length, 2);
                    z = 0.25 * zVal[0] + 0.5 * zVal[1] + 0.25 * zVal[2];
                }
                else
                {
                    Debug.WriteLine("Error: Accelerometer: No device!");
                }

            }
            catch
            {
                Debug.WriteLine("Error: Accelerometer: Read failed!");
            }
        }
        #endregion Functions

    }
}
