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
    /**********************************************************************************************//**
     * @class   Accelerometer
     *
     * @brief   An accelerometer.
     *
     * @author  Alexander Miller
     * @date    11.08.2016
     **************************************************************************************************/

    class Accelerometer
    {
        //Objects
        
        /** @brief   The device. */
        I2cDevice device = null;
        //******

        //Fields

        /** @brief   The x coordinate. */
        private double x = 0.0;

        /** @brief   The y coordinate. */
        private double y = 0.0;

        /** @brief   The z coordinate. */
        private double z = 0.0;

        /** @brief   The x coordinate. */
        private double offsetX = 0.0;

        /** @brief   The y coordinate. */
        private double offsetY = 0.0;

        /** @brief   The z coordinate. */
        private double offsetZ = 0.0;
        //******

        //Arrays

        /** @brief   Buffer for write data. */
        byte[] writeBuffer = new byte[] { 0x28 };

        /** @brief   Buffer for read data. */
        byte[] readBuffer = new byte[6];
        //******

        //Properties


        public Accelerometer()
        {
            init();
            read();
            offsetX = x;
            offsetY = y;
            offsetZ = z;
        }


        /**********************************************************************************************//**
         * @property    public double roll
         *
         * @brief   Gets the angle of the rotation around the y axis.
         *
         * @return  The angle y.
         **************************************************************************************************/

        public double Pitch
        {
            get
            {
                double xz = Math.Round((Math.Acos(x / (Math.Sqrt(x * x + z * z))) - Math.PI / 2) * 100 * Math.PI / 180, 2);
                

                if (double.IsNaN(xz) || double.IsInfinity(xz))
                {
                    return 0;
                }
                return xz;
            }
        }

        /**********************************************************************************************//**
         * @property    public double angleYZ
         *
         * @brief   Gets the angle of the rotation around the x axis.
         *
         * @return  The angle around x.
         **************************************************************************************************/

        public double Roll
        {
            get
            {
                double yz = Math.Round((Math.Acos(y / (Math.Sqrt(y * y + z * z))) - Math.PI / 2)*100 * Math.PI / 180, 2);
                if (double.IsNaN(yz) ||double.IsInfinity(yz))
                {
                    return 0;
                }
 
                return yz;
            }
        }
        //******

        //Functions

        /**********************************************************************************************//**
         * @fn  public async void init()
         *
         * @brief   Initializes the device.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

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

        /**********************************************************************************************//**
         * @fn  public void read()
         *
         * @brief   Read data for each axis.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        public void read()
        {
            try
            {

                if (device != null)
                {
                    int length= 20;
                    bool status = false;
                    byte[] STATUS_REG = new byte[1];
                    device.Write(new byte[] { 0x10, 0x80 });
                    for (int i = 0; i < length; i++)
                    {
                        while (!status)
                        {
                            device.Write(new byte[] { 0x1E });
                            device.Read(STATUS_REG);
                            status = (STATUS_REG[0] & 1) == 1;
                        }
                        device.Write(writeBuffer);
                        device.Read(readBuffer);
                        x += Math.Round(((Int16)(readBuffer[1] << 8 | readBuffer[0]) / 16383.0), 2);
                        y += Math.Round(((Int16)(readBuffer[3] << 8 | readBuffer[2]) / 16383.0), 2);
                        z += Math.Round(((Int16)(readBuffer[5] << 8 | readBuffer[4]) / 16383.0), 2);
                        
                    }
                    x = Math.Round(x / length,2);
                    y = Math.Round(y / length, 2);
                    z = Math.Round(z / length, 2);
                    //Debug.WriteLine("X= " + x + " Y= " + y + " Z= " + z);
                }
                else
                {
                    Debug.WriteLine("Error: Accelerometer no device!");
                }

            }
            catch
            {
                Debug.WriteLine("Error: Accelerometer read failed!");
            }
        }
        //******
    }
}
