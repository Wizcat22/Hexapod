/**********************************************************************************************//**
 * @file    servohat.cs
 *
 * @brief   Implements the servohat class.
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
     * @class   ServoHat
     *
     * @brief   A servo hat.
     *
     * @author  Alexander Miller
     * @date    11.08.2016
     **************************************************************************************************/

    class ServoHat
    {
        //Objects
        
        /** @brief   The device. */
        I2cDevice device = null;
        //******

        byte[] a = new byte[2];

        //Functions

        /**********************************************************************************************//**
         * @fn  public async void init()
         *
         * @brief   Initializes this device.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        public async void init()
        {
            try
            {
                I2cConnectionSettings settings = new I2cConnectionSettings(0x2); // Address
                //I2cConnectionSettings settings = new I2cConnectionSettings(0x42); // Address
                settings.BusSpeed = I2cBusSpeed.FastMode;
                settings.SharingMode = I2cSharingMode.Shared;
                string aqs = I2cDevice.GetDeviceSelector("I2C1");
                DeviceInformationCollection dis = await DeviceInformation.FindAllAsync(aqs);
                device = await I2cDevice.FromIdAsync(dis[0].Id, settings);
            }
            catch
            {
                Debug.WriteLine("Error: Servo init failed!");
            }
        }

        /**********************************************************************************************//**
         * @fn  public void write(byte[] b)
         *
         * @brief   Writes the given array to the device.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   b   The array to write.
         **************************************************************************************************/

        public void write(byte[] b)
        {
            try
            {
                if (device != null)
                {
                    //device.Write(b);
                    
                    a[0] = 1;
                    a[1] += 1 ;
                    device.Write(a);   
                }
                else
                {
                    Debug.WriteLine("Error: Servo write failed!");
                }

            }
            catch
            {
                Debug.WriteLine("Error: Servo hat write failed!");
            }

        }
        //******

    }
}
