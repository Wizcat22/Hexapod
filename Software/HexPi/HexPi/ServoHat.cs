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
    class ServoHat
    {
        //Objects
        I2cDevice device = null;
        //******

        //Functions
        public async void init()
        {
            try
            {
                I2cConnectionSettings settings = new I2cConnectionSettings(0x42); // Address
                settings.BusSpeed = I2cBusSpeed.StandardMode;
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

        public void write(byte[] b)
        {
            try
            {
                if (device != null)
                {
                    device.Write(b);     
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
