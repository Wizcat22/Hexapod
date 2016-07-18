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

    class Accelerometer
    {
        //Objects
        I2cDevice device = null;
        //******

        //Fields
        bool ready = false;
        private double x = 0.0;
        private double y = 0.0;
        private double z = 0.0;
        //******

        //Arrays
        byte[] writeBuffer = new byte[] { 0x28 };
        byte[] readBuffer = new byte[6];
        //******

        //Properties
        public double angleXZ
        {
            get
            {
                double xz = Math.Round(Math.Acos(x / (Math.Sqrt(x * x + z * z))) - Math.PI / 2, 2);
                if(double.IsNaN(xz) || double.IsInfinity(xz))
                {
                    return 0;
                }
                return xz;
            }
        }
        public double angleYZ
        {
            get
            {
                double yz = Math.Round(Math.Acos(y / (Math.Sqrt(y * y + z * z))) - Math.PI / 2, 2);
                if (double.IsNaN(yz) ||double.IsInfinity(yz))
                {
                    return 0;
                }
                return yz;
            }
        }
        //******

        //Functions
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

        public void read()
        {
            try
            {

                if (device != null)
                {
                    device.Write(new byte[] { 0x10, 0x80 });
                    device.Write(writeBuffer);
                    device.Read(readBuffer);
                    x = Math.Round((Int16)(readBuffer[1] << 8 | readBuffer[0]) / 16383.0, 2);
                    y = Math.Round((Int16)(readBuffer[3] << 8 | readBuffer[2]) / 16383.0, 2);
                    z = Math.Round((Int16)(readBuffer[5] << 8 | readBuffer[4]) / 16383.0, 2);

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
