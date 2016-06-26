using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;
using Windows.Gaming.Input;

namespace HexPi
{
    class Controller
    {
        Gamepad input = null;
        I2cDevice i2cDevice = null;
        Task inputTask = null;

        double x = 0;
        double y = 0;
        double z = 0;

        double threshold = 0.2;

        LeftLeg LegL1 = new LeftLeg();
        LeftLeg LegL2 = new LeftLeg();
        LeftLeg LegL3 = new LeftLeg();
        RightLeg LegR1 = new RightLeg();
        RightLeg LegR2 = new RightLeg();
        RightLeg LegR3 = new RightLeg();

        byte[] data = new byte[26];



        public void init()
        {
            init_Gamepad();
            init_i2c();
            data[0] = 11;
            data[25] = 22;
            inputTask = Task.Factory.StartNew(() => HandleInputs());
        }

        private void HandleInputs()
        {
            while (true)
            {
                if (input != null && i2cDevice != null)
                {


                    GamepadReading gamepadStatus = input.GetCurrentReading();

                    x = gamepadStatus.LeftThumbstickY;
                    y = gamepadStatus.LeftThumbstickX;
                    z = (gamepadStatus.LeftTrigger - gamepadStatus.RightTrigger);

                    
                    if (x >= threshold || y>= threshold)
                    {
                        if (x > y && x>=threshold)
                        {
                            LegL1.calcPositions(1, x);
                            LegL2.calcPositions(1, x);
                            LegL3.calcPositions(1, x);
                            LegR1.calcPositions(1, x);
                            LegR2.calcPositions(1, x);
                            LegR3.calcPositions(1, x);
                        }
                        else if (y>x && y >= threshold)
                        {
                            LegL1.calcPositions(2, y);
                            LegL2.calcPositions(2, y);
                            LegL3.calcPositions(2, y);
                            LegR1.calcPositions(2, y);
                            LegR2.calcPositions(2, y);
                            LegR3.calcPositions(2, y);
                        }

                    }

                    LegL1.InverseKinematics();
                    LegL2.InverseKinematics();
                    LegL3.InverseKinematics();
                    LegR1.InverseKinematics();
                    LegR2.InverseKinematics();
                    LegR3.InverseKinematics();
                    LegL1.calcData();
                    LegL2.calcData();
                    LegL3.calcData();
                    LegR1.calcData();
                    LegR2.calcData();
                    LegR3.calcData();

                    data[1] = LegR1.getMotorData(0);
                    data[2] = LegR1.getMotorData(1);
                    data[3] = LegR1.getMotorData(2);

                    data[4] = LegR2.getMotorData(0);
                    data[5] = LegR2.getMotorData(1);
                    data[6] = LegR2.getMotorData(2);

                    data[7] = LegR3.getMotorData(0);
                    data[8] = LegR3.getMotorData(1);
                    data[9] = LegR3.getMotorData(2);

                    data[16] = LegL1.getMotorData(0);
                    data[17] = LegL1.getMotorData(1);
                    data[18] = LegL1.getMotorData(2);

                    data[19] = LegL2.getMotorData(0);
                    data[20] = LegL2.getMotorData(1);
                    data[21] = LegL2.getMotorData(2);

                    data[22] = LegL3.getMotorData(0);
                    data[23] = LegL3.getMotorData(1);
                    data[24] = LegL3.getMotorData(2);

                    SendData(data);

                }
            }
        }

        public void init_Gamepad()
        {
            if (Gamepad.Gamepads.Count() > 0)
            {
                input = Gamepad.Gamepads.First();
            }
            else
            {
                Debug.WriteLine("Error: No Gamepad connected!");
            }

            Gamepad.GamepadAdded += GamepadAddedHandler;
            Gamepad.GamepadRemoved += GamepadRemovedHandler;


        }

        private void GamepadRemovedHandler(object sender, Gamepad e)
        {
            input = null;
            Debug.WriteLine("Warning: Gamepad was removed.");
        }

        private void GamepadAddedHandler(object sender, Gamepad e)
        {
            if (Gamepad.Gamepads.Count() > 0)
            {
                input = Gamepad.Gamepads.First();
            }
            else
            {
                Debug.WriteLine("Error: Could not add Gamepad!");
            }
        }

        private async void init_i2c()
        {
            var settings = new I2cConnectionSettings(0x0); // Arduino address


            settings.BusSpeed = I2cBusSpeed.StandardMode;

            string aqs = I2cDevice.GetDeviceSelector("I2C1");

            var dis = await DeviceInformation.FindAllAsync(aqs);

            i2cDevice = await I2cDevice.FromIdAsync(dis[0].Id, settings);
        }

        private void SendData(byte[] b)
        {
            try
            {
                //Debug.WriteLine("A: " + b[0] + " B: " + b[1] + " C: " + b[2]);
                i2cDevice.Write(b);
            }
            catch (Exception e)
            {
                Debug.WriteLine(e.Message);
            }

        }
    }
}
