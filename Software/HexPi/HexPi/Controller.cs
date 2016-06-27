using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;
using Windows.Gaming.Input;
using System.Threading;

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
        double r = 0;

        double threshold = 0.25;
        byte lastDirection = 0;
        enum directions{CENTER,X,Y,ROTATE};

        byte[] data = new byte[26];
        ILeg[] legs = { new LeftLeg(), new RightLeg(), new LeftLeg(), new RightLeg(), new LeftLeg(), new RightLeg() };

        public double X
        {
            get
            {
                return Math.Round(x,2);
            }
        }

        public double Y
        {
            get
            {
                return Math.Round(y, 2);
            }
        }

        public double Z
        {
            get
            {
                return Math.Round(z, 2);
            }
        }

        public double R
        {
            get
            {
                return Math.Round(z, 2);
            }
        }

        public void init()
        {
            init_Gamepad();
            init_i2c();
            //start byte 
            data[0] = 11;
            //end byte
            data[25] = 22;

            //for (byte i = 0; i < legs.Length; i++)
            //{
                
            //    legs[i] = ((i % 2) == 0) ? (ILeg)new LeftLeg() : (ILeg)new RightLeg();
                
            //}

            //TEST
            legs[0].TOffset = 25;
            legs[3].TOffset = 25;
            legs[4].TOffset = 25;
            legs[1].TOffset = 75;
            legs[2].TOffset = 75;
            legs[5].TOffset = 75;
            ///TEST

            inputTask = Task.Factory.StartNew(() => HandleInputs());
        }

        private void HandleInputs()
        {
            while (true)
            {
                if (input != null)
                {

                    Task.Delay(10).Wait();
                    GamepadReading gamepadStatus = input.GetCurrentReading();
                    x = gamepadStatus.LeftThumbstickY;
                    y = gamepadStatus.LeftThumbstickX;
                    z = (gamepadStatus.LeftTrigger - gamepadStatus.RightTrigger);
                    r = gamepadStatus.RightThumbstickX;


                    if (Math.Abs(x) >= threshold || Math.Abs(y) >= threshold || Math.Abs(r) >= threshold)
                    {
                        if (Math.Abs(x) > Math.Abs(y) && Math.Abs(x) > Math.Abs(r) && Math.Abs(x) >= threshold)
                        {
                            if(lastDirection != (byte)directions.X && lastDirection != (byte)directions.CENTER)
                            {
                                centerLegs();
                            }
                            foreach (ILeg l in legs)
                            {
                                l.calcPositionX(x);
                                lastDirection = (byte)directions.X;
                            }
                        }
                        else if (Math.Abs(y) > Math.Abs(x) && Math.Abs(y) > Math.Abs(r) && Math.Abs(y) >= threshold)
                        {
                            if (lastDirection != (byte)directions.Y && lastDirection != (byte)directions.CENTER)
                            {
                                centerLegs();
                            }
                            foreach (ILeg l in legs)
                            {
                                l.calcPositionY(y);
                                lastDirection = (byte)directions.Y;
                            }
                        }
                        else if (Math.Abs(r) > Math.Abs(x) && Math.Abs(r) > Math.Abs(y) && Math.Abs(r) >= threshold)
                        {
                            if (lastDirection != (byte)directions.ROTATE && lastDirection != (byte)directions.CENTER)
                            {
                                centerLegs();
                            }
                            foreach (ILeg l in legs)
                            {
                                l.calcPositionR(r);
                                lastDirection = (byte)directions.ROTATE;
                            }
                        }

                    }

                    foreach (ILeg l in legs)
                    {
                        l.inverseKinematics();
                        l.calcData();
                    }

                    setData();
                    sendData(data);

                }
            }
        }

        private void setData()
        {
            for (byte i = 0; i < legs.Length; i++)
            {
                if (i < 3)
                {
                    data[1 + (i * 3)] = legs[i].getMotorData(0);
                    data[2 + (i * 3)] = legs[i].getMotorData(1);
                    data[3 + (i * 3)] = legs[i].getMotorData(2);
                }
                else
                {
                    data[7 + (i * 3)] = legs[i].getMotorData(0);
                    data[8 + (i * 3)] = legs[i].getMotorData(1);
                    data[9 + (i * 3)] = legs[i].getMotorData(2);
                }

            }
        }

        private void init_Gamepad()
        {
            
            
            if (Gamepad.Gamepads.Count() > 0)
            {
                input = Gamepad.Gamepads.First();
                Debug.WriteLine("Gamepad connected!");
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
            try
            {
                var settings = new I2cConnectionSettings(0x0); // Arduino address


                settings.BusSpeed = I2cBusSpeed.StandardMode;

                string aqs = I2cDevice.GetDeviceSelector("I2C1");

                var dis = await DeviceInformation.FindAllAsync(aqs);

                i2cDevice = await I2cDevice.FromIdAsync(dis[0].Id, settings);
            }
            catch (Exception e)
            {
                Debug.WriteLine("Error: I2C device not found!");
            }

        }

        private void sendData(byte[] b)
        {
            try
            {
                if (i2cDevice != null)
                {
                    //Debug.WriteLine("A: " + b[0] + " B: " + b[1] + " C: " + b[2]);
                    i2cDevice.Write(b);
                }

            }
            catch (Exception e)
            {
                Debug.WriteLine(e.Message);
            }

        }

        private void centerLegs()
        {
            int time = 100;
            foreach (ILeg l in legs)
            {
                //Up
                l.ZPos = l.StepSizeZ;
                l.inverseKinematics();
                l.calcData();
                setData();
                sendData(data);
                Task.Delay(time).Wait();

                //Center
                l.calcPositionCenter(0);
                lastDirection = (byte)directions.CENTER;
                l.inverseKinematics();
                l.calcData();
                setData();
                sendData(data);
                Task.Delay(time).Wait();

                //Down
                l.ZPos = 0;
                l.inverseKinematics();
                l.calcData();
                setData();
                sendData(data);
                Task.Delay(time).Wait();

            }
        }

        public double getLegPos(byte n, byte xyz)
        {
            switch (xyz)
            {
                case 0: return Math.Round(legs[n].XPos);
                case 1: return Math.Round(legs[n].YPos);
                case 2: return Math.Round(legs[n].ZPos);
                default: return 0.0;
            }
        }


    }
}
