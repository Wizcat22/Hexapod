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
        //I2cDevice i2cDeviceServo = null;
        //I2cDevice i2cDeviceAccel = null;
        ServoHat servo = new ServoHat();
        Accelerometer accel = new Accelerometer();
        Task inputTask = null;

        double x = 0;
        double y = 0;
        double z = 0;
        double r = 0;

        double threshold = 0.25;
        byte lastDirection = 0;
        enum directions { CENTER, X, Y, ROTATE };

        byte[] data = new byte[26];
        ILeg[] legs = { new LeftLeg(), new RightLeg(), new LeftLeg(), new RightLeg(), new LeftLeg(), new RightLeg() };

        public double X
        {
            get
            {
                return Math.Round(x, 2);
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
            servo.init();
            accel.init();

            //start byte 
            data[0] = 11;
            //end byte
            data[25] = 22;



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

                    accel.read();
                    legs[1].Zoff = Math.Sin(accel.angleYZ) * (legs[1].BodyWidth / 2);
                    legs[3].Zoff = Math.Sin(accel.angleYZ) * (legs[1].BodyWidth / 2);
                    legs[5].Zoff = Math.Sin(accel.angleYZ) * (legs[1].BodyWidth / 2);

                    legs[0].Zoff = -Math.Sin(accel.angleYZ) * (legs[1].BodyWidth / 2);
                    legs[2].Zoff = -Math.Sin(accel.angleYZ) * (legs[1].BodyWidth / 2);
                    legs[4].Zoff = -Math.Sin(accel.angleYZ) * (legs[1].BodyWidth / 2);

                    legs[1].Yoff = Math.Cos(accel.angleYZ) * (legs[1].BodyWidth / 2);
                    legs[3].Yoff = Math.Cos(accel.angleYZ) * (legs[1].BodyWidth / 2);
                    legs[5].Yoff = Math.Cos(accel.angleYZ) * (legs[1].BodyWidth / 2);

                    legs[0].Yoff = Math.Cos(accel.angleYZ) * (legs[1].BodyWidth / 2);
                    legs[2].Yoff = Math.Cos(accel.angleYZ) * (legs[1].BodyWidth / 2);
                    legs[4].Yoff = Math.Cos(accel.angleYZ) * (legs[1].BodyWidth / 2);

                    //legs[1].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);
                    //legs[3].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);
                    //legs[5].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);

                    //legs[0].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);
                    //legs[2].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);
                    //legs[4].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);


                    //Debug.WriteLine("" + accel.angleYZ);
                    //Debug.WriteLine("" + Math.Sin(accel.angleYZ));
                    //Debug.WriteLine("" + Math.Sin(accel.angleYZ) * (legs[1].BodyWidth / 2));




                    GamepadReading gamepadStatus = input.GetCurrentReading();
                    x = gamepadStatus.LeftThumbstickY;
                    y = gamepadStatus.LeftThumbstickX;
                    z = (gamepadStatus.LeftTrigger - gamepadStatus.RightTrigger);
                    r = gamepadStatus.RightThumbstickX;


                    if (Math.Abs(x) >= threshold || Math.Abs(y) >= threshold || Math.Abs(r) >= threshold)
                    {
                        if (Math.Abs(x) > Math.Abs(y) && Math.Abs(x) > Math.Abs(r) && Math.Abs(x) >= threshold)
                        {
                            if (lastDirection != (byte)directions.X && lastDirection != (byte)directions.CENTER)
                            {
                                centerLegs();
                            }
                            lastDirection = (byte)directions.X;
                        }
                        else if (Math.Abs(y) > Math.Abs(x) && Math.Abs(y) > Math.Abs(r) && Math.Abs(y) >= threshold)
                        {
                            if (lastDirection != (byte)directions.Y && lastDirection != (byte)directions.CENTER)
                            {
                                centerLegs();
                            }
                            lastDirection = (byte)directions.Y;
                        }
                        else if (Math.Abs(r) > Math.Abs(x) && Math.Abs(r) > Math.Abs(y) && Math.Abs(r) >= threshold)
                        {
                            if (lastDirection != (byte)directions.ROTATE && lastDirection != (byte)directions.CENTER)
                            {
                                centerLegs();
                            }
                            lastDirection = (byte)directions.ROTATE;
                        }
                    }
                    else
                    {
                        x=0;
                        y = 0;
                        r = 0;
                    }

                    foreach (ILeg l in legs)
                    {
                        switch (lastDirection)
                        {
                            case (byte)directions.CENTER: break;
                            case (byte)directions.X: l.calcPositionX(x); break;
                            case (byte)directions.Y: l.calcPositionY(y); break;
                            case (byte)directions.ROTATE: l.calcPositionR(r); break;
                        }

                        l.inverseKinematics();
                        l.calcData();
                    }

                    setData();
                    servo.write(data);

                }
            }
        }

        private void setData()
        {
            data[1] = legs[0].getMotorData(0);
            data[2] = legs[0].getMotorData(1);
            data[3] = legs[0].getMotorData(2);

            data[4] = legs[2].getMotorData(0);
            data[5] = legs[2].getMotorData(1);
            data[6] = legs[2].getMotorData(2);

            data[7] = legs[4].getMotorData(0);
            data[8] = legs[4].getMotorData(1);
            data[9] = legs[4].getMotorData(2);

            data[16] = legs[1].getMotorData(0);
            data[17] = legs[1].getMotorData(1);
            data[18] = legs[1].getMotorData(2);

            data[19] = legs[3].getMotorData(0);
            data[20] = legs[3].getMotorData(1);
            data[21] = legs[3].getMotorData(2);

            data[22] = legs[5].getMotorData(0);
            data[23] = legs[5].getMotorData(1);
            data[24] = legs[5].getMotorData(2);


            //for (byte i = 0; i < legs.Length; i++)
            //{
            //    if (i < 3)
            //    {
            //        data[1 + (i * 3)] = legs[i].getMotorData(0);
            //        data[2 + (i * 3)] = legs[i].getMotorData(1);
            //        data[3 + (i * 3)] = legs[i].getMotorData(2);
            //    }
            //    else
            //    {
            //        data[7 + (i * 3)] = legs[i].getMotorData(0);
            //        data[8 + (i * 3)] = legs[i].getMotorData(1);
            //        data[9 + (i * 3)] = legs[i].getMotorData(2);
            //    }

            //}
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
                servo.write(data);
                Task.Delay(time).Wait();

                //Center
                l.calcPositionCenter(0);
                lastDirection = (byte)directions.CENTER;
                l.inverseKinematics();
                l.calcData();
                setData();
                servo.write(data);
                Task.Delay(time).Wait();

                //Down
                l.ZPos = 0;
                l.inverseKinematics();
                l.calcData();
                setData();
                servo.write(data);
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
