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
        //Objects
        Gamepad input = null;
        Hexapod robot = new Hexapod();
        Task inputTask = null;
        //******

        //Fields
        double x = 0;
        double y = 0;
        double z = 0;
        double r = 0;

        double threshold = 0.25;
        //******

        //Enums
        public enum directions { CENTER, X, Y, ROTATE };
        //******

        //Properties
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
        //******

        //Functions
        public void init()
        {
            init_Gamepad();
            robot.init();

            inputTask = Task.Factory.StartNew(() => handleInputs());
        }

        private void handleInputs()
        {
            while (true)
            {


                Task.Delay(10).Wait();


                if (input != null)
                {
                    GamepadReading gamepadStatus = input.GetCurrentReading();
                    x = gamepadStatus.LeftThumbstickY;
                    y = -gamepadStatus.LeftThumbstickX;
                    z = (gamepadStatus.LeftTrigger - gamepadStatus.RightTrigger);
                    r = gamepadStatus.RightThumbstickX;

                }
                else
                {
                    if (Gamepad.Gamepads.Count() > 0)
                    {
                        input = Gamepad.Gamepads.First();
                    }
                }


                if (Math.Abs(x) >= threshold || Math.Abs(y) >= threshold || Math.Abs(r) >= threshold)
                {
                    if (Math.Abs(x) > Math.Abs(y) && Math.Abs(x) > Math.Abs(r) && Math.Abs(x) >= threshold)
                    {

                        robot.move(x, (byte)directions.X);
                    }
                    else if (Math.Abs(y) > Math.Abs(x) && Math.Abs(y) > Math.Abs(r) && Math.Abs(y) >= threshold)
                    {

                        robot.move(y, (byte)directions.Y);
                    }
                    else if (Math.Abs(r) > Math.Abs(x) && Math.Abs(r) > Math.Abs(y) && Math.Abs(r) >= threshold)
                    {

                        robot.move(r, (byte)directions.ROTATE);
                    }
                }
                else
                {
                    robot.move(0, (byte)directions.CENTER);
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
                Debug.WriteLine("Warning: No Gamepad connected!");
            }

            Gamepad.GamepadAdded += gamepadAddedHandler;
            Gamepad.GamepadRemoved += gamepadRemovedHandler;


        }

        private void gamepadRemovedHandler(object sender, Gamepad e)
        {
            input = null;
            Debug.WriteLine("Warning: Gamepad was removed.");
        }

        private void gamepadAddedHandler(object sender, Gamepad e)
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

        public double getGuiData(byte n, byte xyz)
        {
            return robot.getLegPos(n, xyz);

        }
        //******

    }
}
