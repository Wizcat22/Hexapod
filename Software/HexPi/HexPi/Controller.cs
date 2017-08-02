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
using Windows.System;

namespace HexPi
{
    /**********************************************************************************************//**
     * @class   Controller
     *
     * @brief   A controller class.
     *
     * @author  Alexander Miller
     * @date    11.08.2016
     **************************************************************************************************/

    class Controller
    {
        #region Objects
        /** @brief   The input. */
        Gamepad input = null;

        /** @brief   The robot. */
        Hexapod robot = new Hexapod();

        /** @brief   The input task. */
        Task inputTask = null;
        #endregion Objects

        #region Fields
        /** @brief   The x-axis input. */
        double x = 0;

        /** @brief   The y-axis input. */
        double y = 0;

        /** @brief   The z-axis input. */
        double z = 0;

        /** @brief   The a-axis input. */
        double a = 0;

        /** @brief   The b-axis input */
        double b = 0;

        /** @brief   The threshold for the input. */
        const double threshold = 0.25;

        const byte timeframe = 10;

        #endregion Fields

        #region Enums

        /**********************************************************************************************//**
         * @enum    directions
         *
         * @brief   Values that represent directions.
         **************************************************************************************************/

        public enum directions { CENTER, XY, ROTATE, TURN };
        //******

        public enum modes { WALK, POSE, SHUTDOWN, TERRAIN, BALANCE };

        #endregion Enums

        #region Properties

        #endregion Properties

        #region Functions

        /**********************************************************************************************//**
         * @fn  public void init()
         *
         * @brief   Initializes this object.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        public void init()
        {
            init_Gamepad();
            robot.init();

            inputTask = Task.Factory.StartNew(() => handleInputs());
        }

        /**********************************************************************************************//**
         * @fn  private void handleInputs()
         *
         * @brief   Handles the inputs.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        private void handleInputs()
        {
            DateTime newDate = DateTime.Now;
            DateTime oldDate = DateTime.Now;
            TimeSpan dif;
            byte mode = 0;

            while (true)
            {
                newDate = DateTime.Now;
                dif = newDate - oldDate;

                while (dif.Milliseconds < timeframe)
                {
                    newDate = DateTime.Now;
                    dif = newDate - oldDate;
                }


                oldDate = DateTime.Now;

                //Task.Delay(0).Wait();

                if (input != null)
                {
                    GamepadReading gamepadStatus = input.GetCurrentReading();
                    x = -gamepadStatus.LeftThumbstickY;
                    y = gamepadStatus.LeftThumbstickX;
                    z = (gamepadStatus.LeftTrigger - gamepadStatus.RightTrigger);
                    a = gamepadStatus.RightThumbstickX;
                    b = gamepadStatus.RightThumbstickY;


                    //Shutdown = A&B&X&Y&LThumb
                    if (gamepadStatus.Buttons == (GamepadButtons.A | GamepadButtons.B | GamepadButtons.X | GamepadButtons.Y | GamepadButtons.LeftThumbstick))
                    {
                        mode = (int)modes.SHUTDOWN;
                    }
                    //Pose = RightShoulder
                    else if (gamepadStatus.Buttons == GamepadButtons.RightShoulder)
                    {
                        mode = (int)modes.POSE;
                    }
                    //Terrain = LeftShoulder
                    else if (gamepadStatus.Buttons == GamepadButtons.LeftShoulder)
                    {
                        mode = (int)modes.TERRAIN;
                    }
                    //BALANCE = A
                    else if (gamepadStatus.Buttons == GamepadButtons.A)
                    {
                        mode = (int)modes.BALANCE;
                    }
                    //Walk = default
                    else
                    {
                        mode = (int)modes.WALK;
                    }

                }
                else
                {
                    if (Gamepad.Gamepads.Count() > 0)
                    {
                        input = Gamepad.Gamepads.First();
                    }
                }

                switch (mode)
                {
                    case (byte)modes.WALK:
                        walk((byte)modes.WALK);
                        break;
                    case (byte)modes.BALANCE:
                        walk((byte)modes.BALANCE);
                        break;
                    case (byte)modes.TERRAIN:
                        walk((byte)modes.TERRAIN);
                        break;
                    case (byte)modes.POSE:
                        pose();
                        break;
                    case (byte)modes.SHUTDOWN:
                        shutdown();
                        break;
                    default:
                        break;
                }



            }
        }

        /**********************************************************************************************//**
         * @fn  private void init_Gamepad()
         *
         * @brief   Initializes the gamepad for use as input device.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        private void init_Gamepad()
        {


            if (Gamepad.Gamepads.Count() > 0)
            {
                input = Gamepad.Gamepads.First();
                Debug.WriteLine("Info: Gamepad connected!");
            }
            else
            {
                Debug.WriteLine("Warning: No Gamepad connected!");
            }

            Gamepad.GamepadAdded += gamepadAddedHandler;
            Gamepad.GamepadRemoved += gamepadRemovedHandler;


        }

        /**********************************************************************************************//**
         * @fn  private void gamepadRemovedHandler(object sender, Gamepad e)
         *
         * @brief   Handler, called when the gamepad is removed.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   sender  Source of the event.
         * @param   e       The Gamepad to process.
         **************************************************************************************************/

        private void gamepadRemovedHandler(object sender, Gamepad e)
        {
            input = null;
            Debug.WriteLine("Warning: Gamepad was removed.");
        }

        /**********************************************************************************************//**
         * @fn  private void gamepadAddedHandler(object sender, Gamepad e)
         *
         * @brief   Handler, called when the gamepad is added.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   sender  Source of the event.
         * @param   e       The Gamepad to process.
         **************************************************************************************************/

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

        //Initialize shutdown
        private void shutdown()
        {
            Debug.WriteLine("WARNING: SYSTEM SHUTDOWN INITIALIZED!");
            ShutdownManager.BeginShutdown(ShutdownKind.Shutdown, new TimeSpan(0));
        }

        // Walk in xy direction / turn in r direction
        private void walk(byte mode)
        {
            //check if any of the axis is above the threshold
            if (Math.Abs(x) >= threshold || Math.Abs(y) >= threshold || Math.Abs(a) >= threshold)
            {
                //if the xy coordionate is bigger then all the other coordinates and bigger then the threshold move the robot in xy-direction
                //if ((Math.Abs(x) > Math.Abs(a) && Math.Abs(x) >= threshold) || (Math.Abs(y) > Math.Abs(a) && Math.Abs(y) >= threshold))
                if (((Math.Abs(x) >= threshold) || Math.Abs(y) >= threshold) && (Math.Abs(a) < threshold))
                {
                    robot.move(x, y, 0, (byte)directions.XY, mode);
                }
                else if (((Math.Abs(x) >= threshold) || Math.Abs(y) >= threshold) && (Math.Abs(a) >= threshold))
                {

                    robot.move(x, y, a, (byte)directions.TURN, mode);
                }
                //if the r coordionate is bigger then all the other coordinates and bigger then the threshold turn the robot
                //else if (Math.Abs(a) > Math.Abs(x) && Math.Abs(a) > Math.Abs(y) && Math.Abs(a) >= threshold)
                else if (Math.Abs(a) >= threshold && Math.Abs(x) < threshold && Math.Abs(y) < threshold)
                {

                    robot.move(0, 0, a, (byte)directions.ROTATE, mode);
                }
            }
            //do nothing if none of the values is above the threshold
            else
            {
                robot.move(0, 0, 0, (byte)directions.CENTER, mode);
            }
        }

        //Apply YawPitchRoll to body
        private void pose()
        {
            //1° = 0,0174533 rad
            double degX = 0.174533 * y;
            double degY = 0.174533 * -x;
            double degZ = 0.174533 * z;
            double distA = 20 * a;
            double distB = 20 * b;
            //Set rotation around xyz axis and translation in ab-axis
            robot.pose(degZ, degY, degX, distA, distB);
        }

        #endregion Functions
    }
}
