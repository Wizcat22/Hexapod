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

        public enum directions { POSE, XY, ROTATE, TURN };
        //******

        public enum modes { DEFAULT,TERRAIN, BALANCE };

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
            byte direction = 0;

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
                        shutdown();
                    }
                    //Pose = RightShoulder
                    else if (gamepadStatus.Buttons == GamepadButtons.RightShoulder)
                    {
                        mode = (byte)modes.BALANCE;
                    }
                    //Terrain = LeftShoulder
                    else if (gamepadStatus.Buttons == GamepadButtons.LeftShoulder)
                    {
                        mode = (byte)modes.TERRAIN;
                    }
                    else
                    {
                        mode = (byte)modes.DEFAULT;
                    }

                    
                    if (gamepadStatus.Buttons == GamepadButtons.DPadUp)
                    {
                        direction = (byte)directions.XY;
                    }
                    else if (gamepadStatus.Buttons == GamepadButtons.DPadLeft)
                    {
                        direction = (byte)directions.POSE;
                    }
                    else if (gamepadStatus.Buttons == GamepadButtons.DPadDown)
                    {
                        direction = (byte)directions.ROTATE;
                    }
                    else if (gamepadStatus.Buttons == GamepadButtons.DPadRight)
                    {
                        direction = (byte)directions.TURN;
                    }

                }
                else
                {
                    if (Gamepad.Gamepads.Count() > 0)
                    {
                        input = Gamepad.Gamepads.First();
                    }
                }

                switch (direction)
                {
                    case (byte)directions.XY:
                        walk(mode);
                        break;
                    case (byte)directions.ROTATE:
                        rotate(mode);
                        break;
                    case (byte)directions.TURN:
                        turn(mode);
                        break;
                    case (byte)directions.POSE:
                        pose();
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
            if (Math.Abs(x) >= threshold || Math.Abs(y) >= threshold)
            {
                //if the xy coordionate is bigger then all the other coordinates and bigger then the threshold move the robot in xy-direction
                //if ((Math.Abs(x) > Math.Abs(a) && Math.Abs(x) >= threshold) || (Math.Abs(y) > Math.Abs(a) && Math.Abs(y) >= threshold))
                if (((Math.Abs(x) >= threshold) && !(Math.Abs(y) >= threshold)) )
                {
                    robot.walk(x, 0, mode);
                }
                if (((Math.Abs(y) >= threshold) && !(Math.Abs(x) >= threshold)))
                {
                    robot.walk(0, y, mode);
                }
                else
                {
                    robot.walk(x, y,mode);
                }

            }
            else
            {
                robot.walk(0, 0, mode);
            }
        }

        private void turn(byte mode)
        {
            if (Math.Abs(x) >= threshold && !(Math.Abs(a) >= threshold))
            {
                robot.turn(x, 0, mode);
            }
            else if (Math.Abs(x) >= threshold && (Math.Abs(a) >= threshold))
            {
                robot.turn(x, a, mode);
            }
            else
            {
                robot.turn(0, 0, mode);
            }

        }

        private void rotate(byte mode)
        {
            if (Math.Abs(y) >= threshold)
            {
                robot.rotate(y, mode);
            }
            else
            {
                robot.rotate(0, mode);
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
            double distC = 0;
            //Set rotation around xyz axis and translation in ab-axis
            robot.pose(degZ, degY, degX, distA, distB, distC);
        }

        #endregion Functions
    }
}
