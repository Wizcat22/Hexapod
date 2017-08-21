/**
 * @file    controller.cs.
 *
 * @brief   Implements the controller class
 */

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


    /**
     * @class   Controller
     *
     * @brief   A controller.
     *
     * @author  Alexander Miller
     * @date    13.08.2017
     */

    class Controller
    {
        #region Objects
        /** @brief   The input device (XBox360 Wireless Gamepad). */
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

        /** @brief   The timeframe in microseconds */
        const long timeframe = 25000;

        #endregion Fields

        #region Enums



        /**
         * @enum    directions
         *
         * @brief   Values that represent directions
         */

        public enum directions { POSE, XY, ROTATE, TURN };
        //******

        /**
         * @enum    modes
         *
         * @brief   Values that represent modes
         */

        public enum modes { DEFAULT, TERRAIN, BALANCE, FAST, SUPERFAST };

        #endregion Enums

        #region Properties

        #endregion Properties

        #region Functions


        /**
         * @fn  public void init()
         *
         * @brief   Initializes this object
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        public void init()
        {
            init_Gamepad();
            robot.init();

            inputTask = Task.Factory.StartNew(() => handleInputs());
        }



        /**
         * @fn  private void handleInputs()
         *
         * @brief   Inputhandler 
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        private void handleInputs()
        {
            //Stopwatch for fixed update
            Stopwatch time = Stopwatch.StartNew();
            //operating mode
            byte mode = 0;
            //movement
            byte direction = 0;


            while (true)
            {
                

                if (mode != (byte)modes.SUPERFAST)
                {
                    if (mode == (byte)modes.FAST)
                    {
                        while (((double)time.ElapsedTicks / Stopwatch.Frequency) * 1000000 < timeframe/3)
                        {

                        }
                    }
                    else
                    {
                        //wait until atleast "timeframe" seconds past -> necessary for PID-controller
                        while (((double)time.ElapsedTicks / Stopwatch.Frequency) * 1000000 < timeframe)
                        {

                        }
                    }
                }

                time = Stopwatch.StartNew();




                //if no input device was found
                if (input != null)
                {
                    //Get current Gamepadreading
                    GamepadReading gamepadStatus = input.GetCurrentReading();
                    //Set all input axis
                    x = -gamepadStatus.LeftThumbstickY;
                    y = gamepadStatus.LeftThumbstickX;
                    z = (gamepadStatus.LeftTrigger - gamepadStatus.RightTrigger);
                    a = gamepadStatus.RightThumbstickX;
                    b = gamepadStatus.RightThumbstickY;



                    //set action accodring to button combos

                    //Shutdown = A&B&X&Y&LThumb
                    if (gamepadStatus.Buttons == (GamepadButtons.A | GamepadButtons.B | GamepadButtons.X | GamepadButtons.Y | GamepadButtons.LeftThumbstick))
                    {
                        //Shutdown the system
                        shutdown();
                    }
                    //Super fast = Left Stick and Right Stick
                    else if (gamepadStatus.Buttons == (GamepadButtons.LeftThumbstick | GamepadButtons.RightThumbstick))
                    {
                        mode = (byte)modes.SUPERFAST;
                    }
                    //Fast = Right Stick
                    else if (gamepadStatus.Buttons == GamepadButtons.RightThumbstick)
                    {
                        mode = (byte)modes.FAST;
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
                //if no gamepad was found
                else
                {
                    //search for gamepad
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
                        pose(mode);
                        break;
                    default:
                        break;
                }



            }
        }


        /**
         * @fn  private void init_Gamepad()
         *
         * @brief   Initializes the gamepad for use as input device.
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

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



        /**
         * @fn  private void gamepadRemovedHandler(object sender, Gamepad e)
         *
         * @brief   Handler, called when the gamepad is removed
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   sender  Source of the event.
         * @param   e       A Gamepad to process.
         */

        private void gamepadRemovedHandler(object sender, Gamepad e)
        {
            input = null;
            Debug.WriteLine("Warning: Gamepad was removed.");
        }



        /**
         * @fn  private void gamepadAddedHandler(object sender, Gamepad e)
         *
         * @brief   Handler, called when the gamepad is added.
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   sender  Source of the event.
         * @param   e       A Gamepad to process.
         */

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



        /**
         * @fn  private void shutdown()
         *
         * @brief   Shuts down this system
         *
         * @author  Alex
         * @date    13.08.2017
         */

        private void shutdown()
        {
            Debug.WriteLine("WARNING: SYSTEM SHUTDOWN INITIALIZED!");

            robot.shutdown();

            ShutdownManager.BeginShutdown(ShutdownKind.Shutdown, new TimeSpan(0));
        }



        /**
         * @fn  private void walk(byte mode)
         *
         * @brief   Walks in the given mode
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   mode    The mode.
         */

        private void walk(byte mode)
        {
            //check if any of the axis is above the threshold
            if (Math.Abs(x) >= threshold || Math.Abs(y) >= threshold)
            {
                //if only x is above the threshold
                if (((Math.Abs(x) >= threshold) && !(Math.Abs(y) >= threshold)))
                {
                    //walk in x direction in the given mode

                    robot.walk(x, 0, mode);
                }
                //if only y is above threshold
                else if (((Math.Abs(y) >= threshold) && !(Math.Abs(x) >= threshold)))
                {
                    //walk in y direction in the given mode

                    robot.walk(0, y, mode);
                }
                else
                {
                    //walk in xy direction in the given mode

                    robot.walk(x, y, mode);
                }

            }
            else
            {
                //dont move
                robot.walk(0, 0, mode);
            }
        }

        /**
         * @fn  private void turn(byte mode)
         *
         * @brief   Turns in the given mode
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   mode    The mode.
         */

        private void turn(byte mode)
        {

            if (Math.Abs(x) >= threshold && !(Math.Abs(a) >= threshold))
            {
                //walk
                robot.turn(x, 0, mode);
            }
            else if (Math.Abs(x) >= threshold && (Math.Abs(a) >= threshold))
            {
                //turn 
                robot.turn(x, a, mode);
            }
            else
            {
                //dont move
                robot.turn(0, 0, mode);
            }

        }

        /**
         * @fn  private void rotate(byte mode)
         *
         * @brief   Rotates in the given mode
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   mode    The mode.
         */

        private void rotate(byte mode)
        {
            if (Math.Abs(y) >= threshold)
            {
                //rotate in given direction in the given mode
                robot.rotate(y, mode);
            }
            else
            {
                //dont move
                robot.rotate(0, mode);
            }

        }


        /**
         * @fn  private void pose()
         *
         * @brief   Poses the Hexapod
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        private void pose(byte mode)
        {
            //1° = 0,0174533 rad
            double degX = 0.174533 * y;
            double degY = 0.174533 * -x;
            double degZ = 0.174533 * z;
            double distA = 30 * b;
            double distB = 30 * -a;
            double distC = 0;
            //Set rotation around xyz axis and translation in ab-axis
            robot.pose(degZ, degY, degX, distA, distB, distC, mode);
        }

        #endregion Functions
    }
}
