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
        //Objects
        
        /** @brief   The input. */
        Gamepad input = null;

        /** @brief   The robot. */
        Hexapod robot = new Hexapod();

        /** @brief   The input task. */
        Task inputTask = null;
        //******

        //Fields
        
        /** @brief   The x coordinate of the input. */
        double x = 0;

        /** @brief   The y coordinate of the input. */
        double y = 0;

        /** @brief   The z coordinate of the input. */
        double z = 0;

        /** @brief   The rotational coordinate of the input. */
        double r = 0;

        /** @brief   A bool value. System turns off if true. */
        bool shutdown = false;

        /** @brief   The threshold for the input. */
        double threshold = 0.25;
        //******

        //Enums

        /**********************************************************************************************//**
         * @enum    directions
         *
         * @brief   Values that represent directions.
         **************************************************************************************************/

        public enum directions { CENTER, X, Y, ROTATE };
        //******

        //Properties

        /**********************************************************************************************//**
         * @property    public double X
         *
         * @brief   Gets the x coordinate of the input.
         *
         * @return  The x coordinate.
         **************************************************************************************************/

        public double X
        {
            get
            {
                return Math.Round(x, 2);
            }
        }

        /**********************************************************************************************//**
         * @property    public double Y
         *
         * @brief   Gets the y coordinate of the input.
         *
         * @return  The y coordinate.
         **************************************************************************************************/

        public double Y
        {
            get
            {
                return Math.Round(y, 2);
            }
        }

        /**********************************************************************************************//**
         * @property    public double Z
         *
         * @brief   Gets the z coordinate of the input.
         *
         * @return  The z coordinate.
         **************************************************************************************************/

        public double Z
        {
            get
            {
                return Math.Round(z, 2);
            }
        }

        /**********************************************************************************************//**
         * @property    public double R
         *
         * @brief   Gets the rotational coordinate of the input..
         *
         * @return  The rotational coordinate.
         **************************************************************************************************/

        public double R
        {
            get
            {
                return Math.Round(z, 2);
            }
        }
        //******

        //Functions

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
            while (true)
            {


                Task.Delay(0).Wait();

                
                if (input != null)
                {
                    GamepadReading gamepadStatus = input.GetCurrentReading();
                    x = gamepadStatus.LeftThumbstickY;
                    y = -gamepadStatus.LeftThumbstickX;
                    z = (gamepadStatus.LeftTrigger - gamepadStatus.RightTrigger);
                    r = gamepadStatus.RightThumbstickX;
                    shutdown = (gamepadStatus.Buttons == (GamepadButtons.A | GamepadButtons.B | GamepadButtons.X | GamepadButtons.Y | GamepadButtons.LeftThumbstick));

                }
                else
                {
                    if (Gamepad.Gamepads.Count() > 0)
                    {
                        input = Gamepad.Gamepads.First();
                    }
                }

                //check if the system should be turned off
                if (shutdown)
                {
                    Debug.WriteLine("_WARNING_: SYSTEM SHUTDOWN INITIALIZED!");
                    ShutdownManager.BeginShutdown(ShutdownKind.Shutdown, new TimeSpan(0));
                    
                    
                }

                //check if any of the axis is above the threshold
                if (Math.Abs(x) >= threshold || Math.Abs(y) >= threshold || Math.Abs(r) >= threshold)
                {
                    //if the x coordionate is bigger then all the other coordinates and bigger then the threshold move the robot in x-direction
                    if (Math.Abs(x) > Math.Abs(y) && Math.Abs(x) > Math.Abs(r) && Math.Abs(x) >= threshold)
                    {

                        robot.move(x, (byte)directions.X);
                    }
                    //if the y coordionate is bigger then all the other coordinates and bigger then the threshold move the robot in y-direction
                    else if (Math.Abs(y) > Math.Abs(x) && Math.Abs(y) > Math.Abs(r) && Math.Abs(y) >= threshold)
                    {

                        robot.move(y, (byte)directions.Y);
                    }
                    //if the r coordionate is bigger then all the other coordinates and bigger then the threshold turn the robot
                    else if (Math.Abs(r) > Math.Abs(x) && Math.Abs(r) > Math.Abs(y) && Math.Abs(r) >= threshold)
                    {

                        robot.move(r, (byte)directions.ROTATE);
                    }
                }
                //do nothing if none of the values is above the threshold
                else
                {
                    robot.move(0, (byte)directions.CENTER);
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
                Debug.WriteLine("Gamepad connected!");
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

        public double getGuiData(byte n, byte xyz)
        {
            return robot.getLegPos(n, xyz);

        }
        //******

    }
}
