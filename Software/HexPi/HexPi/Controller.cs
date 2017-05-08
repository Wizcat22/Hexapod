﻿using System;
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

        int mode = 0;

        /** @brief   The threshold for the input. */
        double threshold = 0.25;
        //******

        //Enums

        /**********************************************************************************************//**
         * @enum    directions
         *
         * @brief   Values that represent directions.
         **************************************************************************************************/

        public enum directions { CENTER, XY, ROTATE };
        //******

        public enum modes {WALK,POSE,SHUTDOWN, TERRAIN };

        //Properties


        

        

        

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

                
                //Task.Delay(0).Wait();
                Task.Delay(0).Wait();
                
                if (input != null)
                {
                    GamepadReading gamepadStatus = input.GetCurrentReading();
                    x = gamepadStatus.LeftThumbstickY;
                    y = -gamepadStatus.LeftThumbstickX;
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
                    case (int)modes.WALK:
                        walk(false);
                        break;
                    case (int)modes.TERRAIN:
                        walk(true);
                        break;
                    case (int)modes.POSE:
                        pose();
                        break;
                    case (int)modes.SHUTDOWN:
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

        //******

        //Initialize shutdown
        private void shutdown()
        {
            Debug.WriteLine("_WARNING_: SYSTEM SHUTDOWN INITIALIZED!");
            ShutdownManager.BeginShutdown(ShutdownKind.Shutdown, new TimeSpan(0));
        }

        // Walk in xy direction / turn in r direction
        private void walk(bool terrainmode)
        {
            //check if any of the axis is above the threshold
            if (Math.Abs(x) >= threshold || Math.Abs(y) >= threshold || Math.Abs(a) >= threshold)
            {
                //if the x coordionate is bigger then all the other coordinates and bigger then the threshold move the robot in x-direction
                if ((Math.Abs(x) > Math.Abs(a) && Math.Abs(x) >= threshold) || (Math.Abs(y) > Math.Abs(a) && Math.Abs(y) >= threshold))
                {

                    robot.move(x, y, (byte)directions.XY, terrainmode);
                }
                //if the r coordionate is bigger then all the other coordinates and bigger then the threshold turn the robot
                else if (Math.Abs(a) > Math.Abs(x) && Math.Abs(a) > Math.Abs(y) && Math.Abs(a) >= threshold)
                {

                    robot.move(a, 0, (byte)directions.ROTATE, terrainmode);
                }
            }
            //do nothing if none of the values is above the threshold
            else
            {
                robot.move(0, 0, (byte)directions.CENTER, terrainmode);
            }
        }

        //Apply YawPitchRoll to body
        private void pose()
        {
            //1° = 0,0174533 rad
            double degX = 0.174533 * x;
            double degY = 0.174533 * y;
            double degZ = 0.174533 * z;
            double distA = 50 * a;
            double distB = 50 * b;
            //Set rotation around xyz axis and translation in ab-axis
            robot.pose(degX,degY,degZ,distA,distB);
        }
    }
}
