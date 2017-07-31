/**********************************************************************************************//**
 * @file    hexapod.cs
 *
 * @brief   Implements the hexapod class.
 **************************************************************************************************/

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    /**********************************************************************************************//**
     * @class   Hexapod
     *
     * @brief   A hexapod.
     *
     * @author  Alexander Miller
     * @date    11.08.2016
     **************************************************************************************************/

    class Hexapod
    {
        #region Objects
        /** @brief   The accelerometer. */
        Accelerometer accel = new Accelerometer();
        #endregion Objects

        #region Fields
        /** @brief   The last direction. */
        byte lastDirection = 100;

        double pitch = 0.0;
        double roll = 0.0;

        const double k_p = 0.0;
        const double k_i = 1;
        const double k_d = 0.0;
        const double T = 0.032;

        const double k1 = (k_p + k_i * T + k_d / T);
        const double k2 = (-k_p - 2 * k_d / T);
        const double k3 = (k_d / T);

        const double maxPitch = 10 * 0.0174533;
        const double maxRoll = 10 * 0.0174533;




        /** @brief   Width of the body in mm. */
        const double bodyWidth = 200;

        /** @brief   The body heigth in mm. */
        const double bodyHeigth = 300;
        #endregion Fields

        #region Arrays
        /** @brief   The legs. */
        Leg[] legs = new Leg[6];

        double[] pitch_e = new double[3];
        double[] roll_e = new double[3];
        #endregion Arrays

        #region Functions
        /**********************************************************************************************//**
         * @fn  public void init()
         *
         * @brief   Initializes this device.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        public void init()
        {
            //accel.init();
            //init legs
            //left
            //giat offset,cal a,cal b, cal c , deg offset, i2c address;


            legs[0] = new Leg(25, 8, -5, 2, 135, 0x11, 150, 175);
            legs[2] = new Leg(75, 3, 0, 5, 180, 0x12, 0, 175);
            legs[4] = new Leg(25, 5, -3, 2, 225, 0x13, -150, 175);

            //right
            legs[1] = new Leg(75, -2, -5, 0, 45, 0x21, 150, -175);
            legs[3] = new Leg(25, -1, -3, 0, 0, 0x22, 0, -175);
            legs[5] = new Leg(75, -8, 5, 0, 315, 0x23, -150, -175);

        }

        /**********************************************************************************************//**
         * @fn  public void move(double inc, byte dir)
         *
         * @brief   Moves the robot.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         *
         * @param   inc Amount to increment by.
         * @param   dir The direction to move in.
         **************************************************************************************************/

        public void move(double inc_x, double inc_y, double inc_a, byte dir, byte mode)
        {
            //If the direction has changed, center all legs
            if (dir != lastDirection && lastDirection != (byte)Controller.directions.CENTER)
            {
                centerLegs();
            }

            if (mode == (byte)Controller.modes.BALANCE || mode == (byte)Controller.modes.ADAPTIVE)
            {
                accel.read();
                balance(accel.Pitch, accel.Roll);
            }



            foreach (Leg l in legs)
            {
                switch (dir)
                {
                    case (byte)Controller.directions.XY:
                        l.calcPositionWalk(inc_x, inc_y, mode);

                        if (mode == (byte)Controller.modes.BALANCE || mode == (byte)Controller.modes.ADAPTIVE)
                        {
                            l.calcPose(0, pitch, -roll, 0, 0);
                        }

                        lastDirection = (byte)Controller.directions.XY;
                        break;
                    case (byte)Controller.directions.ROTATE:
                        l.calcPositionRotate(inc_a, mode);
                        lastDirection = (byte)Controller.directions.ROTATE;
                        break;
                    case (byte)Controller.directions.TURN:
                        l.calcPositionTurn(inc_x, inc_y, inc_a);
                        lastDirection = (byte)Controller.directions.TURN;
                        break;
                    case (byte)Controller.directions.CENTER:
                        l.calcPositionCenter();
                        l.calcPose(0, pitch, -roll, 0, 0);
                        lastDirection = (byte)Controller.directions.CENTER;
                        break;
                    default:
                        break;
                }
                if (mode == (byte)Controller.modes.TERRAIN)
                {
                    l.calcDataTerrain();
                }
                else
                {
                    l.calcData();
                }

            }

        }

        public void pose(double yaw, double pitch, double roll, double a, double b)
        {
            //If the direction has changed, center all legs
            if (lastDirection != (byte)Controller.directions.CENTER)
            {
                centerLegs();
                lastDirection = (byte)Controller.directions.CENTER;
            }

            foreach (Leg leg in legs)
            {
                leg.calcPositionCenter();
                leg.calcPose(yaw, pitch, roll, a, b);
                leg.calcData();
            }
        }

        private void balance(double newPitch, double newRoll)
        {
            pitch_e[2] = pitch_e[1];
            pitch_e[1] = pitch_e[0];
            pitch_e[0] = newPitch - pitch;
            pitch = pitch + k1 * pitch_e[0] + k2 * pitch_e[1] + k3 * pitch_e[2];

            roll_e[2] = roll_e[1];
            roll_e[1] = roll_e[0];
            roll_e[0] = newRoll - roll;
            roll = roll + k1 * roll_e[0] + k2 * roll_e[1] + k3 * roll_e[2];

            if (pitch > maxPitch)
            { pitch = maxPitch; }
            else if (pitch < -maxPitch)
            { pitch = -maxPitch; }

            if (roll > maxRoll)
            { roll = maxRoll; }
            else if (roll < -maxRoll)
            { roll = -maxRoll; }
        }

        /**********************************************************************************************//**
         * @fn  private void centerLegs()
         *
         * @brief   Centers all legs.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/
        private void centerLegs()
        {
            //time between steps
            int time = 50;

            pitch = 0;
            roll = 0;

            foreach (Leg l in legs)
            {
                l.calcPose(0, pitch, roll, 0, 0);

                l.calcData();
                //Task.Delay(time).Wait();
            }


            //put all legs down
            foreach (Leg l in legs)
            {
                l.ZPos = 0;

                l.calcData();
                Task.Delay(time).Wait();
            }



            //center each leg
            foreach (Leg l in legs)
            {
                l.calcData();
                Task.Delay(time).Wait();
                //Up
                l.ZPos = (int)l.StepSizeZ;

                l.calcData();
                Task.Delay(time).Wait();

                //Center
                l.calcPositionCenter();
                l.ZPos = (int)l.StepSizeZ;
                l.calcData();
                Task.Delay(time).Wait();

                //Down
                l.calcPositionCenter();

                l.calcData();
                Task.Delay(time).Wait();

            }
        }
        #endregion Functions
    }
}
