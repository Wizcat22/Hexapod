/**********************************************************************************************//**
 * @file    hexapod.cs
 *
 * @brief   Implements the hexapod class.
 **************************************************************************************************/

using System;
using System.Collections.Generic;
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
        //Objects
        
        /** @brief   The accelerometer. */
        //Accelerometer accel = new Accelerometer();
        //******

        //Fields
        
        /** @brief   The last direction. */
        byte lastDirection = 0;

        /** @brief   Width of the body in mm. */
        const double bodyWidth = 200;

        /** @brief   The body heigth in mm. */
        const double bodyHeigth = 300;
        //******

        //Arrays

        /** @brief   The gait offsets. */
        int[] gait = { 25, 75, 75, 25, 25, 75 };

        /** @brief   The angles of the leg paths in rotation. */
        int[] rotation = { 135, 45, 180, 0, 225, 315 };
        
        /** @brief   The legs. */
        ILeg[] legs = new ILeg[6];
        //******


        //Functions

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

            legs[0] = new ILeg(gait[0], 8,-5,2, rotation[0], 0x11);
            legs[2] = new ILeg(gait[2], 3,0,5, rotation[2], 0x12);
            legs[4] = new ILeg(gait[4], 5, -3, 2, rotation[4], 0x13);


            legs[1] = new ILeg(gait[1], -2, -5, 0, rotation[1], 0x21);
            legs[3] = new ILeg(gait[3], -1, -3, 0, rotation[3], 0x22);
            legs[5] = new ILeg(gait[5], -8, 5, 0, rotation[5], 0x23);

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

        public void move(double inc_a,double inc_b, byte dir)
        {
            //If the direction has changed, center all les
            if (dir != lastDirection)
            {
                centerLegs();
            }

            foreach (ILeg l in legs)
            {
                switch (dir)
                {
                    case (byte)Controller.directions.XY:
                        l.calcPositionXY(inc_a,inc_b);

                        lastDirection = (byte)Controller.directions.XY;
                        break;
                    case (byte)Controller.directions.ROTATE:
                        l.calcPositionR(inc_a);
                        lastDirection = (byte)Controller.directions.ROTATE;
                        break;
                    case (byte)Controller.directions.CENTER:
                        //accel.read();
                        //calcOffsets();
                        //l.calcPositionZOffset();
                        lastDirection = (byte)Controller.directions.CENTER;
                        break;
                    default:
                        break;
                }

                l.calcData();
                l.sendData();

            }

        }

        /**********************************************************************************************//**
         * @fn  private void calcOffsets()
         *
         * @brief   Calculates the z offsets of the TCPs to level the body.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        //private void calcOffsets()
        //{
        //    double mul = 1;

        //    for (int i = 0; i < legs.Length; i++)
        //    {
        //        if (i % 2 == 0)
        //        {
        //            legs[i].Zoff += mul * Math.Sin(accel.angleYZ);
        //        }
        //        else
        //        {
        //            legs[i].Zoff -= mul * Math.Sin(accel.angleYZ);
        //        }

        //    }

        //    legs[0].Zoff -= mul * Math.Sin(accel.angleXZ);
        //    legs[1].Zoff -= mul * Math.Sin(accel.angleXZ);

        //    legs[4].Zoff += mul * Math.Sin(accel.angleXZ);
        //    legs[5].Zoff += mul * Math.Sin(accel.angleXZ);

        //}


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

            //clear all z offsets
            foreach (ILeg l in legs)
            {
                l.Zoff = 0;
                
                l.calcData();
                l.sendData();
                Task.Delay(time).Wait();
            }

            //put all legs down
            foreach (ILeg l in legs)
            {
                l.ZPos = 0;
                
                l.calcData();
                l.sendData();
                Task.Delay(time).Wait();
            }

            //center each leg
            foreach (ILeg l in legs)
            {
                //Up
                l.ZPos = l.StepSizeZ;
                
                l.calcData();
                l.sendData();
                Task.Delay(time).Wait();

                //Center
                l.calcPositionCenter();
               
                l.calcData();
                l.sendData();
                Task.Delay(time).Wait();

                //Down
                l.ZPos = 0;
                
                l.calcData();
                l.sendData();
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
        //******
    }
}
