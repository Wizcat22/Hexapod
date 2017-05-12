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
        //Objects

        /** @brief   The accelerometer. */
        Accelerometer accel = new Accelerometer();
        //******

        //Fields

        /** @brief   The last direction. */
        byte lastDirection = 100;

        double yaw = 0.0;
        double pitch = 0.0;
        double roll = 0.0;
        double pitchDiffSum = 0.0;
        double rollDiffSum = 0.0;

        /** @brief   Width of the body in mm. */
        const double bodyWidth = 200;

        /** @brief   The body heigth in mm. */
        const double bodyHeigth = 300;

        //******

        //Arrays



        
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
            //left
            //giat offset,cal a,cal b, cal c , deg offset, i2c address;


            legs[0] = new ILeg(25, 8, -5, 2, 135, 0x11, 150, 175);
            legs[2] = new ILeg(75, 3, 0, 5, 180, 0x12, 0, 175);
            legs[4] = new ILeg(25, 5, -3, 2, 225, 0x13, -150, 175);

            //right
            legs[1] = new ILeg(75, -2, -5, 0, 45, 0x21, 150, -175);
            legs[3] = new ILeg(25, -1, -3, 0, 0, 0x22, 0, -175);
            legs[5] = new ILeg(75, -8, 5, 0, 315, 0x23, -150, -175);

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

        public void move(double inc_a,double inc_b, byte dir,byte mode)
        {
            //If the direction has changed, center all les
            if (dir != lastDirection && lastDirection != (byte)Controller.directions.CENTER)
            {
                centerLegs();
            }

            if (mode == (byte)Controller.modes.BALANCE || mode == (byte)Controller.modes.ADAPTIVE)
            {
                accel.read();
                balance(accel.Pitch, accel.Roll);
            }

            

            foreach (ILeg l in legs)
            {
                switch (dir)
                {
                    case (byte)Controller.directions.XY:
                        l.calcPositionXY(inc_a,inc_b,mode);
                        l.calcPose(0, pitch, roll, 0, 0);
                        lastDirection = (byte)Controller.directions.XY;
                        break;
                    case (byte)Controller.directions.ROTATE:
                        l.calcPositionR(inc_a,mode);
                        lastDirection = (byte)Controller.directions.ROTATE;
                        break;
                    case (byte)Controller.directions.CENTER:

                        
                        //Debug.WriteLine("X= " + Math.Round((accel.Pitch * 180 / Math.PI),2) + "Y= " + Math.Round((accel.Roll * 180 / Math.PI), 2));
                        //Debug.WriteLine("Xa= " + Math.Round((pitch * 180 / Math.PI), 2) + "Ya= " + Math.Round((roll * 180 / Math.PI), 2));
                        l.calcPositionCenter();
                        l.calcPose(0, pitch, roll, 0,0);
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
            //If the direction has changed, center all les
            if (lastDirection != (byte)Controller.directions.CENTER)
            {
                centerLegs();
                lastDirection = (byte)Controller.directions.CENTER;
            }

            foreach (ILeg leg in legs)
            {
                leg.calcPositionCenter();
                leg.calcPose(yaw,pitch,roll,a,b);
                leg.calcData();
            }
        }

        
        private void balance(double newPitch, double newRoll)
        {
            double p = 0.1;
            double i = 0.0;

            pitch -= (pitch-newPitch)*p;
            roll -= (roll - newRoll) *p;

            double maxPitch = 10 * 0.0174533;
            double maxRoll = 10 * 0.0174533;

            if (pitch > maxPitch)
            {pitch = maxPitch;}
            else if (pitch<-maxPitch)
            {pitch = -maxPitch;}

            if (roll > maxRoll)
            { roll = maxRoll; }
            else if (roll < -maxRoll)
            { roll = -maxRoll; }


            //double[] buf = { pitch , -roll  };
            //Debug.WriteLine("pitch= " + (pitch*180/Math.PI) + " roll= " + (roll * 180 / Math.PI));
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

            //clear all z offsets
            foreach (ILeg l in legs)
            {
                
                
                l.calcData();
                Task.Delay(time).Wait();
            }

            //put all legs down
            foreach (ILeg l in legs)
            {
                l.ZPos = 0;
                
                l.calcData();
                Task.Delay(time).Wait();
            }

            //center each leg
            foreach (ILeg l in legs)
            {
                //Up
                l.ZPos = l.StepSizeZ;
                
                l.calcData();
                Task.Delay(time).Wait();

                //Center
                l.calcPositionCenter();
                l.ZPos = l.StepSizeZ;
                l.calcData();
                Task.Delay(time).Wait();

                //Down
                l.calcPositionCenter();

                l.calcData();
                Task.Delay(time).Wait();

            }
        }


        //******
    }
}
