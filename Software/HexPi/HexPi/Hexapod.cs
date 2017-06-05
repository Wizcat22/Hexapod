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

        public void move(double inc_x, double inc_y, double inc_a, byte dir,byte mode)
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
                        l.calcPositionXY(inc_x,inc_y,mode);

                        if (mode == (byte)Controller.modes.BALANCE || mode == (byte)Controller.modes.ADAPTIVE)
                        {
                            l.calcPose(0, pitch, -roll, 0, 0);
                        }
                        
                        lastDirection = (byte)Controller.directions.XY;
                        break;
                    case (byte)Controller.directions.ROTATE:
                        l.calcPositionR(inc_a,mode);
                        lastDirection = (byte)Controller.directions.ROTATE;
                        break;
                    case (byte)Controller.directions.TURN:
                        l.calcPositionTurn(inc_x,inc_y,inc_a);
                        lastDirection = (byte)Controller.directions.TURN;
                        break;
                    case (byte)Controller.directions.CENTER:

                        
                        //Debug.WriteLine("X= " + Math.Round((accel.Pitch * 180 / Math.PI),2) + "Y= " + Math.Round((accel.Roll * 180 / Math.PI), 2));
                        //Debug.WriteLine("Xa= " + Math.Round((pitch * 180 / Math.PI), 2) + "Ya= " + Math.Round((roll * 180 / Math.PI), 2));
                        l.calcPositionCenter();
                        l.calcPose(0, pitch, -roll, 0,0);
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

            //if (mode == (byte)Controller.modes.TERRAIN)
            //{
            //    int a = 0;
            //    int sum = 0;
            //    foreach (ILeg l in legs)
            //    {
            //        if (l.ZPos == 0)
            //        {
            //            a++;
            //            sum += l.readData();
            //        }
            //    }
            //    sum = sum/a;

            //    foreach (ILeg l in legs)
            //    {
            //        if (l.ZPos == 0)
            //        {
            //            l.ZPos = l.readData()-sum;
            //            l.calcData();
            //            l.ZPos = 0;
            //        }
                    
            //    }

            //}


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

        double k_p = 0.0;
        double k_i = 1;
        double k_d = 0.0;
        double T = 0.032;
        double[] pitch_e = new double[3];
        double pitch1 = 0;
        double[] roll_e = new double[3];
        double roll1 = 0;

        private void balance(double newPitch, double newRoll)
        {
            pitch1 = pitch;
            pitch_e[2] = pitch_e[1];
            pitch_e[1] = pitch_e[0];
            pitch_e[0] = newPitch - pitch;
            pitch = pitch1 + (k_p + k_i * T + k_d / T) * pitch_e[0] + (-k_p - 2 * k_d / T) * pitch_e[1] + (k_d / T) * pitch_e[2];

            roll1 = roll;
            roll_e[2] = roll_e[1];
            roll_e[1] = roll_e[0];
            roll_e[0] = newRoll - roll;
            roll = roll1 + (k_p + k_i * T + k_d / T) * roll_e[0] + (-k_p - 2 * k_d / T) * roll_e[1] + (k_d / T) * roll_e[2];
            
            //pitch -= (pitch-newPitch)*p;
            //roll += (roll - newRoll) *p;

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

            pitch = 0;
            roll = 0;

            foreach (ILeg l in legs)
            {
                l.calcPose(0, pitch, roll, 0, 0);

                l.calcData();
                //Task.Delay(time).Wait();
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


        //******
    }
}
