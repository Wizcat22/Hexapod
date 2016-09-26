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
        
        /** @brief   The servo hat. */
        ServoHat servo = new ServoHat();

        /** @brief   The accelerometer. */
        Accelerometer accel = new Accelerometer();
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
        
        /** @brief   The servo data. */
        byte[] data = new byte[26];

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
            servo.init();
            accel.init();

            //init data array
            for (int i = 0; i < data.Length; i++)
            {
                data[i] = 187;
            }
            //set start byte 
            data[0] = 22;
            //set end byte
            data[25] = 11;

            //init legs

            legs[0] = new LeftLeg(gait[0], 5, -6, 4, rotation[0]);
            legs[2] = new LeftLeg(gait[2], 5, -3, 4, rotation[2]);
            legs[4] = new LeftLeg(gait[4], 0, -4, 10, rotation[4]);


            legs[1] = new RightLeg(gait[1], -5, -5, 5, rotation[1]);
            legs[3] = new RightLeg(gait[3], 0, -2, -5, rotation[3]);
            legs[5] = new RightLeg(gait[5], 0, 0, -3, rotation[5]);



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

        public void move(double inc, byte dir)
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
                    case (byte)Controller.directions.X:
                        l.calcPositionX(inc);
                        lastDirection = (byte)Controller.directions.X;
                        break;
                    case (byte)Controller.directions.Y:
                        l.calcPositionY(inc);
                        lastDirection = (byte)Controller.directions.Y;
                        break;
                    case (byte)Controller.directions.ROTATE:
                        l.calcPositionR(inc);
                        lastDirection = (byte)Controller.directions.ROTATE;
                        break;
                    case (byte)Controller.directions.CENTER:
                        accel.read();
                        calcOffsets();
                        l.calcPositionZ(true);
                        lastDirection = (byte)Controller.directions.CENTER;
                        break;
                    default:
                        break;
                }

                l.inverseKinematics();
                l.calcData();

            }

            setData();
            servo.write(data);

        }

        /**********************************************************************************************//**
         * @fn  private void calcOffsets()
         *
         * @brief   Calculates the z offsets of the TCPs to level the body.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        private void calcOffsets()
        {
            double mul = 1;

            for (int i = 0; i < legs.Length; i++)
            {
                if (i % 2 == 0)
                {
                    legs[i].Zoff += mul * Math.Sin(accel.angleYZ);
                }
                else
                {
                    legs[i].Zoff -= mul * Math.Sin(accel.angleYZ);
                }

            }

            legs[0].Zoff -= mul * Math.Sin(accel.angleXZ);
            legs[1].Zoff -= mul * Math.Sin(accel.angleXZ);

            legs[4].Zoff += mul * Math.Sin(accel.angleXZ);
            legs[5].Zoff += mul * Math.Sin(accel.angleXZ);

        }

        /**********************************************************************************************//**
         * @fn  private void setData()
         *
         * @brief   Fills the data array for the servos.
         *
         * @author  Alexander Miller
         * @date    11.08.2016
         **************************************************************************************************/

        private void setData()
        {

            data[19] = legs[0].getMotorData(2);
            data[20] = legs[0].getMotorData(1);
            data[21] = legs[0].getMotorData(0);

            //Debug.WriteLine("Leg 0 : " + data[1] + " " + data[2] + " " + data[3]);

            data[4] = legs[1].getMotorData(2);
            data[5] = legs[1].getMotorData(1);
            data[6] = legs[1].getMotorData(0);

            //Debug.WriteLine("Leg 1 : " + data[4] + " " + data[5] + " " + data[6]);

            data[16] = legs[2].getMotorData(2);
            data[17] = legs[2].getMotorData(1);
            data[18] = legs[2].getMotorData(0);

            //Debug.WriteLine("Leg 2 : " + data[7] + " " + data[8] + " " + data[9]);

            data[7] = legs[3].getMotorData(2);
            data[8] = legs[3].getMotorData(1);
            data[9] = legs[3].getMotorData(0);

            //Debug.WriteLine("Leg 3 : " + data[7] + " " + data[8] + " " + data[9]);

            data[13] = legs[4].getMotorData(2);
            data[14] = legs[4].getMotorData(1);
            data[15] = legs[4].getMotorData(0);

            //Debug.WriteLine("Leg 4 : " + data[19] + " " + data[20] + " " + data[21]);

            data[10] = legs[5].getMotorData(2);
            data[11] = legs[5].getMotorData(1);
            data[12] = legs[5].getMotorData(0);

            //Debug.WriteLine("Leg 5 : " + data[10] + " " + data[11] + " " + data[12]);
        }

        /**********************************************************************************************//**
         * @fn  private void demo()
         *
         * @brief   Initialises some special movements for demonstrations.
         *
         * @author  Alexander Miller
         * @date    26.09.2016
         **************************************************************************************************/
        public void demo()
        {
            throw new NotImplementedException();

            //time between steps
            int time = 30;

            centerLegs();
            Task.Delay(time).Wait();


            //MOVE UP AND DOWN
            //Move down
            for (int i = 0; i < 30; i++)
            {
                foreach (ILeg l in legs)
                {
                    l.ZPos = i;
                }
                demohelper();
                Task.Delay(time).Wait();
            }

            for (int i = 0; i < 10; i++)
            {
                //Move up
                for (int j = 30; j > -30; j--)
                {
                    foreach (ILeg l in legs)
                    {
                        l.ZPos = j;
                    }
                    demohelper();
                    Task.Delay(time).Wait();
                }
                //Move down
                for (int k = -30; k < 30; k++)
                {
                    foreach (ILeg l in legs)
                    {
                        l.ZPos = k;
                    }
                    demohelper();
                    Task.Delay(time).Wait();
                }
            }
            //--------------
            centerLegs();
            //MOVE BACK AND FORWARD
            //Move forward
            for (int i = 0; i < 30; i++)
            {
                foreach (ILeg l in legs)
                {
                    l.XPos = i;
                }
                demohelper();
                Task.Delay(time).Wait();
            }

            for (int i = 0; i < 10; i++)
            {
                //Move forward
                for (int j = 30; j > -30; j--)
                {
                    foreach (ILeg l in legs)
                    {
                        l.XPos = j;
                    }
                    demohelper();
                    Task.Delay(time).Wait();
                }
                //Move back
                for (int k = -30; k < 30; k++)
                {
                    foreach (ILeg l in legs)
                    {
                        l.XPos = k;
                    }
                    demohelper();
                    Task.Delay(time).Wait();
                }
            }
            //--------------
            centerLegs();
            //MOVE LEFT AND RIGHT
            //Move LEFT
            for (int i = 0; i < 30; i++)
            {
                foreach (ILeg l in legs)
                {
                    l.YPos = i;
                }
                demohelper();
                Task.Delay(time).Wait();
            }

            for (int i = 0; i < 10; i++)
            {
                //Move right
                for (int j = 30; j > -30; j--)
                {
                    foreach (ILeg l in legs)
                    {
                        l.YPos = j;
                    }
                    demohelper();
                    Task.Delay(time).Wait();
                }
                //Move left
                for (int k = -30; k < 30; k++)
                {
                    foreach (ILeg l in legs)
                    {
                        l.YPos = k;
                    }
                    demohelper();
                    Task.Delay(time).Wait();
                }
            }
            //--------------
            centerLegs();

            legs[0].TOffset = 0;
            legs[1].TOffset = 0;
            legs[2].TOffset = 0;
            legs[3].TOffset = 0;
            legs[4].TOffset = 0;
            legs[5].TOffset = 0;

            centerLegs();


            //MOVE X AND Z
            for (int i = 0; i < 10; i++)
            {
                for (int k = 0; k < 100; k++)
                {
                    foreach (ILeg l in legs)
                    {
                        l.calcPositionX(1);
                    }
                    demohelper();
                }
            }
            //--------------
            centerLegs();

            //MOVE Y AND Z
            for (int i = 0; i < 10; i++)
            {
                for (int k = 0; k < 100; k++)
                {
                    foreach (ILeg l in legs)
                    {
                        l.calcPositionY(1);
                    }
                    demohelper();
                }
            }
            //--------------
            centerLegs();

            //MOVE X AND Y
            for (int i = 0; i < 10; i++)
            {
                for (int k = 0; k < 100; k++)
                {
                    foreach (ILeg l in legs)
                    {
                        l.calcPositionX(1);
                        l.calcPositionY(0);
                        l.ZPos = 0;
                    }
                    demohelper();
                }
            }
            //--------------
            centerLegs();

            //MOVE X AND Y AND Z
            for (int i = 0; i < 10; i++)
            {
                for (int k = 0; k < 100; k++)
                {
                    foreach (ILeg l in legs)
                    {
                        l.calcPositionX(1);
                        l.calcPositionY(0);
                        l.ZPos = 0;
                    }
                    demohelper();
                }
            }
            //--------------
            centerLegs();

            legs[0].TOffset = gait[0];
            legs[1].TOffset = gait[1];
            legs[2].TOffset = gait[2];
            legs[3].TOffset = gait[3];
            legs[4].TOffset = gait[4];
            legs[5].TOffset = gait[5];
            centerLegs();

            //ROTATE
            for (int i = 0; i < 10; i++)
            {
                for (int k = 0; k < 100; k++)
                {
                    foreach (ILeg l in legs)
                    {
                        l.calcPositionR(1);
                    }
                    demohelper();
                }
            }
            //--------------
            centerLegs();

            //for (int i = 0; i < 30; i++)
            //{
            //    foreach (ILeg l in legs)
            //    {

            //    }
            //}

            //END OF DEMO
            legs[0].TOffset = gait[0];
            legs[1].TOffset = gait[1];
            legs[2].TOffset = gait[2];
            legs[3].TOffset = gait[3];
            legs[4].TOffset = gait[4];
            legs[5].TOffset = gait[5];
            centerLegs();
        }

        /**********************************************************************************************//**
        * @fn  private void demohelper()
        *
        * @brief   This function is used to minimize code in demomode().
        *
        * @author  Alexander Miller
        * @date    26.09.2016
        **************************************************************************************************/
        private void demohelper()
        {
            foreach (ILeg l in legs)
            {
                l.inverseKinematics();
                l.calcData();
                setData();
                servo.write(data);
            }
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
            int time = 30;

            //clear all z offsets
            foreach (ILeg l in legs)
            {
                l.Zoff = 0;
                l.inverseKinematics();
                l.calcData();
                setData();
                servo.write(data);
                Task.Delay(time).Wait();
            }

            //put all legs down
            foreach (ILeg l in legs)
            {
                l.ZPos = 0;
                l.inverseKinematics();
                l.calcData();
            }

            setData();
            servo.write(data);
            Task.Delay(time).Wait();

            //center each leg
            foreach (ILeg l in legs)
            {
                //Up
                l.ZPos = l.StepSizeZ;
                l.inverseKinematics();
                l.calcData();
                setData();
                servo.write(data);
                Task.Delay(time).Wait();

                //Center
                l.calcPositionCenter();
                l.inverseKinematics();
                l.calcData();
                setData();
                servo.write(data);
                Task.Delay(time).Wait();

                //Down
                l.ZPos = 0;
                l.inverseKinematics();
                l.calcData();
                setData();
                servo.write(data);
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
