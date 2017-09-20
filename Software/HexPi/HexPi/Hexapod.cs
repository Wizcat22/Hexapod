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

    /**
     * @class   Hexapod
     *
     * @brief   A hexapod.
     *
     * @author  Alexander Miller
     * @date    13.08.2017
     */

    class Hexapod
    {
        #region Objects
        /** @brief   The accelerometer. */
        Accelerometer accel = new Accelerometer();
        #endregion Objects

        #region Fields
        /** @brief   The last direction. */
        byte lastDirection = 100;

        /** @brief   The pitch of the robot */
        double pitch = 0.0;
        /** @brief   The roll of the robot */
        double roll = 0.0;

        /** @brief   The p constant of the pid-controller */
        const double k_p = 0.01;
        /** @brief   The i constant of the pid-controller */
        const double k_i = 1;
        /** @brief   The d constant of the pid-controller */
        const double k_d = 0.0000;
        /** @brief   The repetition rate of the pid-controller in seconds */
        const double T = 0.025;

        /** @brief   The first combined constant of the pid-controller */
        const double k1 = (k_p + k_i * T + k_d / T);
        /** @brief   The second combined constant of the pid-controller */
        const double k2 = (-k_p - 2 * k_d / T);
        /** @brief   The third combined constant of the pid-controller */
        const double k3 = (k_d / T);

        /** @brief   The maximum pitch */
        const double maxPitch = 10 * 0.0174533;
        /** @brief   The maximum roll */
        const double maxRoll = 10 * 0.0174533;




        /** @brief   Width of the body in mm. */
        const double bodyWidth = 200;

        /** @brief   The body heigth in mm. */
        const double bodyHeigth = 300;
        #endregion Fields

        #region Arrays
        /** @brief   The legs. */
        Leg[] legs = new Leg[6];

        /** @brief   The last 3 pitch differences */
        double[] pitch_e = new double[3];
        /** @brief   The last 3 roll differences */
        double[] roll_e = new double[3];
        #endregion Arrays

        #region Functions


        /**
         * @fn  public void init()
         *
         * @brief   Initializes this device
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        public void init()
        {
            /* Leg positions
             *     ___
             * 0 -|   |- 1
             * 2 -|   |- 3
             * 4 -|___|- 5
             */

            //giat offset,cal alpha,cal beta, cal gamma , rotation offset, i2c address, x offset, y offset;

            //left
            legs[0] = new Leg(25, 10, -5, 2, 135, 0x11, 150, 175);
            legs[2] = new Leg(75, 5, -2, 7, 180, 0x12, 0, 175);
            legs[4] = new Leg(25, 3, -4, 3, 225, 0x13, -150, 175);

            //right
            legs[1] = new Leg(75, 0, -7, 0, 45, 0x21, 150, -175);
            legs[3] = new Leg(25, 0, 0, -3, 0, 0x22, 0, -175);
            legs[5] = new Leg(75, -7, 6, 3, 315, 0x23, -150, -175);




        }



        /**
         * @fn  public void walk(double inc_x, double inc_y, byte mode)
         *
         * @brief   Walks
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   inc_x   The increment of the x coordinate.
         * @param   inc_y   The increment of the y coordinate.
         * @param   mode    The mode.
         */

        public void walk(double inc_x, double inc_y, byte mode)
        {

            //If the direction has changed, center all legs
            if (lastDirection != (byte)Controller.directions.XY)
            {
                centerLegs();

                foreach (Leg l in legs)
                {
                    l.setColor(60);
                }

            }


            if (mode == (byte)Controller.modes.BALANCE)
            {
                //get accelerometer data
                accel.read();
                //change pitch and roll based on accelerometer data
                balance(accel.Pitch, accel.Roll);
            }
            else
            {
                //reset pitch and roll
                pitch = 0;
                roll = 0;
            }

            //calculate the tcp coordinate for each leg and adapt the pose
            foreach (Leg l in legs)
            {
                l.calcPositionWalk(inc_x, inc_y, mode);

                if (mode == (byte)Controller.modes.BALANCE)
                {
                    l.calcPose(0, pitch, -roll, 0, 0, 0);
                }
            }


            //send the tcp coordinates to each leg
            foreach (Leg l in legs)
            {
                if (mode == (byte)Controller.modes.TERRAIN)
                {
                    l.calcDataTerrain();
                }
                else
                {
                    l.calcData();
                }
            }


            //adapt body heigth in terrain mode
            if (mode == (byte)Controller.modes.TERRAIN)

            {
                int a = 0; //number of legs
                int sum = 0; //sum of all z-psoitions
                foreach (Leg l in legs)
                {
                    //if leg is grounded
                    if (l.ZPos == 0)
                    {
                        a++;
                        sum += l.readLegHeight();
                    }
                }
                //get average heigth difference
                sum = sum / a;

                //adapt tcp height
                foreach (Leg l in legs)
                {
                    if (l.ZPos == 0)
                    {
                        l.ZPos = l.readLegHeight() - sum;
                        l.calcData();
                        l.ZPos = 0;
                    }
                }
            }





            lastDirection = (byte)Controller.directions.XY;

        }

        /**
         * @fn  public void turn(double inc_x, double inc_r, byte mode)
         *
         * @brief   Turns around a given point on the y axis
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   inc_x   The increment of the movement.
         * @param   inc_r   Faktor of the radius.
         * @param   mode    The mode.
         */

        public void turn(double inc_x, double inc_r, byte mode)
        {
            //If the direction has changed, center all legs
            if (lastDirection != (byte)Controller.directions.TURN)
            {
                centerLegs();

                foreach (Leg l in legs)
                {
                    l.setColor(235);
                }

            }

            if (mode == (byte)Controller.modes.BALANCE)
            {
                //get accelerometer data
                accel.read();
                //change pitch and roll based on accelerometer data
                balance(accel.Pitch, accel.Roll);
            }
            else
            {
                //reset pitch and roll
                pitch = 0;
                roll = 0;
            }

            //calculate the tcp coordinate for each leg and adapt the pose
            foreach (Leg l in legs)
            {
                l.calcPositionTurn(inc_x, inc_r, mode);

                if (mode == (byte)Controller.modes.BALANCE)
                {
                    l.calcPose(0, pitch, -roll, 0, 0, 0);
                }
            }

            //send the tcp coordinates to each leg
            foreach (Leg l in legs)
            {
                if (mode == (byte)Controller.modes.TERRAIN)
                {
                    l.calcDataTerrain();
                }
                else
                {
                    l.calcData();
                }
            }

            //adapt body heigth in terrain mode
            if (mode == (byte)Controller.modes.TERRAIN)

            {
                int a = 0; //number of legs
                int sum = 0; //sum of all z-psoitions
                foreach (Leg l in legs)
                {
                    //if leg is grounded
                    if (l.ZPos == 0)
                    {
                        a++;
                        sum += l.readLegHeight();
                    }
                }
                //get average heigth difference
                sum = sum / a;

                //adapt tcp height
                foreach (Leg l in legs)
                {
                    if (l.ZPos == 0)
                    {
                        l.ZPos = l.readLegHeight() - sum;
                        l.calcData();
                        l.ZPos = 0;
                    }
                }
            }

            lastDirection = (byte)Controller.directions.TURN;

        }

        /**
         * @fn  public void rotate(double inc_r, byte mode)
         *
         * @brief   Rotates around the center of the body
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   inc_r   The increment of the movement.
         * @param   mode    The mode.
         */

        public void rotate(double inc_r, byte mode)
        {
            //If the direction has changed, center all legs
            if (lastDirection != (byte)Controller.directions.ROTATE)
            {
                centerLegs();

                foreach (Leg l in legs)
                {
                    l.setColor(180);
                }

            }

            if (mode == (byte)Controller.modes.BALANCE)
            {
                //get accelerometer data
                accel.read();
                //change pitch and roll based on accelerometer data
                balance(accel.Pitch, accel.Roll);
            }
            else
            {
                //reset pitch and roll
                pitch = 0;
                roll = 0;
            }

            //calculate the tcp coordinate for each leg and adapt the pose
            foreach (Leg l in legs)
            {
                l.calcPositionRotate(inc_r, mode);

                if (mode == (byte)Controller.modes.BALANCE)
                {
                    l.calcPose(0, pitch, -roll, 0, 0, 0);
                }
            }

            //send the tcp coordinates to each leg
            foreach (Leg l in legs)
            {
                if (mode == (byte)Controller.modes.TERRAIN)
                {
                    l.calcDataTerrain();
                }
                else
                {
                    l.calcData();
                }
            }

            //adapt body heigth in terrain mode
            if (mode == (byte)Controller.modes.TERRAIN)

            {
                int a = 0; //number of legs
                int sum = 0; //sum of all z-psoitions
                foreach (Leg l in legs)
                {
                    //if leg is grounded
                    if (l.ZPos == 0)
                    {
                        a++;
                        sum += l.readLegHeight();
                    }
                }
                //get average heigth difference
                sum = sum / a;

                //adapt tcp height
                foreach (Leg l in legs)
                {
                    if (l.ZPos == 0)
                    {
                        l.ZPos = l.readLegHeight() - sum;
                        l.calcData();
                        l.ZPos = 0;
                    }
                }
            }


            lastDirection = (byte)Controller.directions.ROTATE;
        }



        double t = 0;
        double move = 0;


        public void dance(double inc)
        {

            double yaw = 0.0;
            double pitch = 0.0;
            double roll = 0.0;
            double a = 0.0;
            double b = 0.0;
            double c = 0.0;


            //If the direction has changed, center all legs
            if (lastDirection != (byte)Controller.directions.DANCE)
            {
                centerLegs();

                foreach (Leg l in legs)
                {
                    l.setColor(0);
                }
                lastDirection = (byte)Controller.directions.DANCE;
                move = 0;
                t = 0;

            }

            t += Math.Abs(inc);

            switch (move)
            {
                case 0:
                    //DANCE!
                    roll = 0.174533 * Math.Sin(t);
                    //
                    if (t>2*Math.PI)
                    {
                        t = 0.0;
                        move++;
                    }
                    break;
                case 1:
                    //DANCE!
                    pitch = 0.174533 * Math.Sin(t);
                    //
                    if (t > 2 * Math.PI)
                    {
                        t = 0.0;
                        move++;
                    }
                    break;
                case 2:
                    //DANCE!
                    yaw = 0.174533 * Math.Sin(t);
                    //
                    if (t > 2 * Math.PI)
                    {
                        t = 0.0;
                        move++;
                    }
                    break;
                case 3:
                    //DANCE!
                    roll = 0.174533 * Math.Sin(t);
                    pitch = 0.174533 * Math.Sin(t);
                    //
                    if (t > 2 * Math.PI)
                    {
                        t = 0.0;
                        move++;
                    }
                    break;
                case 4:
                    //DANCE!
                    roll = 0.174533 * Math.Sin(t);
                    yaw = 0.174533 * Math.Sin(t);
                    //
                    if (t > 2 * Math.PI)
                    {
                        t = 0.0;
                        move++;
                    }
                    break;
                case 5:
                    //DANCE!
                    yaw = 0.174533 * Math.Sin(t);
                    pitch = 0.174533 * Math.Sin(t);
                    //
                    if (t > 2 * Math.PI)
                    {
                        t = 0.0;
                        move++;
                    }
                    break;
                case 6:
                    //DANCE!
                    a = 30 * Math.Sin(t);
                    //
                    if (t > 2 * Math.PI)
                    {
                        t = 0.0;
                        move++;
                    }
                    break;
                case 7:
                    //DANCE!
                    b = 30 * Math.Sin(t);
                    //
                    if (t > 2 * Math.PI)
                    {
                        t = 0.0;
                        move++;
                    }
                    break;
                case 8:
                    //DANCE!
                    c = 30 * Math.Sin(t);
                    //
                    if (t > 2 * Math.PI)
                    {
                        t = 0.0;
                        move++;
                    }
                    break;
                case 9:
                    //DANCE!
                    roll = 0.174533 * Math.Sin(t);
                    b = 30 * Math.Sin(t);
                    c = 30 * Math.Sin(t);
                    //
                    if (t > 2 * Math.PI)
                    {
                        t = 0.0;
                        move++;
                    }
                    break;
                default:
                    move = 0;
                    break;
            }

            foreach (Leg leg in legs)
            {
                leg.calcPositionCenter();
                leg.calcPose(yaw, pitch, roll, a, b, c);

            }

            //send tcp positions
            foreach (Leg leg in legs)
            {
                leg.calcData();
            }

        }

        /**
         * @fn  public void pose(double yaw, double pitch, double roll, double a, double b, double c)
         *
         * @brief   Changes the pose of the robot
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   yaw     The yaw.
         * @param   pitch   The pitch.
         * @param   roll    The roll.
         * @param   a       Distance to move on x axis.
         * @param   b       Distance to move on y axis.
         * @param   c       Distance to move on z axis.
         */

        public void pose(double yaw, double pitch, double roll, double a, double b, double c, byte mode)
        {
            //If the direction has changed, center all legs
            if (lastDirection != (byte)Controller.directions.POSE)
            {
                centerLegs();

                foreach (Leg l in legs)
                {
                    l.setColor(120);
                }

            }

            if (mode == (byte)Controller.modes.BALANCE)
            {

                //get accelerometer data
                accel.read();
                //change pitch and roll based on accelerometer data
                balance(accel.Pitch, accel.Roll);

                //change pose
                foreach (Leg leg in legs)
                {
                    leg.calcPositionCenter();
                    leg.calcPose(0, this.pitch, -this.roll, 0, 0, 0);

                }
            }
            else
            {
                this.pitch = 0;
                this.roll = 0;
                //change pose
                foreach (Leg leg in legs)
                {
                    leg.calcPositionCenter();
                    leg.calcPose(yaw, pitch, roll, a, b, c);

                }

            }


            //send tcp positions
            foreach (Leg leg in legs)
            {
                leg.calcData();
            }

            lastDirection = (byte)Controller.directions.POSE;

        }


        /**
         * @fn  private void balance(double newPitch, double newRoll)
         *
         * @brief   PID-controller to balance the robot
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         *
         * @param   newPitch    The new pitch.
         * @param   newRoll     The new roll.
         */

        private void balance(double newPitch, double newRoll)
        {
            //calc new pitch
            pitch_e[2] = pitch_e[1];
            pitch_e[1] = pitch_e[0];
            pitch_e[0] = newPitch;
            pitch = pitch + k1 * pitch_e[0] + k2 * pitch_e[1] + k3 * pitch_e[2];

            //calc new roll
            roll_e[2] = roll_e[1];
            roll_e[1] = roll_e[0];
            roll_e[0] = newRoll;
            roll = roll + k1 * roll_e[0] + k2 * roll_e[1] + k3 * roll_e[2];

            //check boundaries
            if (pitch > maxPitch)
            { pitch = maxPitch; }
            else if (pitch < -maxPitch)
            { pitch = -maxPitch; }

            //check boundaries
            if (roll > maxRoll)
            { roll = maxRoll; }
            else if (roll < -maxRoll)
            { roll = -maxRoll; }
        }

        /**
         * @fn  private void centerLegs()
         *
         * @brief   Centers all legs.
         *
         * @author  Alexander Miller
         * @date    13.08.2017
         */

        private void centerLegs()
        {
            //time between steps
            int time = 100;

            pitch = 0;
            roll = 0;

            foreach (Leg l in legs)
            {
                l.calcPose(0, pitch, roll, 0, 0, 0);

                l.calcData();

            }


            //put all legs down
            foreach (Leg l in legs)
            {
                l.ZPos = 0;

                l.calcData();

            }
            Task.Delay(time).Wait();


            legs[1].calcData();
            legs[2].calcData();
            legs[5].calcData();
            Task.Delay(time).Wait();

            legs[1].ZPos = (int)legs[1].StepSizeZ;
            legs[1].calcData();
            legs[2].ZPos = (int)legs[2].StepSizeZ;
            legs[2].calcData();
            legs[5].ZPos = (int)legs[5].StepSizeZ;
            legs[5].calcData();
            Task.Delay(time).Wait();

            legs[1].calcPositionCenter();
            legs[1].ZPos = (int)legs[1].StepSizeZ;
            legs[1].calcData();
            legs[2].calcPositionCenter();
            legs[2].ZPos = (int)legs[2].StepSizeZ;
            legs[2].calcData();
            legs[5].calcPositionCenter();
            legs[5].ZPos = (int)legs[5].StepSizeZ;
            legs[5].calcData();
            Task.Delay(time).Wait();

            legs[1].calcPositionCenter();
            legs[1].calcData();
            legs[2].calcPositionCenter();
            legs[2].calcData();
            legs[5].calcPositionCenter();
            legs[5].calcData();
            Task.Delay(time).Wait();

            //

            legs[0].calcData();
            legs[3].calcData();
            legs[4].calcData();
            Task.Delay(time).Wait();

            legs[0].ZPos = (int)legs[0].StepSizeZ;
            legs[0].calcData();
            legs[3].ZPos = (int)legs[3].StepSizeZ;
            legs[3].calcData();
            legs[4].ZPos = (int)legs[4].StepSizeZ;
            legs[4].calcData();
            Task.Delay(time).Wait();

            legs[0].calcPositionCenter();
            legs[0].ZPos = (int)legs[0].StepSizeZ;
            legs[0].calcData();
            legs[3].calcPositionCenter();
            legs[3].ZPos = (int)legs[3].StepSizeZ;
            legs[3].calcData();
            legs[4].calcPositionCenter();
            legs[4].ZPos = (int)legs[4].StepSizeZ;
            legs[4].calcData();
            Task.Delay(time).Wait();

            legs[0].calcPositionCenter();
            legs[0].calcData();
            legs[3].calcPositionCenter();
            legs[3].calcData();
            legs[4].calcPositionCenter();
            legs[4].calcData();
            Task.Delay(time).Wait();

        }

        public void shutdown()
        {
            foreach (Leg item in legs)
            {
                item.setColor(300);
            }
        }

        #endregion Functions
    }
}
