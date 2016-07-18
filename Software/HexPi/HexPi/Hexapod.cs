using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    class Hexapod
    {
        //Objects
        ServoHat servo = new ServoHat();
        Accelerometer accel = new Accelerometer();
        //******

        //Fields
        byte lastDirection = 0;
        const double bodyWidth = 20;
        const double bodyHeigth = 30;
        //******

        //Arrays
        byte[] data = new byte[26];
        int[] gait = { 25, 75, 75, 25, 25, 75 };
        ILeg[] legs = new ILeg[6];
        //******


        //Functions
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
            data[0] = 11;
            //set end byte
            data[25] = 22;

            //init legs
            for (int i = 0; i < legs.Length; i++)
            {
                if (i % 2 == 0)
                {
                    legs[i] = new LeftLeg(gait[i],0,0,0);
                }
                else
                {
                    legs[i] = new RightLeg(gait[i],0,0,0);
                }

            }
        }

        public void move(double inc ,byte dir)
        {
            accel.read();
            calcOffsets();

            if(dir != lastDirection && lastDirection != (byte)Controller.directions.CENTER)
            {
                centerLegs();                
                lastDirection = (byte)Controller.directions.CENTER;
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

        private void calcOffsets()
        {
            for (int i = 0; i < legs.Length; i++)
            {
                if (i % 2 == 0)
                {
                    legs[i].Zoff -= 0.1 * Math.Sin(accel.angleYZ) * ((bodyWidth / 2) + legs[i].YPos);

                    //legs[i].Yoff = cosyz;
                }
                else
                {
                    legs[i].Zoff += 0.1 * Math.Sin(accel.angleYZ) * ((bodyWidth / 2) + legs[i].YPos);
                    //legs[i].Yoff = cosyz;
                }

            }

            legs[0].Zoff += 0.1 * Math.Sin(accel.angleXZ) * ((bodyHeigth / 2) + legs[0].XPos);
            legs[1].Zoff += 0.1 * Math.Sin(accel.angleXZ) * ((bodyHeigth / 2) + legs[1].XPos);


            legs[2].Zoff += 0.1 * Math.Sin(accel.angleXZ) * legs[2].XPos;
            legs[3].Zoff += 0.1 * Math.Sin(accel.angleXZ) * legs[3].XPos;


            legs[4].Zoff -= 0.1 * Math.Sin(accel.angleXZ) * ((bodyHeigth / 2) + legs[4].XPos);
            legs[5].Zoff -= 0.1 * Math.Sin(accel.angleXZ) * ((bodyHeigth / 2) + legs[5].XPos);



            //legs[1].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);
            //legs[3].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);
            //legs[5].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);

            //legs[0].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);
            //legs[2].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);
            //legs[4].Xoff = Math.Cos(accel.angleXZ) * (legs[1].BodyWidth / 2);

        }

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

        private void centerLegs()
        {
            int time = 100;
            double zOld = 0.0;

            foreach (ILeg l in legs)
            {
                zOld = l.ZPos;
                //Up
                l.ZPos += l.StepSizeZ/3;
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
                l.ZPos = zOld;
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
