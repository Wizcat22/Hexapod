using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HexPi
{
    abstract class ILeg
    {
        protected double zOffset = 95;
        protected double A1 = 30;
        protected double A2 = 60;
        protected double A3 = 95;
        protected double L1 = 0;
        protected double L2 = 0;
        protected double b = 0;
        protected double alpha = 0;
        protected double beta = 0;
        protected double gamma = 0;
        protected double t = 0;
        protected double tOffset = 0;
        protected double period = 100;
        protected double stepSizeX = 30;
        protected double stepSizeY = 30;
        protected double stepSizeZ = 30;
        protected double xPos = 0;
        protected double yPos = 0;
        protected double zPos = 0;
        protected byte motorData0 = 0;
        protected byte motorData1 = 0;
        protected byte motorData2 = 0;

        public double XPos
        {
            get
            {
                return xPos;
            }

            set
            {
                xPos = value;
            }
        }

        public double YPos
        {
            get
            {
                return yPos;
            }

            set
            {
                yPos = value;
            }
        }

        public double ZPos
        {
            get
            {
                return zPos;
            }

            set
            {
                zPos = value;
            }
        }

        public double StepSizeX
        {
            get
            {
                return stepSizeX;
            }

            set
            {
                stepSizeX = value;
            }
        }

        public double StepSizeY
        {
            get
            {
                return stepSizeY;
            }

            set
            {
                stepSizeY = value;
            }
        }

        public double StepSizeZ
        {
            get
            {
                return stepSizeZ;
            }

            set
            {
                stepSizeZ = value;
            }
        }

        public double TOffset
        {
            get
            {
                return tOffset;
            }

            set
            {
                tOffset = value;
            }
        }

        public void inverseKinematics()
        {
            //ALPHA
            alpha = Math.Atan2(xPos, A1 + A2 + yPos);

            //BETA
            L1 = zOffset - zPos;
            L2 = A2 + yPos;
            b = Math.Sqrt(L1 * L1 + L2 * L2);

            beta = Math.Acos(L1 / b);
            beta = beta + Math.Acos((A2 * A2 - A3 * A3 + b * b) / (2 * A2 * b));

            //GAMMA
            gamma = Math.Acos((A3 * A3 - b * b + A2 * A2) / (2 * A3 * A2));

            //RAD TO DEG
            alpha = alpha * 180 / Math.PI;
            beta = (beta * 180 / Math.PI - 90) * 1;
            gamma = (gamma * 180 / Math.PI - 90) * -1;

            //Debug.WriteLine("DEBUG: " + alpha + " :: " + beta + " :: " + gamma);

        }

        //public void calcPositions(byte direction, double increment)
        //{
        //    t = ((t + increment) % period + period) % period;

        //    switch (direction)
        //    {
        //        //No motion
        //        case 0:
        //            calcPositionsCenter();
        //            break;
        //        //Motion in X-direction
        //        case 1:
        //            calcPositionsX();
        //            break;
        //        //Motion in Y-direction
        //        case 2:
        //            calcPositionsY();
        //            break;
        //        //Rotate
        //        case 3:
        //            calcPositionsRotation();
        //            break;
        //        default: break;
        //    }

        //    //Motion in z-direction
        //    calcPositionZ();
        //}

        protected void calcPositionZ()
        {
            if (t <= period / 2)
            {
                zPos = 0;
            }
            else
            {
                zPos = -1 * (stepSizeZ * 16 / (period * period)) * (t - period / 2 - period / 4) * (t - period / 2 - period / 4) + stepSizeZ;
            }
        }

        public void calcPositionCenter(double increment)
        {
            t = ((t + increment) % period + period) % period;
            t = tOffset;
            xPos = 0.0;
            yPos = 0.0;
        }

        public void calcPositionX(double increment)
        {
            t = ((t + increment) % period + period) % period;
            yPos = 0.0;
            if (t <= period / 2)
            {
                xPos = 4 * stepSizeX / period * t - stepSizeX;
            }
            else
            {
                xPos = -4 * stepSizeX / period * (t - period / 2) + stepSizeX;
            }
            calcPositionZ();
        }

        public void calcPositionY(double increment)
        {
            t = ((t + increment) % period + period) % period;
            xPos = 0.0;
            if (t <= period / 2)
            {
                yPos = 4 * stepSizeY / period * t - stepSizeY;
            }
            else
            {
                yPos = -4 * stepSizeY / period * (t - period / 2) + stepSizeY;
            }
            calcPositionZ();
        }

        public abstract void calcPositionR(double increment);

        public byte getMotorData(int n)
        {
            switch (n)
            {
                case 0: return motorData0;
                case 1: return motorData1;
                case 2: return motorData2;
                default: return 100;
            }

        }

        public abstract void calcData();

    }
}
