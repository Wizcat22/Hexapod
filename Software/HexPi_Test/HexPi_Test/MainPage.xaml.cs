using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading.Tasks;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.Gaming.Input;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Windows.Devices.I2c;
using Windows.Devices.Enumeration;
using System.Windows;

// Die Vorlage "Leere Seite" ist unter http://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409 dokumentiert.

namespace HexPi_Test
{
    /// <summary>
    /// Eine leere Seite, die eigenständig verwendet oder zu der innerhalb eines Rahmens navigiert werden kann.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        MatrixBuilder<double> M = Matrix<double>.Build;
        VectorBuilder<double> V = Vector<double>.Build;
        Gamepad input;
        Task inputTask;
        I2cDevice i2c;
        enum directions { CENTER, MOVEX, MOVEY, TURN };
        double t = 0.0;

        public MainPage()
        {
            this.InitializeComponent();
            init();
        }

        public void init()
        {
            init_Gamepad();
            init_i2c();
            inputTask = Task.Factory.StartNew(() => HandleInputs());


        }

        private void HandleInputs()
        {

            while (true)
            {
                if (input != null && i2c != null)
                {


                    GamepadReading gamepadStatus = input.GetCurrentReading();

                    Vector<double> inputData = V.Dense(3);

                    inputData[0] = gamepadStatus.LeftThumbstickY * 30;
                    inputData[1] = gamepadStatus.LeftThumbstickX * 30;
                    inputData[2] = (gamepadStatus.LeftTrigger - gamepadStatus.RightTrigger) * 30;

                    Vector<double> positionData = CalculateLegPosition(inputData);

                    Vector<double> angleData = CalculateMotorAngle(positionData);

                    Byte[] b = ConvertData(angleData);

                    SendData(b);

                }
            }
        }

        private void SendData(byte[] b)
        {
            try
            {
                Debug.WriteLine("A: " + b[0] + " B: " + b[1] + " C: " + b[2]);
                i2c.Write(b);
            }
            catch (Exception e)
            {
                Debug.WriteLine(e.Source);
            }

        }

        private Vector<double> CalculateMotorAngle(Vector<double> data)
        {
            double zOffset = 95;
            double A1 = 30;
            double A2 = 60;
            double A3 = 95;
            double L1 = 0;
            double L2 = 0;
            double b = 0;
            double x = data[0];
            double y = data[1];
            double z = data[2];
            Vector<double> motorAngle = V.Dense(3);

            //ALPHA
            motorAngle[0] = Math.Atan2(x, A1 + A2 + y);

            //BETA
            L1 = zOffset - z;
            L2 = A2 + y;
            b = Math.Sqrt(L1 * L1 + L2 * L2);

            motorAngle[1] = Math.Acos(L1 / b);
            motorAngle[1] = motorAngle[1] + Math.Acos((A2 * A2 - A3 * A3 + b * b) / (2 * A2 * b));

            //GAMMA
            motorAngle[2] = Math.Acos((A3 * A3 - b * b + A2 * A2) / (2 * A3 * A2));
           
            //RAD TO DEG
            motorAngle[0] = motorAngle[0] * 180 / Math.PI;
            motorAngle[1] = (motorAngle[1] * 180 / Math.PI - 90) * 1;
            motorAngle[2] = (motorAngle[2] * 180 / Math.PI - 90) * -1;

            Debug.WriteLine("DEBUG: " + motorAngle[0] + " :: " + motorAngle[1] + " :: " + motorAngle[2]);
            return motorAngle;

        }

        private Byte[] ConvertData(Vector<double> angleData)
        {
            Byte a = (byte)(1.38888888888 * angleData[0] + 187.5);
            Byte b = (byte)(1.38888888888 * angleData[1] + 187.5);
            Byte c = (byte)(1.38888888888 * angleData[2] + 187.5);
            Byte[] bytes = { 11, a, b, c, a, b, c, a, b, c, 187,187,187, 22 };
            return bytes;
        }

        //TODO: IMPLEMENT WALKING GIAT FUNCTIONS
        private Vector<double> CalculateLegPosition(Vector<double> data)
        {
            int dir = 0;
            int stepResolution = 10; // 10t per step
            double stepsizeX = 30;
            double stepsizeY = 30;
            double stepsizeZ = 30;
            double threshold = 0.2;
            double x = data[0];
            double y = data[1];
            double z = data[2];
            double x_new = 0.0;
            double y_new = 0.0;
            double z_new = 0.0;

            if (x < threshold && y < threshold)
            {
                dir = (int)directions.CENTER;
            }
            else if (x >= threshold && x >= y)
            {
                dir = (int)directions.MOVEX;
            }
            else if (y >= threshold && y >= x)
            {
                dir = (int)directions.MOVEY;
            }


            switch (dir)
            {
                case (int)directions.CENTER:





                    break;
                case (int)directions.MOVEX:
                    t = (t + x) % stepResolution * 2;
                    if (t <= stepResolution)
                    {
                        x_new = stepsizeX / stepResolution * t;
                    }
                    else
                    {
                        x_new = stepsizeX / stepResolution * (t-stepResolution);
                    }


                    break;
                case (int)directions.MOVEY:



                    break;

            }

            if(t <= stepResolution)
            {
                z_new = 0;
            }
            else
            {
                z_new = (-stepsizeZ/((stepResolution / 2)* (stepResolution / 2))) *(t-stepResolution-stepResolution/2)*(t-stepResolution- stepResolution / 2) +stepsizeZ;
            }



            return data;
        }

        private async void init_i2c()
        {
            var settings = new I2cConnectionSettings(0x0); // Arduino address


            settings.BusSpeed = I2cBusSpeed.StandardMode;

            string aqs = I2cDevice.GetDeviceSelector("I2C1");

            var dis = await DeviceInformation.FindAllAsync(aqs);

            i2c = await I2cDevice.FromIdAsync(dis[0].Id, settings);
        }

        public void init_Gamepad()
        {
            if (Gamepad.Gamepads.Count() > 0)
            {
                input = Gamepad.Gamepads.First();
            }
            else
            {
                Debug.WriteLine("Error: No Gamepad connected!");
            }

            Gamepad.GamepadAdded += GamepadAddedHandler;
            Gamepad.GamepadRemoved += GamepadRemovedHandler;


        }

        private void GamepadRemovedHandler(object sender, Gamepad e)
        {
            input = null;
            Debug.WriteLine("Warning: Gamepad was removed");
        }

        private void GamepadAddedHandler(object sender, Gamepad e)
        {
            if (Gamepad.Gamepads.Count() > 0)
            {
                input = Gamepad.Gamepads.First();
            }
            else
            {
                Debug.WriteLine("Error: Connection failed");
            }
        }



    }
}
