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
                    double[,] a = { { gamepadStatus.LeftThumbstickX },
                                    { gamepadStatus.LeftThumbstickY},
                                    { gamepadStatus.LeftTrigger - gamepadStatus.RightTrigger}};
                    Matrix<double> inputData = M.DenseOfArray(a);

                    Matrix<double> positionData = CalculateLegPosition(inputData);

                    Matrix<double> angleData = CalculateMotorAngle(positionData);

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
            catch(Exception e)
            {
                Debug.WriteLine(e.Source);
            }

        }

        private Matrix<double> CalculateMotorAngle(Matrix<double> data)
        {
            Matrix<double> angleData;
            double[,] a = { { 45 * data[0,0] },
                            { 45 * data[1,0]},
                            { 45 * data[2,0]}};
            angleData = M.DenseOfArray(a);
            return angleData;

        }

        private Byte[] ConvertData(Matrix<double> angleData)
        {
            Byte a = (byte)(1.38888888888 * angleData[0, 0] + 187.5);
            Byte b = (byte)(1.38888888888 * angleData[1, 0] + 187.5);
            Byte c = (byte)(1.38888888888 * angleData[2, 0] + 187.5);
            Byte[] bytes = { a, b, c };
            return bytes;
        }

        private Matrix<double> CalculateLegPosition(Matrix<double> data)
        {
            return data;
        }

        private async void init_i2c()
        {
            var settings = new I2cConnectionSettings(0x00); // Arduino address


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
