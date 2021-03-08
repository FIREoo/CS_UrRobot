using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using UrRobot.Socket;
using UrRobot.Coordinates;

namespace WpfCustomControl_UrRobot
{
    /// <summary>
    /// UserControl1.xaml 的互動邏輯
    /// </summary>
    public partial class UserControl_UrRobot : UserControl
    {
        UrSocketControl UR = new UrSocketControl();
        public UserControl_UrRobot()
        {
            InitializeComponent();
            Dispatcher.Invoke(() => { rect_server.Fill = (DefaultColor.Red); });
            Dispatcher.Invoke(() => { rect_client.Fill = (DefaultColor.Red); });
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {

            Window1 a = new Window1();
            a.Show();
        }

        private void Btn_startServer_Click(object sender, RoutedEventArgs e)
        {
            UR.stopServer();
            UR.startServer("192.168.0.111", 888);

            Dispatcher.Invoke(() => { rect_server.Fill = (DefaultColor.Yellow); });
            UR.stateChange += (state) =>
            {
                if (state == tcpState.Connect)
                    Dispatcher.Invoke(() => { rect_server.Fill = (DefaultColor.Green); });
                else if (state == tcpState.Disconnect)
                    Dispatcher.Invoke(() => { rect_server.Fill = (DefaultColor.Red); });
            };
        }

        UrSocketControl.Client URc = new UrSocketControl.Client();
        private void Btn_startClient_Click(object sender, RoutedEventArgs e)
        {
            if (!URc.ClientConnect("192.168.0.200"))
                Console.WriteLine("Client連線失敗");

            URc.Client_RTDE();

            Dispatcher.Invoke(() => { rect_client.Fill = (DefaultColor.Yellow); });
            URc.stateChange += (state) =>
            {
                if (state == tcpState.Connect)
                    Dispatcher.Invoke(() => { rect_client.Fill = (DefaultColor.Green); });
                else if (state == tcpState.Disconnect)
                    Dispatcher.Invoke(() => { rect_client.Fill = (DefaultColor.Red); });
            };
        }


        private void Btn_getPosition_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if(e.ChangedButton == MouseButton.Left)
            {
                URCoordinates pos = URc.getPosition();
                tb_urPos_x.Text = (pos.X.mm).ToString("0.00");
                tb_urPos_y.Text = (pos.Y.mm).ToString("0.00");
                tb_urPos_z.Text = (pos.Z.mm).ToString("0.00");
                tb_urPos_rx.Text = (pos.Rx.rad).ToString("0.00");
                tb_urPos_ry.Text = (pos.Ry.rad).ToString("0.00");
                tb_urPos_rz.Text = (pos.Rz.rad).ToString("0.00");
            }
        }
    }
    public static class DefaultColor
        {
            public static SolidColorBrush DarkRed = new SolidColorBrush(Color.FromRgb(140, 68, 64));
            public static SolidColorBrush Red = new SolidColorBrush(Color.FromRgb(174, 83, 80));
            public static SolidColorBrush LightRed = new SolidColorBrush(Color.FromRgb(197, 129, 126));
            public static SolidColorBrush DarkGreen = new SolidColorBrush(Color.FromRgb(79, 120, 67));
            public static SolidColorBrush Green = new SolidColorBrush(Color.FromRgb(105, 159, 89));
            public static SolidColorBrush LightGreen = new SolidColorBrush(Color.FromRgb(177, 206, 168));
            public static SolidColorBrush DarkBlue = new SolidColorBrush(Color.FromRgb(59, 78, 169));
            public static SolidColorBrush Blue = new SolidColorBrush(Color.FromRgb(91, 102, 189));
            public static SolidColorBrush LightBlue = new SolidColorBrush(Color.FromRgb(153, 161, 211));
            public static SolidColorBrush DarkYellow = new SolidColorBrush(Color.FromRgb(230, 159, 13));
            public static SolidColorBrush Yellow = new SolidColorBrush(Color.FromRgb(243, 180, 48));
            public static SolidColorBrush LightYellow = new SolidColorBrush(Color.FromRgb(248, 205, 116));

        }
}
