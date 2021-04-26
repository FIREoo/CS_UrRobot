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
using System.IO;
using System.Windows.Threading;

namespace WpfCustomControl_UrRobot
{
    /// <summary>
    /// UserControl1.xaml 的互動邏輯
    /// </summary>
    /// 加入此專案，並在XMAL檔案中，必須要加入下面這行
    /// xmlns:UrControl="clr-namespace:WpfCustomControl_UrRobot;assembly=WpfCustomControl_UrRobot"
    /// 使用方式
    /// <control:UserControl_UrRobot />
    public partial class UserControl_UrRobot : UserControl
    {
        public UrSocketControl UR = new UrSocketControl();
        string rootPath = "Path\\";
        public UserControl_UrRobot()
        {
            InitializeComponent();
            Dispatcher.Invoke(() => { rect_server.Fill = (DefaultColor.Red); });
            Dispatcher.Invoke(() => { rect_client.Fill = (DefaultColor.Red); });

            //path
            if (Directory.Exists(rootPath) == false)
                Directory.CreateDirectory(rootPath);

            UI_initial();
        }

        DispatcherTimer timer_UI = new DispatcherTimer();
        public void UI_initial()
        {
            btn_urc_play.RollUp();
            btn_urc_stop.RollUp();

            //update UI timer
            timer_UI.Interval = TimeSpan.FromMilliseconds(500);
            timer_UI.Tick += _timer_Tick;
            void _timer_Tick(object ss, EventArgs ee)
            {
                if (loop_UIpos == true)
                {
                    URCoordinates pos = URc.getPosition();
                    setUIPostion(pos);
                }
                if (loop_UIjoint == true)
                {
                    URJoint joint = URc.getJoint();
                    setUIJoint(joint);
                }
            }
        }
        private void Button_Click(object sender, RoutedEventArgs e)
        {


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
            UR.UrCmd += (cmd) =>
            {
                Dispatcher.Invoke(() =>
                {
                    tb_cmd.Text = cmd.ToString();
                    Console.WriteLine(cmd.ToString());
                });
            };
        }

        //UR client
        UrSocketControl.Client URc = new UrSocketControl.Client();
        private void Btn_startClient_Click(object sender, RoutedEventArgs e)
        {
            Dispatcher.Invoke(() => { rect_client.Fill = (DefaultColor.Yellow); });
            URc.stateChange += (state) =>
            {
                if (state == tcpState.Connect)
                {
                    Dispatcher.Invoke(() =>
                    {
                        rect_client.Fill = (DefaultColor.Green);
                        btn_urc_play.RollOut();
                        btn_urc_stop.RollOut();
                    });
                }
                else if (state == tcpState.Disconnect)
                {
                    Dispatcher.Invoke(() =>
                    {
                        rect_client.Fill = (DefaultColor.Red);
                        btn_urc_play.RollUp();
                        btn_urc_stop.RollUp();
                    });
                }
            };

            if (!URc.ClientConnect("192.168.0.200"))
                Console.WriteLine("Client連線失敗");

            URc.Client_RTDE();
        }
        private void Btn_urc_play_Click(object sender, RoutedEventArgs e)
        {
            URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.play, "");
        }
        private void Btn_urc_stop_Click(object sender, RoutedEventArgs e)
        {
            URc.ClientCmd(UrSocketControl.Client.DashBoardCommand.stop, "");
        }
        //取 座標數值
        private void Btn_getPosition_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left)
            {
                URCoordinates pos = URc.getPosition();
                setUIPostion(pos);
            }
        }
        bool loop_UIpos = false;
        private void Cb_loop_getPosition_Click(object sender, RoutedEventArgs e)
        {

            if (((CheckBox)sender).IsChecked == true)
            {
                loop_UIpos = true;
            }
            else
            {
                loop_UIpos = false;
            }
            if (loop_UIpos == true || loop_UIjoint == true)
                if (timer_UI.IsEnabled == false)
                    timer_UI.Start();
            if (loop_UIpos == false && loop_UIjoint == false)
                if (timer_UI.IsEnabled == true)
                    timer_UI.Stop();
        }
        public void setUIPostion(URCoordinates pos)
        {
            tb_urPos_x.Text = (pos.X.mm).ToString("0.0");
            tb_urPos_y.Text = (pos.Y.mm).ToString("0.0");
            tb_urPos_z.Text = (pos.Z.mm).ToString("0.0");
            tb_urPos_rx.Text = (pos.Rx.rad).ToString("0.000");
            tb_urPos_ry.Text = (pos.Ry.rad).ToString("0.000");
            tb_urPos_rz.Text = (pos.Rz.rad).ToString("0.000");
        }
        public URCoordinates getUIPosition()
        {
            double x = double.Parse(tb_urPos_x.Text);
            double y = double.Parse(tb_urPos_y.Text);
            double z = double.Parse(tb_urPos_z.Text);
            double Rx = double.Parse(tb_urPos_rx.Text);
            double Ry = double.Parse(tb_urPos_ry.Text);
            double Rz = double.Parse(tb_urPos_rz.Text);

            return new URCoordinates(x.mm(), y.mm(), z.mm(), Rx.rad(), Ry.rad(), Rz.rad());
        }

        private void Btn_getJoint_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left)
            {
                URJoint joint = URc.getJoint();
                setUIJoint(joint);
            }
        }
        bool loop_UIjoint = false;
        private void Cb_loop_getJoint_Click(object sender, RoutedEventArgs e)
        {
            if (((CheckBox)sender).IsChecked == true)
            {
                loop_UIjoint = true;
            }
            else
            {
                loop_UIjoint = false;
            }
            if (loop_UIpos == true || loop_UIjoint == true)
                if (timer_UI.IsEnabled == false)
                    timer_UI.Start();
            if (loop_UIpos == false && loop_UIjoint == false)
                if (timer_UI.IsEnabled == true)
                    timer_UI.Stop();
        }
        public void setUIJoint(URJoint joint)
        {
            tb_urJoint_1.Text = (joint.J1.rad).ToString("0.000");
            tb_urJoint_2.Text = (joint.J2.rad).ToString("0.000");
            tb_urJoint_3.Text = (joint.J3.rad).ToString("0.000");
            tb_urJoint_4.Text = (joint.J4.rad).ToString("0.000");
            tb_urJoint_5.Text = (joint.J5.rad).ToString("0.000");
            tb_urJoint_6.Text = (joint.J6.rad).ToString("0.000");
        }
        public URJoint getUIJoint()
        {
            double j1 = double.Parse(tb_urJoint_1.Text);
            double j2 = double.Parse(tb_urJoint_2.Text);
            double j3 = double.Parse(tb_urJoint_3.Text);
            double j4 = double.Parse(tb_urJoint_4.Text);
            double j5 = double.Parse(tb_urJoint_5.Text);
            double j6 = double.Parse(tb_urJoint_6.Text);

            return new URJoint(j1.rad(), j2.rad(), j3.rad(), j4.rad(), j5.rad(), j6.rad());
        }



        //UR server control
        private void Btn_cmd_stop_Click(object sender, RoutedEventArgs e)
        {
            UR.Stop();
        }

        //go path combobox
        private void TextBlock_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Right)
            {
                System.Diagnostics.Process.Start(rootPath);
            }
        }
        private void ComboBox_goPath_ContextMenuOpening(object sender, ContextMenuEventArgs e)
        {
            FileInfo[] rootFile = (new DirectoryInfo(rootPath)).GetFiles("*.path");//get files
            Console.WriteLine("in");
            comboBox_goPath.Items.Clear();
            foreach (FileInfo fi in rootFile)
            {
                comboBox_goPath.Items.Add(fi.Name.Substring(0, fi.Name.IndexOf('.')));
            }
        }
        private void Btn_goPath_Click(object sender, RoutedEventArgs e)
        {
            Task.Run(() => { UR.goFile(comboBox_goPath.Text + ".path"); });
        }

        //jog
        private void Btn_jog_x_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.MouseDevice.LeftButton == MouseButtonState.Pressed)
            {
                UR.goJog(new URCoordinates(tb_in1.Text.toFloat().mm(), tb_in2.Text.toFloat().mm(), tb_in3.Text.toFloat().mm(), tb_in4.Text.toFloat().rad(), tb_in5.Text.toFloat().rad(), tb_in6.Text.toFloat().rad()));
            }
            else if (e.MouseDevice.RightButton == MouseButtonState.Pressed)
            {
                UR.goJog(new URCoordinates(-(tb_in1.Text.toFloat()).mm(), -(tb_in2.Text.toFloat()).mm(), -(tb_in3.Text.toFloat()).mm(), -(tb_in4.Text.toFloat()).rad(), -(tb_in5.Text.toFloat()).rad(), -(tb_in6.Text.toFloat()).rad()));
            }
            e.Handled = true;
        }
        private void Btn_jog_x_PreviewMouseUp(object sender, MouseButtonEventArgs e)
        {
            UR.Stop();
            e.Handled = true;
        }

        //調整 座標數值
        private void Tb_in_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            if (e.Delta > 0)
            {
                ((TextBox)sender).Text = (float.Parse(((TextBox)sender).Text) + 5).ToString("0.0");
            }
            else if (e.Delta < 0)
            {
                ((TextBox)sender).Text = (float.Parse(((TextBox)sender).Text) - 5).ToString("0.0");
            }
        }
        private void Tb_in_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Middle)
                ((TextBox)sender).Text = "0.0";
        }



        URCoordinates P1 = new URCoordinates(-380.mm(), 0.mm(), 400.mm(), 0.rad(), 0.rad(), (Math.PI * 1.5).rad());
        URCoordinates P2 = new URCoordinates(-450.mm(), 0.mm(), 400.mm(), 0.rad(), 0.rad(), (Math.PI * 1.5).rad());
        URCoordinates P3 = new URCoordinates(-415.mm(), 20.mm(), 420.mm(), 0.rad(), 0.rad(), (Math.PI * 1.5).rad());
        URCoordinates P4 = new URCoordinates(-415.mm(), -20.mm(), 380.mm(), 0.rad(), 0.rad(), (Math.PI * 1.5).rad());
        private void Btn_test1_Click(object sender, RoutedEventArgs e)
        {
            UR.goPosition(P1);
        }

        private void Btn_test2_Click(object sender, RoutedEventArgs e)
        {
            UR.goPosition(P2);
        }

        private void Btn_test3_PreviewMouseDown(object sender, MouseButtonEventArgs e)
        {
            URCoordinates Pnow = URc.getPosition();


            double D = Math.Sqrt((P1.X.mm - Pnow.X.mm) * (P1.X.mm - Pnow.X.mm));
            double d = 20;
            if (D > d)
            {
                double dx = P1.X.mm - Pnow.X.mm;
                double cmdX = (dx * d) / D;
                Console.WriteLine("cmdX:" + cmdX);
                URCoordinates jog = new URCoordinates(cmdX.mm(), 0.mm(), 0.mm(), 0.rad(), 0.rad(), 0.rad());
                UR.goJog(jog);
            }
            else
            {
                UR.goPosition(P1);
            }
            e.Handled = true;
        }
        private void Btn_test3_PreviewMouseUp(object sender, MouseButtonEventArgs e)
        {
            UR.Stop();
            e.Handled = true;
        }

        private void Btn_test3_Click(object sender, RoutedEventArgs e)
        {
            UR.goTrack2(P1, URc.getJoint());
            UR.goTrack2(P2, URc.getJoint());
            UR.goTrack2(P3, URc.getJoint());
            UR.goTrack2(P4, URc.getJoint());
            UR.goTrack2(P1, URc.getJoint());

        }
        private void Btn_test4_Click(object sender, RoutedEventArgs e)
        {
            UR.goTrack2(P2, URc.getJoint());
        }


        private void Btn_cmdWindow_Click(object sender, RoutedEventArgs e)
        {
            CmdWindow w = new CmdWindow(this);
            w.Show();
        }

        private void Grid_dragOutJoint_MouseMove(object sender, MouseEventArgs e)
        {
            if (e.LeftButton == MouseButtonState.Pressed)
            {
                DragDrop.DoDragDrop((Grid)sender, getUIJoint().ToString(), DragDropEffects.Copy);
            }
        }
        private void Grid_dragOutPos_MouseMove(object sender, MouseEventArgs e)
        {
            if (e.LeftButton == MouseButtonState.Pressed)
            {
                DragDrop.DoDragDrop((Grid)sender, getUIPosition().ToString(), DragDropEffects.Copy);
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

    public static class ControlEx
    {
        public static void RollUp(this Control control)
        {
            if (control.IsEnabled == false)//already roll up
                return;

            control.IsEnabled = false;
            control.Height = control.Height / 3.0;
        }
        public static void RollOut(this Control control)
        {
            if (control.IsEnabled == true)//already roll out
                return;
            control.IsEnabled = true;
            control.Height = control.Height * 3.0;
        }

    }

}
