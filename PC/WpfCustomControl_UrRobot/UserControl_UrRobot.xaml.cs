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

namespace WpfCustomControl_UrRobot
{
    /// <summary>
    /// UserControl1.xaml 的互動邏輯
    /// </summary>
    public partial class UserControl_UrRobot : UserControl
    {
        public UserControl_UrRobot()
        {
            InitializeComponent();
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {

            Window1 a = new Window1();
            a.Show()
        }
    }
}
