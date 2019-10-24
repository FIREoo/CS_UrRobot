using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UrRobot.Coordinates
{
    static public class ex
    {
        static public Unit M(this double value)
        {
            return new Unit((float)value);
        }
        static public Unit M(this int value)
        {
            return new Unit((float)value);
        }
        static public Unit mm(this int value)
        {
            return new Unit((float)value/1000f);
        }
        static public Unit mm(this double value)
        {
            return new Unit((float)value / 1000f);
        }
        static public Angle rad(this double value)
        {
            return new Angle((float)value);
        }
        static public Angle rad(this int value)
        {
            return new Angle((float)value);
        }
        static public Angle deg(this double value)
        {
            return new Angle(value * (float)Math.PI / 180.0f);
        }
        static public Angle deg(this int value)
        {
            return new Angle(value * (float)Math.PI / 180.0f);
        }

        static public int toInt(this string str)
        {
            return int.Parse(str);
        }
        static public float toFloat(this string str)
        {
            return float.Parse(str);
        }
    }
    public class Unit
    {
        float _M = 0;
        public Unit(float v)
        {
            _M = v;
        }
        public Unit(double v)
        {
            _M = (float)v;
        }
        public Unit()
        {
            _M = 0;
        }

        public float M
        {
            get { return _M; }
            set { _M = value; }
        }
        public float mm
        {
            get { return _M * 1000.0f; }
            set { _M = value / 1000.0f; }
        }
    }
    public class Angle
    {
        float _rad = 0;
        public Angle(float v)
        {
            _rad = v;
        }
        public Angle(double v)
        {
            _rad = (float)v;
        }
        public Angle()
        {
            _rad = 0;
        }
        public float rad
        {
            get { return _rad; }
            set { _rad = value; }
        }
        public float deg
        {
            get { return _rad * 180.0f / (float)Math.PI; }
            set { _rad = value * (float)Math.PI / 180.0f; }
        }

    }


    public class URCoordinates
    {
        public Unit X = new Unit();
        public Unit Y = new Unit();
        public Unit Z = new Unit();
        public Angle Rx = new Angle();
        public Angle Ry = new Angle();
        public Angle Rz = new Angle();
        public byte Grip = 0;


        public URCoordinates(Unit _x, Unit _y, Unit _z, Angle _Rx, Angle _Ry, Angle _Rz, byte _G = 0)
        {
            X = _x;
            Y = _y;
            Z = _z;
            Rx = _Rx;
            Ry = _Ry;
            Rz = _Rz;
            Grip = _G;
        }
        /// <summary>
        /// Unit meter
        /// </summary>
        /// <param name="_x">axis x (unit meter)</param>
        /// <param name="_y">axis y (unit meter)</param>
        /// <param name="_z">axis z (unit meter)</param>
        /// <param name="_Rx">axis Rx (unit meter)</param>
        /// <param name="_Ry">axis Ry (unit meter)</param>
        /// <param name="_Rz">axis Rz (unit meter)</param>
        /// <param name="_G">gripper pos (0~255)</param>
        public URCoordinates(float _x, float _y, float _z, float _Rx, float _Ry, float _Rz, byte _G = 0, string unit = "M")
        {
            if (unit == "M")
            {
                X = new Unit(_x);
                Y = new Unit(_y);
                Z = new Unit(_z);
                Rx = new Angle(_Rx);
                Ry = new Angle(_Ry);
                Rz = new Angle(_Rz);
                Grip = _G;
            }
        }
        public URCoordinates(double _x, double _y, double _z, double _Rx, double _Ry, double _Rz, byte _G = 0, string unit = "M")
        {
            if (unit == "M")
            {
                X = new Unit(_x);
                Y = new Unit(_y);
                Z = new Unit(_z);
                Rx = new Angle(_Rx);
                Ry = new Angle(_Ry);
                Rz = new Angle(_Rz);
                Grip = _G;
            }
        }
        public URCoordinates(URCoordinates input)
        {
            X = input.X;
            Y = input.Y;
            Z = input.Z;
            Rx = input.Rx;
            Ry = input.Ry;
            Rz = input.Rz;
        }

        /// <summary>
        /// e.g.
        /// "p[]" → p[X,Y,Z,Rx,Ry,Rz]
        /// "[]" → [X,Y,Z,Rx,Ry,Rz]
        /// "()" → (X,Y,Z,Rx,Ry,Rz)
        /// "(3)" → (X,Y,Z)
        /// "[7]" → [X,Y,Z,Rx,Ry,Rz,Grip]
        /// </summary>
        /// <param name="format"></param>
        /// <returns></returns>
        public string ToString(string format = "[]")
        {
            string rtn = "";
            if (format.IndexOf('p') >= 0)
                rtn += "p";
            if (format.IndexOf('[') >= 0)
                rtn += "[";
            if (format.IndexOf('(') >= 0)
                rtn += "(";

            if (format.IndexOf('3') >= 0)
                rtn += $"{X.M},{Y.M},{Z.M}";
            else if (format.IndexOf('6') >= 0)
                rtn += $"{X.M},{Y.M},{Z.M},{Rx.rad},{Ry.rad},{Rz.rad}";
            else if (format.IndexOf('7') >= 0)
                rtn += $"{X.M},{Y.M},{Z.M},{Rx.rad},{Ry.rad},{Rz.rad},{Grip}";
            else
                rtn += $"{X.M},{Y.M},{Z.M},{Rx.rad},{Ry.rad},{Rz.rad}";

            if (format.IndexOf(']') >= 0)
                rtn += "]";
            if (format.IndexOf(')') >= 0)
                rtn += ")";

            return rtn;

        }
    }
}
