using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UrRobot.Coordinates
{
    static class ex
    {
        static public Unit M(this double value)
        {
            return new Unit(value);
        }
    }
    public class Unit
    {
        double _M = 0;
        public Unit(double v)
        {
            _M = v;
        }
        public Unit()
        {
            _M = 0;
        }

        public double M
        {
            get { return _M; }
            set { _M = value; }
        }
        public double mm
        {
            get { return _M * 1000.0f; }
            set { _M = value / 1000.0f; }
        }
    }

    public class URCoordinates
    {
       public Unit X = new Unit();
        public Unit Y = new Unit();
        public Unit Z = new Unit();
        public Unit Rx = new Unit();
        public Unit Ry = new Unit();
        public Unit Rz = new Unit();
        public byte Grip = 0;


        public URCoordinates(Unit _x, Unit _y, Unit _z, Unit _Rx, Unit _Ry, Unit _Rz, byte _G = 0)
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
        public URCoordinates(double _x, double _y, double _z, double _Rx, double _Ry, double _Rz, byte _G = 0)
        {
            X = new Unit(_x);
            Y = new Unit(_y);
            Z = new Unit(_z);
            Rx = new Unit(_Rx);
            Ry = new Unit(_Ry);
            Rz = new Unit(_Rz);
            Grip = _G;
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
            else if(format.IndexOf('6') >= 0)
                rtn += $"{X.M},{Y.M},{Z.M},{Rx.M},{Ry.M},{Rz.M}";
            else if (format.IndexOf('7') >= 0)
                rtn += $"{X.M},{Y.M},{Z.M},{Rx.M},{Ry.M},{Rz.M},{Grip}";
            else
                rtn += $"{X.M},{Y.M},{Z.M},{Rx.M},{Ry.M},{Rz.M}";

            if (format.IndexOf(']') >= 0)
                rtn += "]";
            if (format.IndexOf(')') >= 0)
                rtn += ")";

            return rtn;

        }
        //public string ToString(string unit = "m", string type = "[", string format = null)
        //{
        //    if (unit == "mm")
        //    {
        //        X *= 1000f;
        //        Y *= 1000f;
        //        Z *= 1000f;
        //    }
        //    string x = X.ToString(format);
        //    string y = Y.ToString(format);
        //    string z = Z.ToString(format);
        //    string rx = Rx.ToString(format);
        //    string ry = Ry.ToString(format);
        //    string rz = Rz.ToString(format);
        //    if (type == "[")
        //        return $"[{x},{y},{z},{rx},{ry},{rz}]";
        //    else if (type == "(")
        //        return $"({x},{y},{z},{rx},{ry},{rz})";
        //    else if (type == "3(")
        //        return $"({x},{y},{z})";
        //    else if (type == "3[")
        //        return $"[{x},{y},{z}]";
        //    else
        //        return $"[{x},{y},{z},{rx},{ry},{rz}]";
        //}

    }
}
