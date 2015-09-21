using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    public static class RbeToEnu
    {
        public static Vector ConvertVector(Vector rbe)
        {
            if (rbe.Length == 3)
            {
                // Extract
                double r = rbe[0];
                double b = rbe[1];
                double e = rbe[2];

                // Precompute
                double cosb = Math.Cos(b);
                double sinb = Math.Sin(b);
                double cose = Math.Cos(e);
                double sine = Math.Sin(e);

                // Create and return
                Vector enu = new Vector(3);
                enu[0] = r * cose * sinb;
                enu[1] = r * cose * cosb;
                enu[2] = r * sine;
                return enu;
            }
            else if (rbe.Length == 6)
            {
                // Extract
                double r = rbe[0];
                double b = rbe[1];
                double e = rbe[2];
                double rdot = rbe[3];
                double bdot = rbe[4];
                double edot = rbe[5];

                // Precompute
                double cosb = Math.Cos(b);
                double sinb = Math.Sin(b);
                double cose = Math.Cos(e);
                double sine = Math.Sin(e);

                // Create and return
                Vector enu = new Vector(6);
                enu[0] = r * cose * sinb;
                enu[1] = r * cose * cosb;
                enu[2] = r * sine;
                enu[3] = rdot * cose * sinb - r * edot * sine * sinb + r * bdot * cose * cosb;
                enu[4] = rdot * cose * cosb - r * edot * sine * cosb - r * bdot * cose * sinb;
                enu[5] = rdot * sine + r * edot * cose;
                return enu;
            }
            else
            {
                // Unsupported size
                return null;
            }
        }

        public static Matrix ComputeJacobian(Vector rbe)
        {
            if (rbe.Length == 3)
            {
                // Extract
                double r = rbe[0];
                double b = rbe[1];
                double e = rbe[2];

                // Precompute
                double cosb = Math.Cos(b);
                double sinb = Math.Sin(b);
                double cose = Math.Cos(e);
                double sine = Math.Sin(e);

                // Create and return
                Matrix J = new Matrix(3, 3);
                J[0, 0] = cose * sinb;
                J[0, 1] = r * cosb * cose;
                J[0, 2] = -r * sinb * sine;

                J[1, 0] = cosb * cose;
                J[1, 1] = -r * cose * sinb;
                J[1, 2] = -r * cosb * sine;

                J[2, 0] = sine;
                J[2, 1] = 0;
                J[2, 2] = r * cose;
                return J;
            }
            else if (rbe.Length == 6)
            {
                // Extract
                double r = rbe[0];
                double b = rbe[1];
                double e = rbe[2];
                double rdot = rbe[3];
                double bdot = rbe[4];
                double edot = rbe[5];

                // Precompute
                double cosb = Math.Cos(b);
                double sinb = Math.Sin(b);
                double cose = Math.Cos(e);
                double sine = Math.Sin(e);

                // Create and return
                Matrix J = new Matrix(6, 6);
                J[0, 0] = cose * sinb;
                J[0, 1] = r * cosb * cose;
                J[0, 2] = -r * sinb * sine;
                J[0, 3] = 0;
                J[0, 4] = 0;
                J[0, 5] = 0;

                J[1, 0] = cosb * cose;
                J[1, 1] = -r * cose * sinb;
                J[1, 2] = -r * cosb * sine;
                J[1, 3] = 0;
                J[1, 4] = 0;
                J[1, 5] = 0;

                J[2, 0] = sine;
                J[2, 1] = 0;
                J[2, 2] = r * cose;
                J[2, 3] = 0;
                J[2, 4] = 0;
                J[2, 5] = 0;

                J[3, 0] = bdot * cosb * cose - edot * sinb * sine;
                J[3, 1] = rdot * cosb * cose - bdot * r * cose * sinb - edot * r * cosb * sine;
                J[3, 2] = -rdot * sinb * sine - bdot * r * cosb * sine - edot * r * cose * sinb;
                J[3, 3] = cose * sinb;
                J[3, 4] = r * cosb * cose;
                J[3, 5] = -r * sinb * sine;

                J[4, 0] = -bdot * cose * sinb - edot * cosb * sine;
                J[4, 1] = edot * r * sinb * sine - bdot * r * cosb * cose - rdot * cose * sinb;
                J[4, 2] = bdot * r * sinb * sine - edot * r * cosb * cose - rdot * cosb * sine;
                J[4, 3] = cosb * cose;
                J[4, 4] = -r * cose * sinb;
                J[4, 5] = -r * cosb * sine;

                J[5, 0] = edot * cose;
                J[5, 1] = 0;
                J[5, 2] = rdot * cose - edot * r * sine;
                J[5, 3] = sine;
                J[5, 4] = 0;
                J[5, 5] = r * cose;

                return J;
            }
            else
            {
                // Unsupported size
                return null;
            }
        }
    }
}
