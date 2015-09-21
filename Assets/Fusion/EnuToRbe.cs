using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    public static class EnuToRbe
    {
        public static Vector ConvertVector(Vector enu)
        {
            if (enu.Length == 3)
            {
                // Extract
                double e = enu[0];
                double n = enu[1];
                double u = enu[2];

                // Precompute
                double e2 = e*e;
                double n2 = n * n;
                double u2 = u * u;
                double gr2 = e2 + n2;
                double gr = Math.Sqrt(gr2);

                // Create and return
                Vector rbe = new Vector(3);
                rbe[0] = (float)Math.Sqrt(gr2 + u2);
                rbe[1] = (float)Math.Atan2(e, n);
                rbe[2] = (float)Math.Atan2(u, gr);
                return rbe;
            }
            else if (enu.Length == 6)
            {
                // Extract
                double e = enu[0];
                double n = enu[1];
                double u = enu[2];
                double edot = enu[3];
                double ndot = enu[4];
                double udot = enu[5];

                // Precompute
                double e2 = e * e;
                double n2 = n * n;
                double u2 = u * u;
                double gr2 = e2 + n2;
                double gr = Math.Sqrt(gr2);

                // Create and return
                Vector rbe = new Vector(6);
                rbe[0] = Math.Sqrt(gr2 + u2);
                rbe[1] = Math.Atan2(e, n);
                rbe[2] = Math.Atan2(u, gr);
                rbe[3] = (e*edot + n*ndot + u*udot)/Math.Sqrt(gr2 + u2);
                rbe[4] = (edot*n - ndot*e)/gr2;
                rbe[5] = (udot*gr - (n*ndot + e*edot)*u/gr)/(gr2 + u2);   
                return rbe;
            }
            else
            {
                // Unsupported size
                return null;
            }
        }

        public static Matrix ComputeJacobian(Vector enu)
        {
            if (enu.Length == 3)
            {
                // Extract
                double e = enu[0];
                double n = enu[1];
                double u = enu[2];

                // Precompute
                double e2 = e * e;
                double n2 = n * n;
                double u2 = u * u;
                double gr2 = e2 + n2;
                double gr = Math.Sqrt(gr2);
                double r2 = gr2 + u2;
                double r = Math.Sqrt(r2);

                // Create and return
                Matrix J = new Matrix(3,3);
                J[0, 0] = e / r;
                J[0, 1] = n / r;
                J[0, 2] = u / r;

                J[1, 0] = n / gr2;
                J[1, 1] = -e / gr2;
                J[1, 2] = 0;

                J[2, 0] = -e * u / (gr * r2);
                J[2, 1] = -n * u / (gr * r2);
                J[2, 2] = gr / r2;
                return J;
            }
            else if (enu.Length == 6)
            {
                // Extract
                double e = enu[0];
                double n = enu[1];
                double u = enu[2];
                double edot = enu[3];
                double ndot = enu[4];
                double udot = enu[5];

                // Precompute
                double e2 = e * e;
                double n2 = n * n;
                double u2 = u * u;
                double gr2 = e2 + n2;
                double gr = Math.Sqrt(gr2);
                double r2 = gr2 + u2;
                double r = Math.Sqrt(r2);

                // Create and return
                Matrix J = new Matrix(6, 6);
                J[0, 0] = e / r;
                J[0, 1] = n / r;
                J[0, 2] = u / r;
                J[0, 3] = 0;
                J[0, 4] = 0;
                J[0, 5] = 0;

                J[1, 0] = n / gr2;
                J[1, 1] = -e / gr2;
                J[1, 2] = 0;
                J[1, 3] = 0;
                J[1, 4] = 0;
                J[1, 5] = 0;

                J[2, 0] = -e * u / (gr * r2);
                J[2, 1] = -n * u / (gr * r2);
                J[2, 2] = gr / r2;
                J[2, 3] = 0;
                J[2, 4] = 0;
                J[2, 5] = 0;

                J[3, 0] = (edot*n2 - e*ndot*n + edot*u2 - e*udot*u)/(r*r2);
                J[3, 1] = (ndot*e2 - edot*n*e + ndot*u2 - n*udot*u)/(r*r2);
                J[3, 2] = (udot*e2 - edot*u*e + udot*n2 - ndot*u*n)/(r*r2);
                J[3, 3] = e/r;
                J[3, 4] = n/r;
                J[3, 5] = u/r;

                J[4, 0] = -(- ndot*e2 + 2*edot*e*n + ndot*n2)/(gr2*gr2);
                J[4, 1] = (edot*e2 + 2*ndot*e*n - edot*n2)/(gr2*gr2);
                J[4, 2] = 0;
                J[4, 3] = n/gr2;
                J[4, 4] = -e/gr2;
                J[4, 5] = 0;

                J[5, 0] = ((e*udot)/gr - (edot*u)/gr + (e*u*(e*edot + n*ndot))/(gr*gr2))/r2 - (2*e*(udot*gr - (u*(e*edot + n*ndot))/gr))/(r2*r2);
                J[5, 1] = ((n*udot)/gr - (ndot*u)/gr + (n*u*(e*edot + n*ndot))/(gr*gr2))/r2 - (2*n*(udot*gr - (u*(e*edot + n*ndot))/gr))/(r2*r2);
                J[5, 2] = -(edot*e*e2 + ndot*e2*n + 2*udot*e2*u + edot*e*n2 - edot*e*u2 + ndot*n*n2 + 2*udot*n2*u - ndot*n*u2)/(gr*r2*r2);
                J[5, 3] = -(e*u)/(gr*r2);
                J[5, 4] = -(n*u)/(gr*r2);
                J[5, 5] = gr / r2;

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
