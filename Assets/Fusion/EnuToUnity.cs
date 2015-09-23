using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    public static class EnuToUnity
    {
        public static Vector ConvertVector(Vector enu, Vector unityReference)
        {
            if (enu.Length == 3)
            {
                Vector unity = new Vector(3);
                unity[0] = enu[0] + unityReference[0];
                unity[1] = enu[2] + unityReference[1]; // Purposely swapped 1 and 2 on ENU
                unity[2] = enu[1] + unityReference[2]; // Purposely swapped 1 and 2 on ENU
                return unity;
            }
            else if (enu.Length == 6)
            {
                Vector unity = new Vector(6);
                unity[0] = enu[0] + unityReference[0];
                unity[1] = enu[2] + unityReference[1]; // Purposely swapped 1 and 2 on ENU
                unity[2] = enu[1] + unityReference[2]; // Purposely swapped 1 and 2 on ENU
                unity[3] = enu[3] + unityReference[3];
                unity[4] = enu[5] + unityReference[4]; // Purposely swapped 4 and 5 on ENU
                unity[5] = enu[4] + unityReference[5]; // Purposely swapped 4 and 5 on ENU
                return unity;
            }
            else
            {
                // Unsupported size
                #if(UNITY_STANDALONE)
                Debug.LogError("EnuToUnity::ConvertVector(): Unsupported input size");
                #else
                Console.WriteLine("EnuToUnity::ConvertVector(): Unsupported input size");
                #endif
                return null;
            }
        }

        public static Matrix ComputeJacobian(Vector enu)
        {
            if (enu.Length == 3)
            {
                // Create and return
                Matrix J = new Matrix(3, 3, 0);
                J[0, 0] = 1;
                J[1, 2] = 1;
                J[2, 1] = 1;
                return J;
            }
            else if (enu.Length == 6)
            {
                Matrix J = new Matrix(6, 6, 0);
                J[0, 0] = 1;
                J[1, 2] = 1;
                J[2, 1] = 1;
                J[3, 3] = 1;
                J[4, 5] = 1;
                J[5, 4] = 1;
                return J;
            }
            else
            {
                // Unsupported size
                #if(UNITY_STANDALONE)
                Debug.LogError("EnuToUnity::ComputeJacobian(): Unsupported input size");
                #else
                Console.WriteLine("EnuToUnity::ComputeJacobian(): Unsupported input size");
                #endif
                return null;
            }
        }
    }
}
