using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    public static class UnityToEnu
    {
        public static Vector ConvertVector(Vector unity, Vector unityReference)
        {
            if (unity.Length == 3)
            {
                Vector enu = new Vector(3);
                enu[0] = unity[0] - unityReference[0];
                enu[1] = unity[2] - unityReference[2]; // Purposely swapped 1 and 2 on ENU
                enu[2] = unity[1] - unityReference[1]; // Purposely swapped 1 and 2 on ENU
                return unity;
            }
            else if (unity.Length == 6)
            {
                Vector enu = new Vector(6);
                enu[0] = unity[0] - unityReference[0];
                enu[1] = unity[2] - unityReference[2]; // Purposely swapped 1 and 2 on ENU
                enu[2] = unity[1] - unityReference[1]; // Purposely swapped 1 and 2 on ENU
                enu[3] = unity[3] - unityReference[3];
                enu[4] = unity[5] - unityReference[5]; // Purposely swapped 4 and 5 on ENU
                enu[5] = unity[4] - unityReference[4]; // Purposely swapped 4 and 5 on ENU
                return unity;
            }
            else
            {
                // Unsupported size
                #if(UNITY_STANDALONE)
                Debug.LogError("UnityToEnu::ConvertVector(): Unsupported input size");
                #else
                Console.WriteLine("UnityToEnu::ConvertVector(): Unsupported input size");
                #endif
                return null;
            }
        }

        public static Matrix ComputeJacobian(Vector unity)
        {
            if (unity.Length == 3)
            {
                // Create and return
                Matrix J = new Matrix(3, 3, 0);
                J[0, 0] = 1;
                J[1, 2] = 1;
                J[2, 1] = 1;
                return J;
            }
            else if (unity.Length == 6)
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
                Debug.LogError("UnityToEnu::ComputeJacobian(): Unsupported input size");
                #else
                Console.WriteLine("UnityToEnu::ComputeJacobian(): Unsupported input size");
                #endif
                return null;
            }
        }
    }
}
