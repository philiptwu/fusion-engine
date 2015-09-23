using System;
using MathNet.Numerics.LinearAlgebra;
#if(UNITY_STANDALONE)
using UnityEngine;
#endif

namespace AutonomyTestbed.Fusion
{
    public static class Coordinate
    {
        // Register types
        public enum Type
        {
            UNITY,
            ENU,
            RBE
        };

        // This function converts vectors between locations and coordiante frames
        public static void Convert(Vector fromUnityReference, Type fromType, Vector fromVector,
            Vector toUnityReference, Type toType, out Vector toVector, out Matrix fromToJacobian)
        {
            // Assign default values to output
            toVector = null;
            fromToJacobian = null;
            
            // Determine length of input vector
            int N = fromVector.Length;
            N = (fromUnityReference.Length < N) ? fromUnityReference.Length : N;
            N = (toUnityReference.Length < N) ? toUnityReference.Length : N;
            if (N != 3 && N != 6)
            {
                #if(UNITY_STANDALONE)
                Debug.LogError("Coordinate::Convert(): Length of input vector not supported, cannot convert");
                return;
                #else
                Console.WriteLine("Coordinate::Convert(): Length of input vector not supported, cannot convert");
                return;
                #endif
            }
            else
            {
                // Make copies of vectors truncated to minimum length N
                Vector temp = new Vector(N);
                for (int i = 0; i < N; i++)
                {
                    temp[i] = fromUnityReference[i];
                }
                fromUnityReference = temp;
                temp = new Vector(N);
                for (int i = 0; i < N; i++)
                {
                    temp[i] = fromVector[i];
                }
                fromVector = temp;
                temp = new Vector(N);
                for (int i = 0; i < N; i++)
                {
                    temp[i] = toUnityReference[i];
                }
                toUnityReference = temp;
            }

            // Convert the vector
            switch (fromType)
            {
                case Type.UNITY:
                    switch (toType)
                    {
                        case Type.UNITY: // UNITY to UNITY
                            {
                                toVector = fromVector.Clone();
                                fromToJacobian = Matrix.Identity(N, N);
                                break;
                            }
                        case Type.ENU: // UNITY to ENU
                            {
                                toVector = UnityToEnu.ConvertVector(fromVector, toUnityReference);
                                fromToJacobian = UnityToEnu.ComputeJacobian(fromVector);
                                break;
                            }
                        case Type.RBE: // UNITY to RBE
                            {
                                Vector toEnu = UnityToEnu.ConvertVector(fromVector, toUnityReference);
                                toVector = EnuToRbe.ConvertVector(toEnu);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu) * UnityToEnu.ComputeJacobian(fromVector);
                                break;
                            }
                    }
                    break;
                case Type.ENU:
                    switch (toType)
                    {
                        case Type.UNITY: // ENU to UNITY
                            {
                                toVector = EnuToUnity.ConvertVector(fromVector, fromUnityReference);
                                fromToJacobian = EnuToUnity.ComputeJacobian(fromVector);
                                break;
                            }
                        case Type.ENU: // ENU to ENU
                            {
                                Vector unity = EnuToUnity.ConvertVector(fromVector, fromUnityReference);
                                toVector = UnityToEnu.ConvertVector(unity, toUnityReference);
                                fromToJacobian = UnityToEnu.ComputeJacobian(unity) * EnuToUnity.ComputeJacobian(fromVector);
                                break;
                            }
                        case Type.RBE: // ENU to RBE
                            {
                                Vector unity = EnuToUnity.ConvertVector(fromVector, fromUnityReference);
                                Vector toEnu = UnityToEnu.ConvertVector(unity, toUnityReference);
                                toVector = EnuToRbe.ConvertVector(toEnu);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu) * UnityToEnu.ComputeJacobian(unity) * EnuToUnity.ComputeJacobian(fromVector);
                                break;
                            }
                    }
                    break;
                case Type.RBE:
                    switch (toType)
                    {
                        case Type.UNITY: // RBE to UNITY
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector);
                                toVector = EnuToUnity.ConvertVector(fromEnu, fromUnityReference);
                                fromToJacobian = EnuToUnity.ComputeJacobian(fromEnu) * RbeToEnu.ComputeJacobian(fromVector);
                                break;
                            }
                        case Type.ENU: // RBE to ENU
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector);
                                Vector unity = EnuToUnity.ConvertVector(fromEnu, fromUnityReference);
                                toVector = UnityToEnu.ConvertVector(unity, toUnityReference);
                                fromToJacobian = UnityToEnu.ComputeJacobian(unity) * EnuToUnity.ComputeJacobian(fromEnu) * RbeToEnu.ComputeJacobian(fromVector);
                                break;
                            }
                        case Type.RBE: // RBE to RBE
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector);
                                Vector unity = EnuToUnity.ConvertVector(fromEnu, fromUnityReference);
                                Vector toEnu = UnityToEnu.ConvertVector(unity, toUnityReference);
                                toVector = EnuToRbe.ConvertVector(toEnu);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu) * UnityToEnu.ComputeJacobian(unity) * EnuToUnity.ComputeJacobian(fromEnu) * RbeToEnu.ComputeJacobian(fromVector);
                                break;
                            }
                    }
                    break;
            }
        }
    }
}
