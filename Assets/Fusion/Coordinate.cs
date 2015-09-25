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
            UNITY3,
            UNITY6,
            ENU3,
            ENU6,
            RBE3,
            RBE6
        };

        // This function converts vectors between locations and coordiante frames
        public static void Convert(Vector fromReferenceUnity6, Type fromType, Vector fromVector,
            Vector toReferenceUnity6, Type toType, out Vector toVector, out Matrix fromToJacobian)
        {
            // Assign default values to output
            toVector = null;
            fromToJacobian = null;

            // Convert the vector
            switch (fromType)
            {
                case Type.UNITY3:
                    #region From UNITY3
                    switch (toType)
                    {
                        case Type.UNITY3: // UNITY3 to UNITY3
                            {
                                toVector = VectorUtilities.Resize(fromVector, 3);
                                fromToJacobian = Matrix.Identity(3, 3);
                                break;
                            }
                        case Type.UNITY6: // UNITY3 to UNITY6
                            {
                                // Unsupported
                                break;
                            }
                        case Type.ENU3: // UNITY3 to ENU3
                            {
                                toVector = UnityToEnu.ConvertVector(fromVector, toReferenceUnity6, 3);
                                fromToJacobian = UnityToEnu.ComputeJacobian(fromVector, 3);
                                break;
                            }
                        case Type.ENU6: // UNITY3 to ENU6
                            {
                                // Unsupported
                                break;
                            }
                        case Type.RBE3: // UNITY3 to RBE3
                            {
                                Vector toEnu = UnityToEnu.ConvertVector(fromVector, toReferenceUnity6, 3);
                                toVector = EnuToRbe.ConvertVector(toEnu, 3);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu, 3) * UnityToEnu.ComputeJacobian(fromVector, 3);
                                break;
                            }
                        case Type.RBE6: // UNITY3 to RBE6
                            {
                                // Unsupported
                                break;
                            }
                    }
                    #endregion
                    break;
                case Type.UNITY6:
                    #region From UNITY6
                    switch (toType)
                    {
                        case Type.UNITY3: // UNITY6 to UNITY3
                            {
                                toVector = VectorUtilities.Resize(fromVector, 3);
                                fromToJacobian = Matrix.Identity(3, 3);
                                fromToJacobian = VectorUtilities.Resize(fromToJacobian, 3, 6);
                                break;
                            }
                        case Type.UNITY6: // UNITY6 to UNITY6
                            {
                                toVector = VectorUtilities.Resize(fromVector, 6);
                                fromToJacobian = Matrix.Identity(6, 6);
                                break;
                            }
                        case Type.ENU3: // UNITY6 to ENU3
                            {
                                toVector = UnityToEnu.ConvertVector(fromVector, toReferenceUnity6, 3);
                                fromToJacobian = UnityToEnu.ComputeJacobian(fromVector, 3);
                                fromToJacobian = VectorUtilities.Resize(fromToJacobian, 3, 6);
                                break;
                            }
                        case Type.ENU6: // UNITY6 to ENU6
                            {
                                toVector = UnityToEnu.ConvertVector(fromVector, toReferenceUnity6, 6);
                                fromToJacobian = UnityToEnu.ComputeJacobian(fromVector, 6);
                                break;
                            }
                        case Type.RBE3: // UNITY6 to RBE3
                            {
                                Vector toEnu = UnityToEnu.ConvertVector(fromVector, toReferenceUnity6, 3);
                                toVector = EnuToRbe.ConvertVector(toEnu, 3);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu, 3) * UnityToEnu.ComputeJacobian(fromVector, 3);
                                fromToJacobian = VectorUtilities.Resize(fromToJacobian, 3, 6);
                                break;
                            }
                        case Type.RBE6: // UNITY6 to RBE6
                            {
                                Vector toEnu = UnityToEnu.ConvertVector(fromVector, toReferenceUnity6, 6);
                                toVector = EnuToRbe.ConvertVector(toEnu, 6);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu, 6) * UnityToEnu.ComputeJacobian(fromVector, 6);
                                break;
                            }
                    }
                    #endregion
                    break;
                case Type.ENU3:
                    #region From ENU3
                    switch (toType)
                    {
                        case Type.UNITY3: // ENU3 to UNITY3
                            {
                                toVector = EnuToUnity.ConvertVector(fromVector, fromReferenceUnity6, 3);
                                fromToJacobian = EnuToUnity.ComputeJacobian(fromVector, 3);
                                break;
                            }
                        case Type.UNITY6: // ENU3 to UNITY6
                            {
                                // Unsupported
                                break;
                            }
                        case Type.ENU3: // ENU3 to ENU3
                            {
                                Vector unity = EnuToUnity.ConvertVector(fromVector, fromReferenceUnity6, 3);
                                toVector = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 3);
                                fromToJacobian = UnityToEnu.ComputeJacobian(unity, 3) * EnuToUnity.ComputeJacobian(fromVector, 3);
                                break;
                            }
                        case Type.ENU6: // ENU3 to ENU6
                            {
                                // Unsupported
                                break;
                            }
                        case Type.RBE3: // ENU3 to RBE3
                            {
                                Vector unity = EnuToUnity.ConvertVector(fromVector, fromReferenceUnity6, 3);
                                Vector toEnu = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 3);
                                toVector = EnuToRbe.ConvertVector(toEnu, 3);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu, 3) * UnityToEnu.ComputeJacobian(unity, 3) * EnuToUnity.ComputeJacobian(fromVector, 3);
                                break;
                            }
                        case Type.RBE6: // ENU3 to RBE6
                            {
                                // Unsupported
                                break;
                            }
                    }
                    #endregion
                    break;
                case Type.ENU6:
                    #region From ENU6
                    switch (toType)
                    {
                        case Type.UNITY3: // ENU6 to UNITY3
                            {
                                toVector = EnuToUnity.ConvertVector(fromVector, fromReferenceUnity6, 3);
                                fromToJacobian = EnuToUnity.ComputeJacobian(fromVector, 3);
                                fromToJacobian = VectorUtilities.Resize(fromToJacobian, 3, 6);
                                break;
                            }
                        case Type.UNITY6: // ENU6 to UNITY6
                            {
                                toVector = EnuToUnity.ConvertVector(fromVector, fromReferenceUnity6, 6);
                                fromToJacobian = EnuToUnity.ComputeJacobian(fromVector, 6);
                                break;
                            }
                        case Type.ENU3: // ENU6 to ENU3
                            {
                                Vector unity = EnuToUnity.ConvertVector(fromVector, fromReferenceUnity6, 3);
                                toVector = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 3);
                                fromToJacobian = UnityToEnu.ComputeJacobian(unity, 3) * EnuToUnity.ComputeJacobian(fromVector, 3);
                                fromToJacobian = VectorUtilities.Resize(fromToJacobian, 3, 6);
                                break;
                            }
                        case Type.ENU6: // ENU6 to ENU6
                            {
                                Vector unity = EnuToUnity.ConvertVector(fromVector, fromReferenceUnity6, 6);
                                toVector = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 6);
                                fromToJacobian = UnityToEnu.ComputeJacobian(unity, 6) * EnuToUnity.ComputeJacobian(fromVector, 6);
                                break;
                            }
                        case Type.RBE3: // ENU6 to RBE3
                            {
                                Vector unity = EnuToUnity.ConvertVector(fromVector, fromReferenceUnity6, 3);
                                Vector toEnu = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 3);
                                toVector = EnuToRbe.ConvertVector(toEnu, 3);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu, 3) * UnityToEnu.ComputeJacobian(unity, 3) * EnuToUnity.ComputeJacobian(fromVector, 3);
                                fromToJacobian = VectorUtilities.Resize(fromToJacobian, 3, 6);
                                break;
                            }
                        case Type.RBE6: // ENU6 to RBE6
                            {
                                Vector unity = EnuToUnity.ConvertVector(fromVector, fromReferenceUnity6, 6);
                                Vector toEnu = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 6);
                                toVector = EnuToRbe.ConvertVector(toEnu, 6);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu, 6) * UnityToEnu.ComputeJacobian(unity, 6) * EnuToUnity.ComputeJacobian(fromVector, 6);
                                break;
                            }
                    }
                    #endregion
                    break;
                case Type.RBE3:
                    #region From RBE3
                    switch (toType)
                    {
                        case Type.UNITY3: // RBE3 to UNITY3
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector, 3);
                                toVector = EnuToUnity.ConvertVector(fromEnu, fromReferenceUnity6, 3);
                                fromToJacobian = EnuToUnity.ComputeJacobian(fromEnu, 3) * RbeToEnu.ComputeJacobian(fromVector, 3);
                                break;
                            }
                        case Type.UNITY6: // RBE3 to UNITY6
                            {
                                // Unsupported
                                break;
                            }
                        case Type.ENU3: // RBE3 to ENU3
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector, 3);
                                Vector unity = EnuToUnity.ConvertVector(fromEnu, fromReferenceUnity6, 3);
                                toVector = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 3);
                                fromToJacobian = UnityToEnu.ComputeJacobian(unity, 3) * EnuToUnity.ComputeJacobian(fromEnu, 3) * RbeToEnu.ComputeJacobian(fromVector, 3);
                                break;
                            }
                        case Type.ENU6: // RBE3 to ENU6
                            {
                                // Unsupported
                                break;
                            }
                        case Type.RBE3: // RBE3 to RBE3
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector, 3);
                                Vector unity = EnuToUnity.ConvertVector(fromEnu, fromReferenceUnity6, 3);
                                Vector toEnu = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 3);
                                toVector = EnuToRbe.ConvertVector(toEnu, 3);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu, 3) * UnityToEnu.ComputeJacobian(unity, 3) * EnuToUnity.ComputeJacobian(fromEnu, 3) * RbeToEnu.ComputeJacobian(fromVector, 3);
                                break;
                            }
                        case Type.RBE6: // RBE3 to RBE6
                            {
                                // Unsupported
                                break;
                            }
                    }
                    #endregion
                    break;
                case Type.RBE6:
                    #region From RBE6
                    switch (toType)
                    {
                        case Type.UNITY3: // RBE6 to UNITY3
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector, 3);
                                toVector = EnuToUnity.ConvertVector(fromEnu, fromReferenceUnity6, 3);
                                fromToJacobian = EnuToUnity.ComputeJacobian(fromEnu, 3) * RbeToEnu.ComputeJacobian(fromVector, 3);
                                fromToJacobian = VectorUtilities.Resize(fromToJacobian, 3, 6);
                                break;
                            }
                        case Type.UNITY6: // RBE6 to UNITY6
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector, 6);
                                toVector = EnuToUnity.ConvertVector(fromEnu, fromReferenceUnity6, 6);
                                fromToJacobian = EnuToUnity.ComputeJacobian(fromEnu, 6) * RbeToEnu.ComputeJacobian(fromVector, 6);
                                break;
                            }
                        case Type.ENU3: // RBE6 to ENU3
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector, 3);
                                Vector unity = EnuToUnity.ConvertVector(fromEnu, fromReferenceUnity6, 3);
                                toVector = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 3);
                                fromToJacobian = UnityToEnu.ComputeJacobian(unity, 3) * EnuToUnity.ComputeJacobian(fromEnu, 3) * RbeToEnu.ComputeJacobian(fromVector, 3);
                                fromToJacobian = VectorUtilities.Resize(fromToJacobian, 3, 6);
                                break;
                            }
                        case Type.ENU6: // RBE6 to ENU6
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector, 6);
                                Vector unity = EnuToUnity.ConvertVector(fromEnu, fromReferenceUnity6, 6);
                                toVector = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 6);
                                fromToJacobian = UnityToEnu.ComputeJacobian(unity, 6) * EnuToUnity.ComputeJacobian(fromEnu, 6) * RbeToEnu.ComputeJacobian(fromVector, 6);
                                break;
                            }
                        case Type.RBE3: // RBE6 to RBE3
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector, 3);
                                Vector unity = EnuToUnity.ConvertVector(fromEnu, fromReferenceUnity6, 3);
                                Vector toEnu = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 3);
                                toVector = EnuToRbe.ConvertVector(toEnu, 3);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu, 3) * UnityToEnu.ComputeJacobian(unity, 3) * EnuToUnity.ComputeJacobian(fromEnu, 3) * RbeToEnu.ComputeJacobian(fromVector, 3);
                                fromToJacobian = VectorUtilities.Resize(fromToJacobian, 3, 6);
                                break;
                            }
                        case Type.RBE6: // RBE6 to RBE6
                            {
                                Vector fromEnu = RbeToEnu.ConvertVector(fromVector, 6);
                                Vector unity = EnuToUnity.ConvertVector(fromEnu, fromReferenceUnity6, 6);
                                Vector toEnu = UnityToEnu.ConvertVector(unity, toReferenceUnity6, 6);
                                toVector = EnuToRbe.ConvertVector(toEnu, 6);
                                fromToJacobian = EnuToRbe.ComputeJacobian(toEnu, 6) * UnityToEnu.ComputeJacobian(unity, 6) * EnuToUnity.ComputeJacobian(fromEnu, 6) * RbeToEnu.ComputeJacobian(fromVector, 6);
                                break;
                            }
                    }
                    #endregion
                    break;
            } // End switch(fromType)

            // Output error message if encountered unsupported conversion
            if (toVector == null || fromToJacobian == null)
            {
                // Set both to nulls
                toVector = null;
                fromToJacobian = null;

                // Give error message
                #if(UNITY_STANDALONE)
                Debug.LogError("Coordinate::Convert(): Coordinate conversion not supported, returning nulls");
                #else
                Console.WriteLine("Coordinate::Convert(): Coordinate conversion not supported, returning nulls");
                #endif
            }
        } // End public static void Convert(...)
    }
}
