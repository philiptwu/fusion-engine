using System;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

namespace AutonomyTestbed.Fusion
{
    public static class Coordinate
    {
        // Register types
        public enum Type
        {
            ENU,
            RBE
        };

        // This function converts vectors between locations and coordiante frames
        public static Vector Convert(GameObject fromLocation, Type fromType, Vector fromVector,
            GameObject toLocation, Type toType)
        {
            // Convert the vector
            switch (fromType)
            {
                case Type.ENU:
                    switch (toType)
                    {
                        case Type.ENU: // From ENU to ENU
                            return fromVector - ComputeENUDisplacement(fromLocation, toLocation);
                        case Type.RBE: // From ENU to RBE
                            return EnuToRbe.ConvertVector(fromVector - ComputeENUDisplacement(fromLocation, toLocation));
                        default: // Unrecognized
                            return null;
                    }
                case Type.RBE:
                    switch (toType)
                    {
                        case Type.ENU: // From RBE to ENU
                            return RbeToEnu.ConvertVector(fromVector) - ComputeENUDisplacement(fromLocation, toLocation);
                        case Type.RBE: // From RBE to RBE
                            return EnuToRbe.ConvertVector(RbeToEnu.ConvertVector(fromVector) - ComputeENUDisplacement(fromLocation, toLocation));
                        default: // Unrecognized
                            return null;
                    }
                default: // Unrecognized
                    return null;
            }
        }

        // This function converts 
        public static void Convert(GameObject fromLocation, Type fromType, Vector fromVector, Matrix fromCovariance,
            GameObject toLocation, Type toType, out Vector toVector, out Matrix toCovariance, out Matrix fromToJacobian)
        {
            // First convert the vector
            Convert(fromLocation, fromType, fromVector, toLocation, toType, out toVector);

            // Get the Jacobian
            switch (fromType)
            {
                case Type.ENU:
                    fromToJacobian = ComputeFromENUJacobian(fromVector, toType);
                    break;
                case Type.RBE:
                    fromToJacobian = ComputeFromRBEJacobian(fromVector, toType);
                    break;
                default:
                    fromToJacobian = null;
                    break;
            }

            // Convert covariance matrix
            Matrix fromToJacobianT = fromToJacobian.Clone();
            fromToJacobianT.Transpose();
            toCovariance = fromToJacobian * fromCovariance * fromToJacobianT;
        }

        #region Private Jacobian Computation
        private static Matrix ComputeFromENUJacobian(Vector fromVector, Type toType)
        {
            // Switch on to type
            switch (toType)
            {
                case Type.ENU:
                    return Matrix.Identity(fromVector.Length, fromVector.Length);
                case Type.RBE:
                    return EnuToRbe.ComputeJacobian(fromVector);
                default:
                    return null;
            }
        }

        private static Matrix ComputeFromRBEJacobian(Vector fromVector, Type toType)
        {
            // Switch on to type
            switch (toType)
            {
                case Type.ENU:
                    return RbeToEnu.ComputeJacobian(fromVector);
                case Type.RBE:
                    return Matrix.Identity(fromVector.Length, fromVector.Length);
                default:
                    return null;
            }
        }
        #endregion

        #region PrivateHelperFunctions
        // Note: Be very careful since for the first time we are relating Unity to measurement/track units/scales
        private static Vector ComputeENUDisplacement(GameObject fromLocation, GameObject toLocation)
        {
            // Compute displacement in Unity coordinates
            Vector3 displacementUnity = toLocation.transform.position - fromLocation.transform.position;

            // Populate a vector with displacement in ENU
            Vector displacementENU = new Vector(3);
            displacementENU[0] = displacementUnity.x;
            displacementENU[1] = displacementUnity.z; // Note: Y and Z purposely swapped
            displacementENU[2] = displacementUnity.y; // Note: Y and Z purposely swapped
            return displacementENU;
        }
        #endregion
    }
}
