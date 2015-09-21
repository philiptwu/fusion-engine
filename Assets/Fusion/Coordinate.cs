using System;
using MathNet.Numerics.LinearAlgebra;

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

        public static void Convert(Type fromType, Vector fromVector, Type toType, out Vector toVector)
        {
            // Convert the vector
            switch (fromType)
            {
                case Type.ENU:
                    toVector = ConvertENUVector(fromVector, toType);
                    break;
                case Type.RBE:
                    toVector = ConvertRBEVector(fromVector, toType);
                    break;
                default:
                    toVector = null;
                    break;
            }
        }

        public static void Convert(Type fromType, Vector fromVector, Matrix fromCovariance, Type toType, out Vector toVector, out Matrix toCovariance, out Matrix fromToJacobian)
        {
            // First convert the vector
            Convert(fromType, fromVector, toType, out toVector);

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


        #region Vector Conversion
        private static Vector ConvertENUVector(Vector fromVector, Type toType)
        {
            // Switch on to type
            switch (toType)
            {
                case Type.ENU:
                    return fromVector;
                case Type.RBE:
                    return EnuToRbe.ConvertVector(fromVector); 
                default:
                    return null;
            }
        }

        private static Vector ConvertRBEVector(Vector fromVector, Type toType)
        {
            // Switch on to type
            switch (toType)
            {
                case Type.ENU:
                    return RbeToEnu.ConvertVector(fromVector);
                case Type.RBE:
                    return fromVector;
                default:
                    return null;
            }
        }
        #endregion

        #region Jacobian Computation
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

    }
}
