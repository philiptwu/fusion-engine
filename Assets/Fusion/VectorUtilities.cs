using System;
using MathNet.Numerics.LinearAlgebra;
#if(UNITY_STANDALONE)
using UnityEngine;
#endif

namespace AutonomyTestbed.Fusion
{
    public static class VectorUtilities
    {
        // Resizes vector v to length desiredLength, pads with 0's if necessary
        public static Vector Resize(Vector v, int desiredLength)
        {
            // Allocate blank vector initialized with 0's
            Vector y = new Vector(desiredLength, 0.0f);

            // Copy over min of either original vector length or desired length
            int minLength = (desiredLength < v.Length) ? desiredLength : v.Length;
            for (int i = 0; i < minLength; i++)
            {
                y[i] = v[i];
            }

            // Return to caller
            return y;
        }

        // Resizes matrix m to length dimensions, pads with 0's if necessary
        public static Matrix Resize(Matrix m, int desiredNumRows, int desiredNumCols)
        {
            // Allocate blank matrix initialized with 0's
            Matrix y = new Matrix(desiredNumRows, desiredNumCols, 0.0f);

            // Copy over min of either original vector length or desired length
            int minNumRows = (desiredNumRows < m.RowCount) ? desiredNumRows : m.RowCount;
            int minNumCols = (desiredNumCols < m.ColumnCount) ? desiredNumCols : m.ColumnCount;
            for (int i = 0; i < minNumRows; i++)
            {
                for (int j = 0; j < minNumCols; j++)
                {
                    y[i,j] = m[i,j];
                }
            }
            // Return to caller
            return y;
        }

    }
}
