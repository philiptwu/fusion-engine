using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;

// Source:
// http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html

namespace AutonomyTestbed.Fusion
{
    public class HungarianAlgorithm
    {
        private int[,] C;
        private int[,] M;
        private int[,] path;
        private int[] RowCover;
        private int[] ColCover;
        private int nrow;
        private int ncol;
        private int path_count = 0;
        private int path_row_0;
        private int path_col_0;
        private int step;
        private bool debugMode;

        public HungarianAlgorithm()
        {
            debugMode = false;
        }

        public void SetDebugMode(bool debugMode)
        {
            this.debugMode = debugMode;
        }

        private void resetMaskandCovers()
        {
            M = new int[nrow, ncol];
            path = new int[nrow * ncol, 2];
            RowCover = new int[nrow];
            ColCover = new int[ncol];

            for (int r = 0; r < nrow; r++)
            {
                RowCover[r] = 0;
                for (int c = 0; c < ncol; c++)
                {
                    M[r, c] = 0;
                }
            }
            for (int c = 0; c < ncol; c++)
                ColCover[c] = 0;
        }

        //For each row of the cost matrix, find the smallest element and subtract
        //it from every element in its row.  When finished, Go to Step 2.
        private void step_one(ref int step)
        {
            int min_in_row;

            for (int r = 0; r < nrow; r++)
            {
                min_in_row = C[r, 0];
                for (int c = 0; c < ncol; c++)
                    if (C[r, c] < min_in_row)
                        min_in_row = C[r, c];
                for (int c = 0; c < ncol; c++)
                    C[r, c] -= min_in_row;
            }
            step = 2;
        }

        //Find a zero (Z) in the resulting matrix.  If there is no starred 
        //zero in its row or column, star Z. Repeat for each element in the 
        //matrix. Go to Step 3.
        private void step_two(ref int step)
        {
            for (int r = 0; r < nrow; r++)
                for (int c = 0; c < ncol; c++)
                {
                    if (C[r, c] == 0 && RowCover[r] == 0 && ColCover[c] == 0)
                    {
                        M[r, c] = 1;
                        RowCover[r] = 1;
                        ColCover[c] = 1;
                    }
                }
            for (int r = 0; r < nrow; r++)
                RowCover[r] = 0;
            for (int c = 0; c < ncol; c++)
                ColCover[c] = 0;
            step = 3;
        }

        //Cover each column containing a starred zero.  If K columns are covered, 
        //the starred zeros describe a complete set of unique assignments.  In this 
        //case, Go to DONE, otherwise, Go to Step 4.
        private void step_three(ref int step)
        {
            int colcount;
            for (int r = 0; r < nrow; r++)
                for (int c = 0; c < ncol; c++)
                    if (M[r, c] == 1)
                        ColCover[c] = 1;

            colcount = 0;
            for (int c = 0; c < ncol; c++)
                if (ColCover[c] == 1)
                    colcount += 1;
            if (colcount >= ncol || colcount >= nrow)
                step = 7;
            else
                step = 4;
        }

        //methods to support step 4
        private void find_a_zero(ref int row, ref int col)
        {
            int r = 0;
            int c;
            bool done;
            row = -1;
            col = -1;
            done = false;
            while (!done)
            {
                c = 0;
                while (true)
                {
                    if (C[r, c] == 0 && RowCover[r] == 0 && ColCover[c] == 0)
                    {
                        row = r;
                        col = c;
                        done = true;
                    }
                    c += 1;
                    if (c >= ncol || done)
                        break;
                }
                r += 1;
                if (r >= nrow)
                    done = true;
            }
        }

        private bool star_in_row(int row)
        {
            bool tmp = false;
            for (int c = 0; c < ncol; c++)
                if (M[row, c] == 1)
                    tmp = true;
            return tmp;
        }

        private void find_star_in_row(int row, ref int col)
        {
            col = -1;
            for (int c = 0; c < ncol; c++)
                if (M[row, c] == 1)
                    col = c;
        }

        //Find a noncovered zero and prime it.  If there is no starred zero 
        //in the row containing this primed zero, Go to Step 5.  Otherwise, 
        //cover this row and uncover the column containing the starred zero. 
        //Continue in this manner until there are no uncovered zeros left. 
        //Save the smallest uncovered value and Go to Step 6.
        private void step_four(ref int step)
        {
            int row = -1;
            int col = -1;
            bool done;

            done = false;
            while (!done)
            {
                find_a_zero(ref row, ref col);
                if (row == -1)
                {
                    done = true;
                    step = 6;
                }
                else
                {
                    M[row, col] = 2;
                    if (star_in_row(row))
                    {
                        find_star_in_row(row, ref col);
                        RowCover[row] = 1;
                        ColCover[col] = 0;
                    }
                    else
                    {
                        done = true;
                        step = 5;
                        path_row_0 = row;
                        path_col_0 = col;
                    }
                }
            }
        }

        // methods to support step 5
        private void find_star_in_col(int c, ref int r)
        {
            r = -1;
            for (int i = 0; i < nrow; i++)
                if (M[i, c] == 1)
                    r = i;
        }

        private void find_prime_in_row(int r, ref int c)
        {
            for (int j = 0; j < ncol; j++)
                if (M[r, j] == 2)
                    c = j;
        }

        private void augment_path()
        {
            for (int p = 0; p < path_count; p++)
                if (M[path[p, 0], path[p, 1]] == 1)
                    M[path[p, 0], path[p, 1]] = 0;
                else
                    M[path[p, 0], path[p, 1]] = 1;
        }

        private void clear_covers()
        {
            for (int r = 0; r < nrow; r++)
                RowCover[r] = 0;
            for (int c = 0; c < ncol; c++)
                ColCover[c] = 0;
        }

        private void erase_primes()
        {
            for (int r = 0; r < nrow; r++)
                for (int c = 0; c < ncol; c++)
                    if (M[r, c] == 2)
                        M[r, c] = 0;
        }


        //Construct a series of alternating primed and starred zeros as follows.  
        //Let Z0 represent the uncovered primed zero found in Step 4.  Let Z1 denote 
        //the starred zero in the column of Z0 (if any). Let Z2 denote the primed zero 
        //in the row of Z1 (there will always be one).  Continue until the series 
        //terminates at a primed zero that has no starred zero in its column.  
        //Unstar each starred zero of the series, star each primed zero of the series, 
        //erase all primes and uncover every line in the matrix.  Return to Step 3.
        private void step_five(ref int step)
        {
            bool done;
            int r = -1;
            int c = -1;

            path_count = 1;
            path[path_count - 1, 0] = path_row_0;
            path[path_count - 1, 1] = path_col_0;
            done = false;
            while (!done)
            {
                find_star_in_col(path[path_count - 1, 1], ref r);
                if (r > -1)
                {
                    path_count += 1;
                    path[path_count - 1, 0] = r;
                    path[path_count - 1, 1] = path[path_count - 2, 1];
                }
                else
                    done = true;
                if (!done)
                {
                    find_prime_in_row(path[path_count - 1, 0], ref c);
                    path_count += 1;
                    path[path_count - 1, 0] = path[path_count - 2, 0];
                    path[path_count - 1, 1] = c;
                }
            }
            augment_path();
            clear_covers();
            erase_primes();
            step = 3;
        }

        //methods to support step 6
        private void find_smallest(ref int minval)
        {
            for (int r = 0; r < nrow; r++)
                for (int c = 0; c < ncol; c++)
                    if (RowCover[r] == 0 && ColCover[c] == 0)
                        if (minval > C[r, c])
                            minval = C[r, c];
        }

        //Add the value found in Step 4 to every element of each covered row, and subtract 
        //it from every element of each uncovered column.  Return to Step 4 without 
        //altering any stars, primes, or covered lines.
        private void step_six(ref int step)
        {
            int minval = int.MaxValue;
            find_smallest(ref minval);
            for (int r = 0; r < nrow; r++)
                for (int c = 0; c < ncol; c++)
                {
                    if (RowCover[r] == 1)
                        C[r, c] += minval;
                    if (ColCover[c] == 0)
                        C[r, c] -= minval;
                }
            step = 4;
        }

        public int[] Solve(int[,] C)
        {
            // Save information about the cost matrix
            this.C = C;
            nrow = C.GetLength(0);
            ncol = C.GetLength(1);

            // Reset variables
            resetMaskandCovers();
            
            // Compute
            step = 1;
            RunMunkres();

            // Package result
            int[] result = new int[nrow];
            for (int r = 0; r < nrow; r++)
            {
                int oneIdx = -1;
                for (int c = 0; c < ncol; c++)
                {
                    if (M[r, c] == 1)
                    {
                        oneIdx = c;
                        break;
                    }
                }

                if (oneIdx >= 0 && oneIdx < ncol)
                {
                    result[r] = oneIdx;
                }
                else
                {
                    Console.WriteLine("HungarianAlgorithm::Solve(): Did not find assignment for worker " + r);
                    result[r] = -1;
                }
            }

            // Return to caller
            return result;
        }

        private void RunMunkres()
        {
            bool done = false;
            while (!done)
            {
                ShowCostMatrix();
                ShowMaskMatrix();
                switch (step)
                {
                    case 1:
                        step_one(ref step);
                        break;
                    case 2:
                        step_two(ref step);
                        break;
                    case 3:
                        step_three(ref step);
                        break;
                    case 4:
                        step_four(ref step);
                        break;
                    case 5:
                        step_five(ref step);
                        break;
                    case 6:
                        step_six(ref step);
                        break;
                    case 7:
                        done = true;
                        break;
                }
            }
        }

        private void ShowCostMatrix()
        {
            if (debugMode)
            {
                Console.WriteLine("\n");
                Console.WriteLine("------------Step {0}-------------", step);
                for (int r = 0; r < nrow; r++)
                {
                    Console.WriteLine();
                    Console.Write("     ");
                    for (int c = 0; c < ncol; c++)
                    {
                        Console.Write(Convert.ToString(C[r, c]) + " ");
                    }
                }
            }
        }

        private void ShowMaskMatrix()
        {
            if (debugMode)
            {
                Console.WriteLine();
                Console.Write("\n    ");
                for (int c = 0; c < ncol; c++)
                    Console.Write(" " + Convert.ToString(ColCover[c]));
                for (int r = 0; r < nrow; r++)
                {
                    Console.Write("\n  " + Convert.ToString(RowCover[r]) + "  ");
                    for (int c = 0; c < ncol; c++)
                    {
                        Console.Write(Convert.ToString(M[r, c]) + " ");
                    }
                }
            }
        }
    }
}
