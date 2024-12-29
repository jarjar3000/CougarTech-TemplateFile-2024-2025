class Matrix
{
    public:
        /**
         * @brief Function to add two matrices together and store the result in a preexisting array
         * @note To add two matricies, they must be the same size.
         * @param A The address, in memory, of a 2D array (a pointer to the first element in the array)
         * @param B The address, in memory, of a 2D array (a pointer to the first element in the array)
         * @param result The address, in memory, of a 2D array (a pointer to the first element in the array) for the result to be stored in
         */
        template <int rows, int columns>
        static void add(const int (&A)[rows][columns], const int (&B)[rows][columns], const int (&result)[rows][columns])
        {
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < columns; j++)
                {
                    result[i][j] = A[i][j] + B[i][j];
                }
            }
        }

        /**
         * @brief Function to subtract two matricies and store the result in a preexisting array
         * @note To subtract matricies, they must be the same size.
         * @param A The address, in memory, of a 2D array (a pointer to the first element in the array)
         * @param B The address, in memory, of a 2D array (a pointer to the first element in the array)
         * @param result The address, in memory, of a 2D array (a pointer to the first element in the array) for the result to be stored in
         */
        template <int rows, int columns>
        static void subtract(const int (&A)[rows][columns], const int (&B)[rows][columns], const int (&result)[rows][columns])
        {
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < columns; j++)
                {
                    result[i][j] = A[i][j] - B[i][j];
                }
            }
        }

        /**
         * @brief Function to multiply two matricies and store the result in a preexisting array
         * @note To multiply two matricies, the first matrix must have the same number of columns as the second one has rows. The resulting matrix will have the same number of rows as the first matrix and the same number of columns as the second one.
         * @param A The address, in memory, of a 2D array (a pointer to the first element in the array)
         * @param B The address, in memory, of a 2D array (a pointer to the first element in the array)
         * @param result The address, in memory, of a 2D array (a pointer to the first element in the array) for the result to be stored in
         */
        template <int rows, int columns, int resultColumns>
        static void multiply(const int (&A)[rows][columns], const int (&B)[columns][resultColumns], const int (&result)[rows][resultColumns])
        {
            for (int i = 0; i < rows; i++) // Iterate through rows of result
            {
                for (int j = 0; j < resultColumns; j++) // Iterate through columns of result
                {
                    result[i][j] = 0; // Initialize result matrix
                    for (int k = 0; k < columns; k++) // Do multiplication
                    {
                        result[i][j] += A[i][k] * B[k][j]; // Both share dimension k (A has same number of columns as B has rows)
                    }
                }
            }
        }

        /**
         * @brief Function to transpose a matrix and store the result in a preexisting array
         * @note Transposing may change the dimensions of the array
         * @param A The address, in memory, of a 2D array (a pointer to the first element in the array)
         * @param result The address, in memory, of a 2D array (a pointer to the first element in the array) for the result to be stored in
         */
        template <int rows, int columns>
        static void transpose(const int (&A)[rows][columns], const int (&result)[columns][rows])
        {
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < columns; j++)
                {
                    result[j][i] = A[i][j];
                }
            }
        }
};