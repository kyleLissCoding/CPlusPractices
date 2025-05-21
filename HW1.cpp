
// changed to C++ io stream instead of C's io stream
#include <iostream>
// added the vector library to ensure vectors work correctly in file
#include <vector>
//added namespace for general use across the functions
using namespace std;

// added the inline function to the sum function
inline void sum(int *p, int n, vector<int> d)
{
    *p = 0;
    // added declaration to for loop
    for (int i = 0; i < n; ++i)
        *p = *p + d[i];
}

int main()
{
    //added const instead of define at top of file
    const int N = 40;
    int accum = 0;
    // changed the C array to a vector to be more C++ like
    vector<int> data(N);

    // added declaration statement to i in for loop
    for (int i = 0; i < N; ++i)
        data[i] = i;

    sum(&accum, N, data);

    // changed print function to cout
    cout << "sum is " << accum << endl;

    return 0;
}