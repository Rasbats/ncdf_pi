#include "netcdf.h"

class Array2D {
private:
	int  *Data;
	int  nRows;                        // (logical) number of rows
	int  nCols;
	int  nTimes; // (logical) number of columns
	int  Map(int R, int C, int T) const;      // translate [R][C] to right 1D index
	bool ValidRC(int R, int C, int T) const;  // check if [R][C] specify a cell
public:
	float value;
	Array2D();
	Array2D(int R, int C, int T);             // allocate array of R*C cells
	Array2D(const Array2D& Source);    // copy constructor and assignment
	Array2D& operator=(const Array2D& Source);
	void Set(int R, int C, int T, float Value); // store Value at A[R][C] (logically)
	float  Get(int R, int C, int T) const;      // get Value at 
	~Array2D();
};