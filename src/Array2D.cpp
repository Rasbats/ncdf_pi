#include "Array2D.h"

#define NULL 0

Array2D::Array2D(int R, int C, int T) {
	nRows = R;
	nCols = C;
	nTimes = T;

	Data = new int[nRows * nCols * nTimes];
	if (Data == NULL) {
		nRows = nCols = nTimes = 0;
	}
}
Array2D::~Array2D() {
	delete Data;
}




void Array2D::Set(int R, int C, int T, float Value){
	
	 value = Value;
}

float Array2D::Get(int R, int C, int T) const{
	return value;
}
