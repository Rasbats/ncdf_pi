#ifndef ncdfREADER_H
#define ncdfREADER_H

#include <wx/wx.h>

class MainDialog;
class ncdfData;
class ncdfDataMessage;

class ncdfDataMessage
{
public:
	double getInterpolatedValue(ncdfDataMessage g2message, double** grid, double px, double py, bool numericalInterpolation) const;
	bool isPointInMap(ncdfDataMessage g2message, double x, double y) const;
	bool isXInMap(ncdfDataMessage g2message, double x) const;
	bool isYInMap(ncdfDataMessage g2message, double y) const;
	bool hasValue(double** grid, wxUint32 nolat, wxUint32 nolon, unsigned int indexlon, unsigned int indexlat) const;
	wxUint8		version;
	wxUint64 	length;
	wxUint8		discipline;
	// end Sector 0
	wxUint8		masterTableVersion;
	wxUint8		localTableVersion;
	wxUint8  	referenceTime;
	wxDateTime	dt;
	wxUint8		productionState;
	wxUint8		dataType;
	// end Secstor 1
	wxUint16	templateNo3;
	wxUint32	noSectors;
	wxUint32	noPointsParallel;
	wxUint32	noPointsMeridian;
	wxDouble	firstGridPointLat;
	wxDouble	firstGridPointLong;
	wxDouble	lastGridPointLat;
	wxDouble	lastGridPointLong;
	wxDouble	iDirectionIncr; // i = parallel
	wxDouble	jDirectionIncr; // j = Meridian;
	wxUint8		resCompFlag;
	wxUint8		scanMode;
	// end Secstor 3
	wxUint16	templateNo4;
	wxUint8		paramCategory;
	wxUint8		paramNo;
	wxUint8		typeOfProcess;
	wxUint16	hoursAfterRefTime;
	wxUint8		minutesAfterRefTime;
	wxUint8		indicatorTimeRange;
	wxUint32	forcastTimeUnits;
	// end Secstor 4
	wxUint16	templateNo5;
	wxFloat32	referenceVal;
	wxUint16	binaryScaleFacor;
	wxUint16	decimalScaleFactor;
	wxUint8		noBits;
	wxUint8		typeOrgFieldValues;
	wxUint32	scaledValueSurface1;
	wxUint8		compressionType;
	wxUint8		compressionRatio;
	// end Secstor 5
	wxInt8*		bmpMask;
	wxInt32		bmpSize;
	// end Secstor 6
	wxDouble*	data;
	wxDouble*    ucurr;
	wxDouble*    vcurr;
	wxDateTime  dataDateTime;
	int			minutesAfterStart;
	wxDouble* uvlats;
	wxDouble* uvlons;
	int			numberOfPoints;
	wxString   fileName;
	// end Sector 7
};


struct message
{
	wxUint8		version;
	wxUint64 	length;
	wxUint8		discipline;
	// end Sector 0
	wxUint8		masterTableVersion;
	wxUint8		localTableVersion;
	wxUint8  	referenceTime;
	wxDateTime	dt;
	wxUint8		productionState;
	wxUint8		dataType;
	// end Secstor 1
	wxUint16	templateNo3;
	wxUint32	noSectors;
	wxUint32	noPointsParallel;
	wxUint32	noPointsMeridian;
	wxDouble	firstGridPointLat;
	wxDouble	firstGridPointLong;
	wxDouble	lastGridPointLat;
	wxDouble	lastGridPointLong;
	wxDouble	iDirectionIncr; // i = parallel
	wxDouble	jDirectionIncr; // j = Meridian;
	wxUint8		resCompFlag;
	wxUint8		scanMode;
	// end Secstor 3
	wxUint16	templateNo4;
	wxUint8		paramCategory;
	wxUint8		paramNo;
	wxUint8		typeOfProcess;
	wxUint16	hoursAfterRefTime;
	wxUint8		minutesAfterRefTime;
	wxUint8		indicatorTimeRange;
	wxUint32	forcastTimeUnits;
	// end Secstor 4
	wxUint16	templateNo5;
	wxFloat32	referenceVal;
	wxUint16	binaryScaleFacor;
	wxUint16	decimalScaleFactor;
	wxUint8		noBits;
	wxUint8		typeOrgFieldValues;
	wxUint32	scaledValueSurface1;
	wxUint8		compressionType;
	wxUint8		compressionRatio;
	// end Secstor 5
	wxInt8*		bmpMask;
	wxInt32		bmpSize;
	// end Secstor 6
	wxDouble*	data;
	wxDouble    ucurr;
	wxDouble    vcurr;
	// end Sector 7
};
typedef struct message ncdfMessage;

class ncdfReader {

public:
	ncdfReader(MainDialog *dlg);
	~ncdfReader();

	void readncdfFile(ncdfDataMessage dataMessage);


	ncdfData *ncdfData1;
	ncdfMessage ncdfMessage1;
	ncdfDataMessage ncdf2DataMessage;
	bool isReading;
	bool gotData = false;


private:
	MainDialog *gui;
	double **gridu;
	double **gridv;
};

#endif // ncdfREADER_H
