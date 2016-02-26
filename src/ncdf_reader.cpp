#include "ncdf_pi.h"
#include "ncdf_reader.h"
#include "ncdf.h"
#include "helper.h"
#include "ncdfdata.h"

class ncdfDataMessage;

ncdfReader::ncdfReader(MainDialog *md)
{
	gui = md;
	ncdfMessage.data = NULL;
	ncdfMessage.bmpMask = NULL;
	ncdfData = NULL;

	
	isReading = false;
}

ncdfReader::~ncdfReader()
{
	if(ncdfData)
		delete ncdfData;
}

void ncdfReader::readncdfFile(ncdfDataMessage dataMessage)
{
	gui->myMessage = dataMessage;	
	gui->gridu = new double*[dataMessage.noPointsMeridian];

	for (int i = 0; i < dataMessage.noPointsMeridian; ++i) { // = NLAT
		gui->gridu[i] = new double[dataMessage.noPointsParallel];	// = NLON	
	}
	int c = 0;
	for (int i = 0; i <dataMessage.noPointsMeridian; i++)    //This loops on the rows.Nj
	{
		for (int j = 0; j<dataMessage.noPointsParallel; j++) //This loops on the columns.Ni
		{			    
				gui->gridu[i][j] = dataMessage.ucurr[c];
				c++;			
		}
	}
	gui->gridv = new double*[dataMessage.noPointsMeridian];

	for (int i = 0; i < dataMessage.noPointsMeridian; ++i) {
		gui->gridv[i] = new double[dataMessage.noPointsParallel];		
	}
	c = 0;
	for (int i = 0; i <dataMessage.noPointsMeridian; i++)    //This loops on the rows.Nj
	{
		for (int j = 0; j<dataMessage.noPointsParallel; j++) //This loops on the columns.Ni
		{
			
			gui->gridv[i][j] = dataMessage.vcurr[c]; 
			c++;
		}
	}

	wxDateTime ddt;
	ddt = dataMessage.dataDateTime;
	
    wxString ls = ddt.Format(_T("%a %d %b %Y %H:%M"));
	gui->m_staticTextDateTime->SetLabel(ls);	

	gui->pPlugIn->GetncdfOverlayFactory()->reset();
	gui->pPlugIn->GetncdfOverlayFactory()->setData(gui,
							   dataMessage,
							   dataMessage.numberOfPoints,
							   dataMessage.firstGridPointLat,
							   dataMessage.firstGridPointLong,
							   dataMessage.lastGridPointLat,
							   dataMessage.lastGridPointLong
							   );						   
	isReading = false;
}



