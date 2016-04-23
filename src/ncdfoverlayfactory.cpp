/*
    Copyright (c) 2011, Konni <email>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Konni <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Konni <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#define PI 3.14159265

#include "ncdfoverlayfactory.h"
#include "ncdf.h"
#include "IsoLine2.h"
#include <wx/colour.h>
#include <wx/dynarray.h>

#ifdef __WXMSW__
#define snprintf _snprintf
#endif // __WXMSW__

#define NUM_CURRENT_ARROW_POINTS 9
static wxPoint CurrentArrowArray[NUM_CURRENT_ARROW_POINTS] = { wxPoint(0, 0), wxPoint(0, -10),
wxPoint(55, -10), wxPoint(55, -25), wxPoint(100, 0), wxPoint(55, 25), wxPoint(55,
10), wxPoint(0, 10), wxPoint(0, 0)};

//----------------------------------------------------------------------------------------------------------
//    ncdf Overlay Factory Implementation
//----------------------------------------------------------------------------------------------------------
ncdfOverlayFactory::ncdfOverlayFactory()
{
      hash = NULL;
      
      m_pbm_current 	= NULL;     
      
      m_last_vp_latMax = -99999.0;
      rect = NULL;
      
      space[0]=30; space[1]=50; space[2]=100; space[3]=200; space[4]=400; space[5]=600; space[6]=1200;
	  m_bReadyToRender = false;
      m_space = 0;
}

ncdfOverlayFactory::~ncdfOverlayFactory()
{
    m_bReadyToRender = false;
	renderSelectionRectangle = false;
}

void ncdfOverlayFactory::setData(MainDialog *gui, ncdfDataMessage g2data, int numberOfPoints, wxDouble tlat, wxDouble tlon, wxDouble blat, wxDouble blon)
{

	this->g2data = g2data;
	this->numberOfPoints = numberOfPoints;
    this->tlat = tlat; this->tlon = tlon; this->blat = blat; this->blon = blon;
    this->gui = gui;
    
    this->m_bReadyToRender = true;
}

void ncdfOverlayFactory::setSelectionRectangle(Selection *rect)
{
    this->rect = rect;
}

void ncdfOverlayFactory::reset()
{
    this->m_bReadyToRender = false;
    clearBmp();
}


bool ncdfOverlayFactory::RenderGLncdfOverlay(wxGLContext *pcontext, PlugIn_ViewPort *vp)
{
	m_pdc = NULL;  // inform lower layers that this is OpenGL render
	return DoRenderncdfOverlay(vp);
}


bool ncdfOverlayFactory::RenderncdfOverlay(wxDC &dc, PlugIn_ViewPort *vp)
{
#if wxUSE_GRAPHICS_CONTEXT
	wxMemoryDC *pmdc;
	pmdc = wxDynamicCast(&dc, wxMemoryDC);
	wxGraphicsContext *pgc = wxGraphicsContext::Create(*pmdc);
	m_gdc = pgc;
#endif
	m_pdc = &dc;
	return DoRenderncdfOverlay(vp);
}


bool ncdfOverlayFactory::DoRenderncdfOverlay(PlugIn_ViewPort *vp )
{
    this->vp = vp;
	//return true; //for debug
    
    if(vp->view_scale_ppm != m_last_vp_scale)
    {
      if(vp->view_scale_ppm < 0.001135)
	  m_space = space[0];
      else if(vp->view_scale_ppm <= 0.001135)
	  m_space = space[1];
      else if(vp->view_scale_ppm <= 0.018165)
	  m_space = space[2]; 
      else if(vp->view_scale_ppm <= 0.072659)
	  m_space = space[3];  

    }
	if (m_last_vp_latMax)	

		if (m_last_vp_latMax != vp->lat_max)
		{
			clearBmp();
		}

	if (!m_bReadyToRender) return false;


	if(gui->m_checkBoxBmpCurrentForce->GetValue())

      RenderncdfCurrentBmp();    
	
	if(gui->m_checkBoxDCurrent->GetValue())
      RenderncdfCurrent();    

    m_last_vp_scale = vp->view_scale_ppm;
    m_last_vp_latMax = vp->lat_max;    
  
    return true;
}

void ncdfOverlayFactory::clearBmp()
{

      if(m_pbm_current)
	delete m_pbm_current;
    
	m_pbm_current 	= NULL;  
	
}

void ncdfOverlayFactory::RenderSelectionRectangle()
{
    wxPoint ptop, pbottom;
    wxInt32 width,height;
    
    GetCanvasPixLL(vp, &ptop, rect->topLat, rect->topLon);
    if(rect->bottomLat != ncdf_NOTDEF)
    {
      GetCanvasPixLL(vp, &pbottom, rect->bottomLat, rect->bottomLon);
		width = pbottom.x  - ptop.x;
		height = pbottom.y - ptop.y;
    
		pmdc->DrawLine(ptop.x,ptop.y, pbottom.x,ptop.y);
		pmdc->DrawLine(pbottom.x,ptop.y, pbottom.x,pbottom.y);
		pmdc->DrawLine(pbottom.x,pbottom.y,ptop.x,pbottom.y);
		pmdc->DrawLine(ptop.x,pbottom.y,ptop.x,ptop.y);  
    }
}



void ncdfOverlayFactory::RenderncdfCurrent()
{
	
      wxColour colour;
      wxPoint p;
	  int mi;	
	 

	  for (mi = 0; mi < numberOfPoints; mi++){		  
 
		  double lat = g2data.uvlats[mi];
		  double lon = g2data.uvlons[mi];

		  bool barbs = true;

		  //    Set minimum spacing between wind arrows
		  int space;

		  if (barbs)
			  space = 30;
		  else
			  space = 20;

		  GetGlobalColor(_T("UBLCK"), &colour);
		
		  if (g2data.ucurr[mi] != NULL && g2data.vcurr[mi] != NULL){
			  GetCanvasPixLL(vp, &p, lat, lon);
			  if (PointInLLBox(vp, lon, lat))
			  {
				 double dir = 90. + (atan2(g2data.vcurr[mi], -g2data.ucurr[mi])  * 180. / PI);
				 if (dir < 0) dir = 360 + dir;
					  drawWaveArrow(p.x, p.y, dir - 90, colour);					
			  }
		  }
	  }
}


bool ncdfOverlayFactory::RenderncdfCurrentBmp()
{

	static wxPoint porg;
      static int width, height;    
      // If needed, create the bitmap
            if(m_pbm_current == NULL)
            {
                 wxPoint pmin;
                  GetCanvasPixLL(vp,  &pmin, tlat, tlon);
                  wxPoint pmax;
                  GetCanvasPixLL(vp,  &pmax, blat, blon);

                  width = abs(pmax.x - pmin.x);
                  height = abs(pmax.y - pmin.y);	

				  

	          if(vp->pix_width < width) width = vp->pix_width;
		  if(vp->pix_height < height || vp->lat_max < blat) 
		  { 
		    GetCanvasPixLL(vp,  &porg, vp->lat_max, tlon);
		    height = vp->pix_height; 		    
		  }
		  if(vp->pix_width == width && vp->pix_height == height) 
		  { 
		    GetCanvasPixLL(vp,  &porg, vp->lat_max, vp->lon_min);
		  }
		  else	if(vp->pix_width != width && vp->pix_height != height)  
		    GetCanvasPixLL(vp,  &porg, blat, tlon);
		 		  
                  {
					  //    Dont try to create enormous GRIB bitmaps ( no more than the screen size )
					  if (width > m_ParentSize.GetWidth() || height > m_ParentSize.GetHeight()){
						  return false;
					  }

					  //    This could take a while....
			      double **currentDir = (double **) g2data.ucurr;
				  if (currentDir == NULL){ 
					  return false;
				  }

			      
			      double **currentForce = (double **) g2data.vcurr;
				  if (currentForce == NULL){					 
					  return false;
				  }
                              wxImage gr_image(width, height);
                              gr_image.InitAlpha();

                              int ncdf_pixel_size = 8;
			      
			      ::wxBeginBusyCursor();
				  int myCounter = 0;
                              wxPoint p;
                              for(int ipix = 0 ; ipix < (width + 1) ; ipix += ncdf_pixel_size)
                              {
                                    for(int jpix = 0 ; jpix < (height + 1) ; jpix += ncdf_pixel_size)
                                    {
                                          double lat, lon;
                                          p.x = ipix + porg.x;
                                          p.y = jpix + porg.y;
                                          GetCanvasLLPix( vp, p, &lat, &lon);

					                if(!PointInLLBox(vp, lon, lat) && !PointInLLBox(vp, lon-360.0, lat)) continue;
									double vx = gui->myMessage.getInterpolatedValue(gui->myMessage, gui->gridu, lon, lat, true);
									double vy = gui->myMessage.getInterpolatedValue(gui->myMessage, gui->gridv, lon, lat, true);

                                          if ((vx != ncdf_NOTDEF) && (vy != ncdf_NOTDEF))
                                          {
											    double  vkn = sqrt(vx*vx+vy*vy)*3.6/1.852;
                                                wxColour c = GetSeaCurrentGraphicColor(vkn);
                                                unsigned char r = c.Red();
                                                unsigned char g = c.Green();
                                                unsigned char b = c.Blue();

                                                for(int xp=0 ; xp < ncdf_pixel_size ; xp++)
                                                      for(int yp=0 ; yp < ncdf_pixel_size ; yp++)
                                                {
                                                      gr_image.SetRGB(ipix + xp, jpix + yp, r,g,b);
													  gr_image.SetAlpha(ipix + xp, jpix + yp, 128);

                                                }
                                          }
                                          else
                                          {
                                                for(int xp=0 ; xp < ncdf_pixel_size ; xp++)
                                                      for(int yp=0 ; yp < ncdf_pixel_size ; yp++)
                                                {
                                                      gr_image.SetAlpha(ipix + xp, jpix + yp, 0);
                                                }
                                          }
									 myCounter++;
									}
									
                              }
                              wxImage bl_image = (gr_image.Blur(4));

                        //    Create a Bitmap
                              m_pbm_current = new wxBitmap(bl_image);
                              wxMask *gr_mask = new wxMask(*m_pbm_current, wxColour(0,0,0));
                              m_pbm_current->SetMask(gr_mask);

                              ::wxEndBusyCursor();
                  }
            }

            if(m_pbm_current)
            {      
                  DrawOLBitmap(*m_pbm_current, porg.x, porg.y, true);
            }
      return true;
	  
 
}


void ncdfOverlayFactory::drawWaveArrow(int i, int j, double ang, wxColour arrowColor)
{
 double si=sin(ang * PI / 180.),  co=cos(ang * PI / 180.);

 wxPen pen(arrowColor, 1);
 if (m_pdc) {	
	 
		  m_pdc->SetPen(pen);
		  m_pdc->SetBrush(*wxTRANSPARENT_BRUSH);

#if wxUSE_GRAPHICS_CONTEXT
		  if (m_hiDefGraphics && m_gdc)
			  m_gdc->SetPen(pen);
#endif
			}
#ifdef ocpnUSE_GL
	  else
		  glColor3ub(arrowColor.Red(), arrowColor.Green(), arrowColor.Blue());
#endif
      int arrowSize = 26;
      int dec = -arrowSize/2;

      drawTransformedLine(pen, si,co, i,j,  dec,-2,  dec + arrowSize, -2);
	  drawTransformedLine(pen, si, co, i, j, dec, 2, dec + arrowSize, +2);

	  drawTransformedLine(pen, si, co, i, j, dec - 2, 0, dec + 5, 6);    // flèche
	  drawTransformedLine(pen, si, co, i, j, dec - 2, 0, dec + 5, -6);   // flèche

}

void ncdfOverlayFactory::drawTransformedLine( wxPen pen, double si, double co,int di, int dj, int i,int j, int k,int l)
{
      int ii, jj, kk, ll;
      ii = (int) (i*co-j*si +0.5) + di;
      jj = (int) (i*si+j*co +0.5) + dj;
      kk = (int) (k*co-l*si +0.5) + di;
      ll = (int) (k*si+l*co +0.5) + dj;

	  wxColor colour;
	  GetGlobalColor(_T("UBLCK"), &colour);

	  if (m_pdc){
		  m_pdc->SetPen(pen);
		  m_pdc->SetBrush(*wxTRANSPARENT_BRUSH);
		  m_pdc->DrawLine(ii, jj, kk, ll);

	  }
	  else{
		  DrawGLLine(ii, jj, kk, ll, 0.5, colour);
	  }
}

void ncdfOverlayFactory::DrawGLLine(double x1, double y1, double x2, double y2, double width, wxColour myColour)
{
	{
		wxColour isoLineColor = myColour;
		glColor4ub(isoLineColor.Red(), isoLineColor.Green(), isoLineColor.Blue(),
			255/*isoLineColor.Alpha()*/);

		glPushAttrib(GL_COLOR_BUFFER_BIT | GL_LINE_BIT | GL_ENABLE_BIT |
			GL_POLYGON_BIT | GL_HINT_BIT); //Save state
		{

			//      Enable anti-aliased lines, at best quality
			glEnable(GL_LINE_SMOOTH);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
			glLineWidth(width);

			glBegin(GL_LINES);
			glVertex2d(x1, y1);
			glVertex2d(x2, y2);
			glEnd();
		}

		glPopAttrib();
	}
}

void ncdfOverlayFactory::DrawOLBitmap(const wxBitmap &bitmap, wxCoord x, wxCoord y, bool usemask)
{
	wxBitmap bmp;
	if (x < 0 || y < 0) {
		int dx = (x < 0 ? -x : 0);
		int dy = (y < 0 ? -y : 0);
		int w = bitmap.GetWidth() - dx;
		int h = bitmap.GetHeight() - dy;
		/* picture is out of viewport */
		if (w <= 0 || h <= 0) return;
		wxBitmap newBitmap = bitmap.GetSubBitmap(wxRect(dx, dy, w, h));
		x += dx;
		y += dy;
		bmp = newBitmap;
	}
	else {
		bmp = bitmap;
	}
	if (m_pdc)
		m_pdc->DrawBitmap(bmp, x, y, usemask);
	else {
		wxImage image = bmp.ConvertToImage();
		int w = image.GetWidth(), h = image.GetHeight();

		if (usemask) {
			unsigned char *d = image.GetData();
			unsigned char *a = image.GetAlpha();

			unsigned char mr, mg, mb;
			if (!image.GetOrFindMaskColour(&mr, &mg, &mb) && !a) printf(
				"trying to use mask to draw a bitmap without alpha or mask\n");

			unsigned char *e = new unsigned char[4 * w * h];
			{
				for (int y = 0; y < h; y++)
					for (int x = 0; x < w; x++) {
						unsigned char r, g, b;
						int off = (y * image.GetWidth() + x);
						r = d[off * 3 + 0];
						g = d[off * 3 + 1];
						b = d[off * 3 + 2];

						e[off * 4 + 0] = r;
						e[off * 4 + 1] = g;
						e[off * 4 + 2] = b;

						e[off * 4 + 3] =
							a ? a[off] : ((r == mr) && (g == mg) && (b == mb) ? 0 : 255);
					}
			}

			glColor4f(1, 1, 1, 1);

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glRasterPos2i(x, y);
			glPixelZoom(1, -1);
			glDrawPixels(w, h, GL_RGBA, GL_UNSIGNED_BYTE, e);
			glPixelZoom(1, 1);
			glDisable(GL_BLEND);

			delete[](e);
		}
		else {
			glRasterPos2i(x, y);
			glPixelZoom(1, -1); /* draw data from top to bottom */
			glDrawPixels(w, h, GL_RGB, GL_UNSIGNED_BYTE, image.GetData());
			glPixelZoom(1, 1);
		}
	}
}

void ncdfOverlayFactory::DrawAllCurrentsInViewPort(double dlat, double dlon, double ddir, double dfor, wxDC &myDC, PlugIn_ViewPort *myVP)
{
	
	if (myVP->chart_scale > 1000000){
		return;
	}

	wxDC *m_dc = &myDC;

	double rot_vp = myVP->rotation * 180 / M_PI;

	// Set up the scaler
	int mmx, mmy;
	wxDisplaySizeMM(&mmx, &mmy);

	int sx, sy;
	wxDisplaySize(&sx, &sy);

	double m_pix_per_mm = ((double)sx) / ((double)mmx);

	int mm_per_knot = 10;
	float current_draw_scaler = mm_per_knot * m_pix_per_mm * 100 / 100.0;

	// End setting up scaler

	float tcvalue, dir;

	tcvalue = dfor;
	dir = ddir;
	
	double lat, lon;

	lat = dlat;
	lon = dlon;

	int pixxc, pixyc;
	wxPoint cpoint;
	GetCanvasPixLL(myVP, &cpoint, lat, lon);
	
	pixxc = cpoint.x;
	pixyc = cpoint.y;


	//    Adjust drawing size using logarithmic scale
	double a1 = fabs(tcvalue) * 10;
	a1 = wxMax(1.0, a1);      // Current values less than 0.1 knot
	// will be displayed as 0
	double a2 = log10(a1);
	double scale = current_draw_scaler * a2;
	
	drawCurrentArrow(pixxc, pixyc, dir - 90 + rot_vp, scale / 100, tcvalue, myDC, myVP);
	
	int shift = 0;

	if (!m_dc){

		wxColour colour;
		colour = GetSeaCurrentGraphicColor(tcvalue);
		c_GLcolour = colour;  // for filling GL arrows

		drawGLPolygons(this, m_dc, vp, DrawGLPolygon(), lat, lon, shift);
	}

	shift = 5;

	if (!m_dc){		
		
		DrawGLLabels(this, m_dc, vp,  DrawGLText(fabs(tcvalue), 1), lat, lon, 0);
		shift = 13;
		
		DrawGLLabels(this, m_dc,vp, DrawGLTextDir(dir, 0), lat, lon, shift);		
	}

	char sbuf[20];
	
	wxFont *pTCFont;
	pTCFont = wxTheFontList->FindOrCreateFont(12, wxDEFAULT, wxNORMAL, wxBOLD, FALSE,
		wxString(_T("Eurostile Extended")));

	if (m_dc)
	{
		m_dc->SetFont(*pTCFont);
		snprintf(sbuf, 19, "%3.1f", fabs(tcvalue));
		m_dc->DrawText(wxString(sbuf, wxConvUTF8), pixxc, pixyc);
		shift = 13;
	}

	if (m_dc)
	{
		snprintf(sbuf, 19, "%03.0f", dir);
		m_dc->DrawText(wxString(sbuf, wxConvUTF8), pixxc, pixyc + shift);
	}



}

void ncdfOverlayFactory::DrawAllGLCurrentsInViewPort(double dlat, double dlon, double ddir, double dfor, wxGLContext *pcontext, PlugIn_ViewPort *myVP)
{

	if (myVP->chart_scale > 1000000){
		return;
	}

	m_pdc = NULL;
	

	double rot_vp = myVP->rotation * 180 / M_PI;

	// Set up the scaler
	int mmx, mmy;
	wxDisplaySizeMM(&mmx, &mmy);

	int sx, sy;
	wxDisplaySize(&sx, &sy);

	double m_pix_per_mm = ((double)sx) / ((double)mmx);

	int mm_per_knot = 10;
	float current_draw_scaler = mm_per_knot * m_pix_per_mm * 100 / 100.0;

	// End setting up scaler

	float tcvalue, dir;

	tcvalue = dfor;
	dir = ddir;

	double lat, lon;

	lat = dlat;
	lon = dlon;

	int pixxc, pixyc;
	wxPoint cpoint;
	GetCanvasPixLL(myVP, &cpoint, lat, lon);

	pixxc = cpoint.x;
	pixyc = cpoint.y;


	//    Adjust drawing size using logarithmic scale
	double a1 = fabs(tcvalue) * 10;
	a1 = wxMax(1.0, a1);      // Current values less than 0.1 knot
	// will be displayed as 0
	double a2 = log10(a1);
	double scale = current_draw_scaler * a2;

	drawCurrentArrow(pixxc, pixyc, dir - 90 + rot_vp, scale / 100, tcvalue, *m_pdc, myVP);

	int shift = 0;

	

		wxColour colour;
		colour = GetSeaCurrentGraphicColor(tcvalue);
		c_GLcolour = colour;  // for filling GL arrows

		drawGLPolygons(this, m_pdc, myVP, DrawGLPolygon(), lat, lon, shift);
	
		DrawGLLabels(this, m_pdc, myVP, DrawGLText(fabs(tcvalue), 1), lat, lon, 0);
		shift = 13;

		DrawGLLabels(this, m_pdc, myVP, DrawGLTextDir(dir, 0), lat, lon, shift);

}

void ncdfOverlayFactory::drawCurrentArrow(int x, int y, double rot_angle, double scale, double rate, wxDC &dc, PlugIn_ViewPort *vp)
{
	double m_rate = fabs(rate);
	wxPoint p[9];

	wxColour colour;
	colour = GetSeaCurrentGraphicColor(m_rate);

	c_GLcolour = colour;  // for filling GL arrows

	wxPen pen(colour, 2);
	wxBrush brush(colour);

	myDC = &dc;

	if (myDC) {
		myDC->SetPen(pen);
		myDC->SetBrush(brush);
	}
	

	if (scale > 1e-2) {

		float sin_rot = sin(rot_angle * PI / 180.);
		float cos_rot = cos(rot_angle * PI / 180.);

		// Move to the first point

		float xt = CurrentArrowArray[0].x;
		float yt = CurrentArrowArray[0].y;

		float xp = (xt * cos_rot) - (yt * sin_rot);
		float yp = (xt * sin_rot) + (yt * cos_rot);
		int x1 = (int)(xp * scale);
		int y1 = (int)(yp * scale);

		p[0].x = x;
		p[0].y = y;

		p_basic[0].x = 100;
		p_basic[0].y = 100;

		// Walk thru the point list
		for (int ip = 1; ip < NUM_CURRENT_ARROW_POINTS; ip++) {
			xt = CurrentArrowArray[ip].x;
			yt = CurrentArrowArray[ip].y;

			float xp = (xt * cos_rot) - (yt * sin_rot);
			float yp = (xt * sin_rot) + (yt * cos_rot);
			int x2 = (int)(xp * scale);
			int y2 = (int)(yp * scale);

			p_basic[ip].x = 100 + x2;
			p_basic[ip].y = 100 + y2;
			
			

			if (myDC){
				myDC->DrawLine(x1 + x, y1 + y, x2 + x, y2 + y);
			}
			else{
				DrawGLLine(x1 + x, y1 + y, x2 + x, y2 + y, 2, colour);
			}

			p[ip].x = x1 + x;
			p[ip].y = y1 + y;

			x1 = x2;
			y1 = y2;
		}
		
		if (myDC){
			myDC->SetBrush(brush);
			myDC->DrawPolygon(9, p);
		}
		
	}
}

wxImage &ncdfOverlayFactory::DrawGLPolygon(){

	wxString labels;
	labels = _T("");  // dummy label for drawing with

	wxColour c_orange = c_GLcolour;

	wxPen penText(c_orange);
	wxBrush backBrush(c_orange);

	wxMemoryDC mdc(wxNullBitmap);

	wxFont mfont(9, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
	mdc.SetFont(mfont);

	int w, h;
	mdc.GetTextExtent(labels, &w, &h);

	w = 200;
	h = 200;

	wxBitmap bm(w, h);
	mdc.SelectObject(bm);
	mdc.Clear();

	mdc.SetPen(penText);
	mdc.SetBrush(backBrush);
	mdc.SetTextForeground(c_orange);
	mdc.SetTextBackground(c_orange);

	int xd = 0;
	int yd = 0;

	mdc.DrawPolygon(9, p_basic, 0);

	mdc.SelectObject(wxNullBitmap);

	m_labelCacheText[labels] = bm.ConvertToImage();

	m_labelCacheText[labels].InitAlpha();

	wxImage &image = m_labelCacheText[labels];

	unsigned char *d = image.GetData();
	unsigned char *a = image.GetAlpha();

	w = image.GetWidth(), h = image.GetHeight();
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++) {
			int r, g, b;
			int ioff = (y * w + x);
			r = d[ioff * 3 + 0];
			g = d[ioff * 3 + 1];
			b = d[ioff * 3 + 2];

			a[ioff] = 255 - (r + g + b) / 3;
		}

	return m_labelCacheText[labels];

}
void ncdfOverlayFactory::drawGLPolygons(ncdfOverlayFactory *pof, wxDC *dc,
	PlugIn_ViewPort *vp, wxImage &imageLabel, double myLat, double myLon, int offset)
{

	//---------------------------------------------------------
	// Ecrit les labels
	//---------------------------------------------------------

	wxPoint ab;
	GetCanvasPixLL(vp, &ab, myLat, myLon);

	wxPoint cd;
	GetCanvasPixLL(vp, &cd, myLat, myLon);

	int w = imageLabel.GetWidth();
	int h = imageLabel.GetHeight();

	int label_offset = 0;
	int xd = (ab.x + cd.x - (w + label_offset * 2)) / 2;
	int yd = (ab.y + cd.y - h) / 2 + offset;

	if (dc) {
		/* don't use alpha for isobars, for some reason draw bitmap ignores
		the 4th argument (true or false has same result) */
		wxImage img(w, h, imageLabel.GetData(), true);
		dc->DrawBitmap(img, xd, yd, false);
	}
	else { /* opengl */

		int w = imageLabel.GetWidth(), h = imageLabel.GetHeight();

		unsigned char *d = imageLabel.GetData();
		unsigned char *a = imageLabel.GetAlpha();

		unsigned char mr, mg, mb;
		if (!imageLabel.GetOrFindMaskColour(&mr, &mg, &mb) && !a) wxMessageBox(_T(
			"trying to use mask to draw a bitmap without alpha or mask\n"));

		unsigned char *e = new unsigned char[4 * w * h];
		{
			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++) {
					unsigned char r, g, b;
					int off = (y * imageLabel.GetWidth() + x);
					r = d[off * 3 + 0];
					g = d[off * 3 + 1];
					b = d[off * 3 + 2];

					e[off * 4 + 0] = r;
					e[off * 4 + 1] = g;
					e[off * 4 + 2] = b;

					e[off * 4 + 3] =
						a ? a[off] : ((r == mr) && (g == mg) && (b == mb) ? 0 : 255);
				}
		}

		glColor4f(1, 1, 1, 1);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glRasterPos2i(xd, yd);
		glPixelZoom(1, -1);
		glDrawPixels(w, h, GL_RGBA, GL_UNSIGNED_BYTE, e);
		glPixelZoom(1, 1);
		glDisable(GL_BLEND);

		delete[](e);

	}


}

void ncdfOverlayFactory::DrawGLLabels(ncdfOverlayFactory *pof, wxDC *dc,
	PlugIn_ViewPort *vp, wxImage &imageLabel, double myLat, double myLon, int offset)
{

	//---------------------------------------------------------
	// Ecrit les labels
	//---------------------------------------------------------

	wxPoint ab;
	GetCanvasPixLL(vp, &ab, myLat, myLon);

	wxPoint cd;
	GetCanvasPixLL(vp, &cd, myLat, myLon);

	int w = imageLabel.GetWidth();
	int h = imageLabel.GetHeight();

	int label_offset = 0;
	int xd = (ab.x + cd.x - (w + label_offset * 2)) / 2;
	int yd = (ab.y + cd.y - h) / 2 + offset;

	if (dc) {
		/* don't use alpha for isobars, for some reason draw bitmap ignores
		the 4th argument (true or false has same result) */
		wxImage img(w, h, imageLabel.GetData(), true);
		dc->DrawBitmap(img, xd, yd, false);
	}
	else { /* opengl */

		int w = imageLabel.GetWidth(), h = imageLabel.GetHeight();

		unsigned char *d = imageLabel.GetData();
		unsigned char *a = imageLabel.GetAlpha();

		unsigned char mr, mg, mb;
		if (!imageLabel.GetOrFindMaskColour(&mr, &mg, &mb) && !a) wxMessageBox(_T(
			"trying to use mask to draw a bitmap without alpha or mask\n"));

		unsigned char *e = new unsigned char[4 * w * h];
		{
			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++) {
					unsigned char r, g, b;
					int off = (y * imageLabel.GetWidth() + x);
					r = d[off * 3 + 0];
					g = d[off * 3 + 1];
					b = d[off * 3 + 2];

					e[off * 4 + 0] = r;
					e[off * 4 + 1] = g;
					e[off * 4 + 2] = b;

					e[off * 4 + 3] =
						a ? a[off] : ((r == mr) && (g == mg) && (b == mb) ? 0 : 255);
				}
		}

		glColor4f(1, 1, 1, 1);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glRasterPos2i(xd, yd);
		glPixelZoom(1, -1);
		glDrawPixels(w, h, GL_RGBA, GL_UNSIGNED_BYTE, e);
		glPixelZoom(1, 1);
		glDisable(GL_BLEND);

		delete[](e);

	}


}


wxImage &ncdfOverlayFactory::DrawGLText(double value, int precision){

	wxString labels;

	int p = precision;

	labels.Printf(_T("%.*f"), p, value);

	wxMemoryDC mdc(wxNullBitmap);

	wxFont *pTCFont;
	pTCFont = wxTheFontList->FindOrCreateFont(12, wxDEFAULT, wxNORMAL, wxBOLD, FALSE,
		wxString(_T("Eurostile Extended")));
	mdc.SetFont(*pTCFont);

	int w, h;
	mdc.GetTextExtent(labels, &w, &h);

	int label_offset = 10;   //5

	wxBitmap bm(w + label_offset * 2, h + 1);
	mdc.SelectObject(bm);
	mdc.Clear();

	wxColour text_color;

	GetGlobalColor(_T("UINFD"), &text_color);
	wxPen penText(text_color);
	mdc.SetPen(penText);

	mdc.SetBrush(*wxTRANSPARENT_BRUSH);
	mdc.SetTextForeground(text_color);
	mdc.SetTextBackground(wxTRANSPARENT);

	int xd = 0;
	int yd = 0;

	mdc.DrawText(labels, label_offset + xd, yd + 1);

	mdc.SelectObject(wxNullBitmap);

	m_labelCache[value] = bm.ConvertToImage();

	m_labelCache[value].InitAlpha();

	wxImage &image = m_labelCache[value];

	unsigned char *d = image.GetData();
	unsigned char *a = image.GetAlpha();

	w = image.GetWidth(), h = image.GetHeight();
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++) {
			int r, g, b;
			int ioff = (y * w + x);
			r = d[ioff * 3 + 0];
			g = d[ioff * 3 + 1];
			b = d[ioff * 3 + 2];

			a[ioff] = 255 - (r + g + b) / 3;
		}

	return m_labelCache[value];
}

wxImage &ncdfOverlayFactory::DrawGLTextDir(double value, int precision){

	wxString labels;

	int p = precision;

	labels.Printf(_T("%03.*f"), p, value);

	wxMemoryDC mdc(wxNullBitmap);

	wxFont *pTCFont;
	pTCFont = wxTheFontList->FindOrCreateFont(12, wxDEFAULT, wxNORMAL, wxBOLD, FALSE,
		wxString(_T("Eurostile Extended")));

	mdc.SetFont(*pTCFont);

	int w, h;
	mdc.GetTextExtent(labels, &w, &h);

	int label_offset = 10;   //5

	wxBitmap bm(w + label_offset * 2, h + 1);
	mdc.SelectObject(bm);
	mdc.Clear();

	wxColour text_color;

	GetGlobalColor(_T("UINFD"), &text_color);
	wxPen penText(text_color);
	mdc.SetPen(penText);

	mdc.SetBrush(*wxTRANSPARENT_BRUSH);
	mdc.SetTextForeground(text_color);
	mdc.SetTextBackground(wxTRANSPARENT);

	int xd = 0;
	int yd = 0;

	mdc.DrawText(labels, label_offset + xd, yd + 1);

	mdc.SelectObject(wxNullBitmap);

	m_labelCache[value] = bm.ConvertToImage();

	m_labelCache[value].InitAlpha();

	wxImage &image = m_labelCache[value];

	unsigned char *d = image.GetData();
	unsigned char *a = image.GetAlpha();

	w = image.GetWidth(), h = image.GetHeight();
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++) {
			int r, g, b;
			int ioff = (y * w + x);
			r = d[ioff * 3 + 0];
			g = d[ioff * 3 + 1];
			b = d[ioff * 3 + 2];

			a[ioff] = 255 - (r + g + b) / 3;
		}

	return m_labelCache[value];
}

wxImage &ncdfOverlayFactory::DrawGLTextString(wxString myText){

	wxString labels;
	labels = myText;

	wxMemoryDC mdc(wxNullBitmap);

	wxFont *pTCFont;
	pTCFont = wxTheFontList->FindOrCreateFont(12, wxDEFAULT, wxNORMAL, wxBOLD, FALSE,
		wxString(_T("Eurostile Extended")));
	mdc.SetFont(*pTCFont);

	int w, h;
	mdc.GetTextExtent(labels, &w, &h);

	int label_offset = 10;   //5

	wxBitmap bm(w + label_offset * 2, h + 1);
	mdc.SelectObject(bm);
	mdc.Clear();

	wxColour text_color;

	GetGlobalColor(_T("UINFD"), &text_color);
	wxPen penText(text_color);
	mdc.SetPen(penText);

	mdc.SetBrush(*wxTRANSPARENT_BRUSH);
	mdc.SetTextForeground(text_color);
	mdc.SetTextBackground(wxTRANSPARENT);

	int xd = 0;
	int yd = 0;

	mdc.DrawText(labels, label_offset + xd, yd + 1);
	mdc.SelectObject(wxNullBitmap);

	m_labelCacheText[myText] = bm.ConvertToImage();

	m_labelCacheText[myText].InitAlpha();

	wxImage &image = m_labelCacheText[myText];

	unsigned char *d = image.GetData();
	unsigned char *a = image.GetAlpha();

	w = image.GetWidth(), h = image.GetHeight();
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++) {
			int r, g, b;
			int ioff = (y * w + x);
			r = d[ioff * 3 + 0];
			g = d[ioff * 3 + 1];
			b = d[ioff * 3 + 2];

			a[ioff] = 255 - (r + g + b) / 3;
		}

	return m_labelCacheText[myText];
}

void ncdfOverlayFactory::drawPetiteBarbule(wxDC *pmdc, wxPen pen, bool south,
                                 double si, double co, int di, int dj, int b)
{
      if (south)
            drawTransformedLine(pen, si,co, di,dj,  b,0,  b+2, -5);
      else
            drawTransformedLine(pen, si,co, di,dj,  b,0,  b+2, 5);
}

void ncdfOverlayFactory::drawGrandeBarbule(wxDC *pmdc, wxPen pen, bool south,
                                 double si, double co, int di, int dj, int b)
{
      if (south)
            drawTransformedLine(pen, si,co, di,dj,  b,0,  b+4,-10);
      else
            drawTransformedLine(pen, si,co, di,dj,  b,0,  b+4,10);
}


void ncdfOverlayFactory::drawTriangle(wxDC *pmdc, wxPen pen, bool south,
                            double si, double co, int di, int dj, int b)
{
      if (south) {
            drawTransformedLine(pen, si,co, di,dj,  b,0,  b+4,-10);
            drawTransformedLine(pen, si,co, di,dj,  b+8,0,  b+4,-10);
      }
      else {
            drawTransformedLine(pen, si,co, di,dj,  b,0,  b+4,10);
            drawTransformedLine(pen, si,co, di,dj,  b+8,0,  b+4,10);
      }
}

// Is the given point in the vp ??
bool PointInLLBox(PlugIn_ViewPort *vp, double x, double y)
{


    if (  x >= (vp->lon_min) && x <= (vp->lon_max) &&
            y >= (vp->lat_min) && y <= (vp->lat_max) )
            return TRUE;
    return FALSE;
}



wxColour ncdfOverlayFactory::GetSeaCurrentGraphicColor(double val_in)
{
      //    HTML colors taken from NOAA WW3 Web representation

      double val = val_in;
     // val *= 50. / 2.;

      val = wxMax(val, 0.0);

      wxColour c;
      if((val >= 0) && (val < .25))            c.Set(_T("#002ad9"));
      else if((val >= 0.25) && (val < 0.50))   c.Set(_T("#006ed9"));
      else if((val >= 0.50) && (val < 1.00))   c.Set(_T("#00b2d9"));
      else if((val >= 1.00) && (val < 1.25))   c.Set(_T("#00d4d4"));
      else if((val >= 1.25) && (val < 1.50))   c.Set(_T("#00d9a6"));
      else if((val >= 1.50) && (val < 1.75))   c.Set(_T("#00d900"));
      else if((val >= 1.75) && (val < 2.00))   c.Set(_T("#95d900"));
      else if((val >= 2.00) && (val < 2.25))   c.Set(_T("#d9d900"));
      else if((val >= 2.25) && (val < 2.50))   c.Set(_T("#d9ae00"));
      else if((val >= 2.50) && (val < 2.75))   c.Set(_T("#d98300"));
      else if((val >= 2.75) && (val < 3.00))   c.Set(_T("#d95700"));
      else if((val >= 3.00) && (val < 3.25))   c.Set(_T("#d90000"));
      else if((val >= 3.25) && (val < 3.50))   c.Set(_T("#ae0000"));
      else if((val >= 3.50) && (val < 3.75))   c.Set(_T("#8c0000"));
      else if((val >= 3.75) && (val < 4.00))   c.Set(_T("#870000"));
      else if((val >= 4.00) && (val < 4.25))   c.Set(_T("#690000"));
      else if((val >= 4.25) && (val < 4.50))   c.Set(_T("#550000"));
      else if( val >= 4.50)                    c.Set(_T("#410000"));

      return c;
}


// Calculates if two boxes intersect. If so, the function returns _ON.
// If they do not intersect, two scenario's are possible:
// other is outside this -> return _OUT
// other is inside this -> return _IN
OVERLAP Intersect(PlugIn_ViewPort *vp,
       double lat_min, double lat_max, double lon_min, double lon_max, double Marge)
{

    if (((vp->lon_min - Marge) > (lon_max + Marge)) ||
         ((vp->lon_max + Marge) < (lon_min - Marge)) ||
         ((vp->lat_max + Marge) < (lat_min - Marge)) ||
         ((vp->lat_min - Marge) > (lat_max + Marge)))
        return _OUT;

    // Check if other.bbox is inside this bbox
    if ((vp->lon_min <= lon_min) &&
         (vp->lon_max >= lon_max) &&
         (vp->lat_max >= lat_max) &&
         (vp->lat_min <= lat_min))
        return _IN;

    // Boundingboxes intersect
    return _ON;
}

