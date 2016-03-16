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
           // ClearCachedData();
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
		 // g2data.ucurr[mi] = NAN;
	  //wxMessageBox(_T("here"));
	 // return;
		  if (g2data.ucurr[mi] != NULL && g2data.vcurr[mi] != NULL){
			  GetCanvasPixLL(vp, &p, lat, lon);
			  if (PointInLLBox(vp, lon, lat))
			  {
				 
					  //double force = sqrt(ucurr[mi] * ucurr[mi] + vcurr[mi] * vcurr[mi])*3.6 / 1.852;
					  double dir = 90. + (atan2(g2data.vcurr[mi], -g2data.ucurr[mi])  * 180. / PI);
					  if (dir < 0) dir = 360 + dir;

					 // wxMessageBox(_T("here"));
					//  return;
					  drawWaveArrow(p.x, p.y, dir - 90, colour);
					
			  }
		  }
	  }
}


bool ncdfOverlayFactory::RenderncdfCurrentBmp()
{
	//wxMessageBox(_T("here in bmp"));

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
                        //    This could take a while....
			      double **currentDir = (double **) g2data.ucurr;
				  if (currentDir == NULL){ 
					  wxMessageBox(_T("null"));
					  return false;
				  }

			      
			      double **currentForce = (double **) g2data.vcurr;
				  if (currentForce == NULL){
					 // wxMessageBox(_T("null"));
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
//wxMessageBox(_T("here in m_pdc"));
//return;
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
 /*
      wxPen pen( arrowColor, 1);
      pmdc->SetPen(pen);
      pmdc->SetBrush(*wxTRANSPARENT_BRUSH);
*/
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

	  /*
#if wxUSE_GRAPHICS_CONTEXT
     // if(0g_bncdfUseHiDef)
      {
       ;  if(m_pgc)
            {
                  m_pgc->SetPen(pen);
                  m_pgc->StrokeLine(ii, jj, kk, ll);
            }
      }
      else
      {
		  if (m_pdc){
			  m_pdc->SetPen(pen);
			  m_pdc->SetBrush(*wxTRANSPARENT_BRUSH);
			  m_pdc->DrawLine(ii, jj, kk, ll);
			  
		  }
		//  else{
		//	 DrawGLLine(ii, jj, kk, ll, 1, colour);
		//  }                  
      }

#else
	  if (m_pdc){
		  m_pdc->SetPen(pen);
		  m_pdc->SetBrush(*wxTRANSPARENT_BRUSH);
		  m_pdc->DrawLine(ii, jj, kk, ll);

	  }
	  else{
		  DrawGLLine(ii, jj, kk, ll, 2, colour);
	  }
#endif
*/
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

