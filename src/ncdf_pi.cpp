/******************************************************************************
 * $Id: ncdf_pi.cpp,v 1.8 2010/06/21 01:54:37 bdbcat Exp $
 *
 * Project:  OpenCPN
 * Purpose:  ncdf Plugin
 * Author:   David Register, Mike Rossiter
 *
 ***************************************************************************
 *   Copyright (C) 2010 by David S. Register   *
 *   bdbcat@yahoo.com   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************
 */


#include "wx/wxprec.h"

#ifndef  WX_PRECOMP
  #include "wx/wx.h"
#endif //precompiled headers


#include <wx/treectrl.h>
#include <wx/fileconf.h>
#include <wx/stdpaths.h>

#include "ncdf_pi.h"
#include "ncdf.h"



// the class factories, used to create and destroy instances of the PlugIn

extern "C" DECL_EXP opencpn_plugin* create_pi(void *ppimgr)
{
    return new ncdf_pi(ppimgr);
}

extern "C" DECL_EXP void destroy_pi(opencpn_plugin* p)
{
    delete p;
}


//---------------------------------------------------------------------------------------------------------
//
//    ncdf PlugIn Implementation
//
//---------------------------------------------------------------------------------------------------------

#include "icons2.h"

//---------------------------------------------------------------------------------------------------------
//
//          PlugIn initialization and de-init
//
//---------------------------------------------------------------------------------------------------------

ncdf_pi::ncdf_pi(void *ppimgr)
      :opencpn_plugin_17(ppimgr)
{
      // Create the PlugIn icons
	initialize_images();
}

int ncdf_pi::Init(void)
{

      AddLocaleCatalog( _T("opencpn-ncdf_pi") );

      // Set some default private member parameters
      m_ncdf_dialog_x = 0;
      m_ncdf_dialog_y = 0;
      m_ncdf_dialog_sx = 270;
      m_ncdf_dialog_sy = 460;
      m_pncdfDialog = NULL;
      m_pncdfOverlayFactory = NULL;
	  ::wxDisplaySize(&m_display_width, &m_display_height);
	  m_choice = 0;
	  b_showODAS = false;

      //    Get a pointer to the opencpn configuration object
      m_pconfig = GetOCPNConfigObject();

      //    And load the configuration items
      LoadConfig();

      // Get a pointer to the opencpn display canvas, to use as a parent for the ncdf dialog
      m_parent_window = GetOCPNCanvasWindow();

      //    This PlugIn needs a toolbar icon, so request its insertion if enabled locally
      if(m_bncdfShowIcon)
            m_leftclick_tool_id  = InsertPlugInTool(_T(""), _img_ncdf, _img_ncdf, wxITEM_NORMAL,
                  _("ncdf"), _T(""), NULL,
                   ncdf_TOOL_POSITION, 0, this);

      // Create the drawing factory

      m_pncdfOverlayFactory = new ncdfOverlayFactory;
	  m_pncdfOverlayFactory->m_bReadyToRender=false;
	  

	  wxMenu dummy_menu;
	  m_position_menu_id = AddCanvasContextMenuItem
		  (new wxMenuItem(&dummy_menu, -1, _("Drop Tidal Arrow")), this);
	  SetCanvasContextMenuItemViz(m_position_menu_id, true);

      return (WANTS_OVERLAY_CALLBACK |
           WANTS_OPENGL_OVERLAY_CALLBACK |
           WANTS_CURSOR_LATLON       |
           WANTS_TOOLBAR_CALLBACK    |
           INSTALLS_TOOLBAR_TOOL     |
           WANTS_CONFIG              
           //WANTS_PREFERENCES
            );
}

bool ncdf_pi::DeInit(void)
{
      if(m_pncdfDialog)
      {
		 m_pncdfOverlayFactory->m_bReadyToRender = false;
		 m_pncdfDialog->Close();
      }
     
	delete m_pncdfOverlayFactory;
	m_pncdfOverlayFactory = NULL;
    return true;
}

int ncdf_pi::GetAPIVersionMajor()
{
      return MY_API_VERSION_MAJOR;
}

int ncdf_pi::GetAPIVersionMinor()
{
      return MY_API_VERSION_MINOR;
}

int ncdf_pi::GetPlugInVersionMajor()
{
      return PLUGIN_VERSION_MAJOR;
}

int ncdf_pi::GetPlugInVersionMinor()
{
      return PLUGIN_VERSION_MINOR;
}

wxBitmap *ncdf_pi::GetPlugInBitmap()
{
      return _img_ncdf_pi;
}

wxString ncdf_pi::GetCommonName()
{
      return _("ncdf");
}


wxString ncdf_pi::GetShortDescription()
{
      return _("ncdf PlugIn for OpenCPN");
}


wxString ncdf_pi::GetLongDescription()
{
      return _("ncdf PlugIn for OpenCPN\n\
For surface current direction and speed.");

}


void ncdf_pi::SetDefaults(void)
{
      // If the config somehow says NOT to show the icon, override it so the user gets good feedback
      if(!m_bncdfShowIcon)
      {
            m_bncdfShowIcon = true;

            m_leftclick_tool_id  = InsertPlugInTool(_T(""), _img_ncdf, _img_ncdf, wxITEM_NORMAL,
                  _("ncdf"), _T(""), NULL,
                   ncdf_TOOL_POSITION, 0, this);
      }
}

void ncdf_pi::OnContextMenuItemCallback(int id)
{
	if (!m_pncdfDialog)
		return;

	if (id == m_position_menu_id)

		m_pncdfDialog->m_cursor_lat = GetCursorLat();
		m_pncdfDialog->m_cursor_lon = GetCursorLon();

		wxString myLat = wxString::Format(wxT("%5.2f"), (double)m_pncdfDialog->m_cursor_lat);

	if (m_pncdfDialog) {
		m_pncdfDialog->OnContextMenu(m_pncdfDialog->m_cursor_lat, m_pncdfDialog->m_cursor_lon);
	}

}
int ncdf_pi::GetToolbarToolCount(void)
{
      return 1;
}

void ncdf_pi::OnToolbarToolCallback(int id)
{
      if(NULL == m_pncdfDialog)
      {
        m_pncdfDialog = new MainDialog(m_parent_window);
	    m_pncdfDialog->setPlugIn(this);
      }
      m_pncdfDialog->Show();                        // Show modeless, so it stays on the screen
	  
}

void ncdf_pi::OnncdfDialogClose()
{
	  m_pncdfDialog->Hide();
      if(m_pncdfOverlayFactory)
      {
        m_pncdfOverlayFactory->reset();
	    RequestRefresh(m_parent_window);
      }
      SaveConfig();     
}

bool ncdf_pi::RenderOverlay(wxDC &pmdc, PlugIn_ViewPort *vp)
{    
	
	if (b_showODAS == true){
		RenderOverlayArrow(&pmdc, vp);
		return true;	
	}
		
    if(m_pncdfDialog && m_pncdfOverlayFactory)
    {
            if(m_pncdfOverlayFactory->isReadyToRender())
            {
				m_pncdfOverlayFactory->SetParentSize(m_display_width, m_display_height);
				m_pncdfOverlayFactory->RenderncdfOverlay ( pmdc, vp );
                  return true;
            }
            else
                  return false;
    }
    else
            return false;
	
}

bool ncdf_pi::RenderGLOverlay(wxGLContext *pcontext, PlugIn_ViewPort *vp)
{
	if (b_showODAS == true){
		RenderGLOverlayArrow(pcontext, vp);
		return true;
	}

	if (m_pncdfDialog && m_pncdfOverlayFactory)
	{
		if (m_pncdfOverlayFactory->isReadyToRender())
		{
			m_pncdfOverlayFactory->SetParentSize(m_display_width, m_display_height);
			m_pncdfOverlayFactory->RenderGLncdfOverlay(pcontext, vp);
			return true;
		}
		else
			return false;
	}
	else
		return false;
}

bool ncdf_pi::RenderOverlayArrow(wxDC *dc, PlugIn_ViewPort *vp)
{
	if (!m_pncdfDialog || !m_pncdfDialog->IsShown())
		return false;

	if (m_pncdfDialog && m_pncdfOverlayFactory)
	{
		for (std::list<Arrow*>::iterator it = m_pncdfDialog->m_ArrowList.begin();
			it != m_pncdfDialog->m_ArrowList.end(); it++){

			double myLat = (*it)->m_lat;
			double myLon = (*it)->m_lon;
			m_pncdfDialog->getCurrentData(myLat, myLon);
			m_pncdfOverlayFactory->DrawAllCurrentsInViewPort(myLat, myLon, ddir, dfor, *dc, vp);
		}
	}

	return true;

}

bool ncdf_pi::RenderGLOverlayArrow(wxGLContext *pcontext, PlugIn_ViewPort *vp)
{
	if (!m_pncdfDialog || !m_pncdfDialog->IsShown())
		return false;
	if (m_pncdfDialog && m_pncdfOverlayFactory)
	{
		for (std::list<Arrow*>::iterator it = m_pncdfDialog->m_ArrowList.begin();
			it != m_pncdfDialog->m_ArrowList.end(); it++){

			double myLat = (*it)->m_lat;
			double myLon = (*it)->m_lon;
			m_pncdfDialog->getCurrentData(myLat, myLon);
			m_pncdfOverlayFactory->DrawAllGLCurrentsInViewPort(myLat, myLon, ddir, dfor, pcontext, vp);
		}
	}

	return true;

}
void ncdf_pi::SetCursorLatLon(double lat, double lon)
{
      if(m_pncdfDialog)
      {
            m_pncdfDialog->SetCursorLatLon(lat, lon);
      }
      
}


bool ncdf_pi::LoadConfig(void)
{
      wxFileConfig *pConf = (wxFileConfig *)m_pconfig;

      if(pConf)
      {
            pConf->SetPath ( _T( "/Settings" ) );
            pConf->Read ( _T( "ncdfUseHiDef" ),  &m_bncdfUseHiDef, 0 );
            pConf->Read ( _T( "ShowncdfIcon" ),  &m_bncdfShowIcon, 1 );


            m_ncdf_dialog_sx = pConf->Read ( _T ( "ncdfDialogSizeX" ), 270L );
            m_ncdf_dialog_sy = pConf->Read ( _T ( "ncdfDialogSizeY" ), 460L );
            m_ncdf_dialog_x =  pConf->Read ( _T ( "GRI2BDialogPosX" ), 20L );
            m_ncdf_dialog_y =  pConf->Read ( _T ( "ncdfDialogPosY" ), 20L );

            if((m_ncdf_dialog_x < 0) || (m_ncdf_dialog_x > m_display_width))
                  m_ncdf_dialog_x = 5;
            if((m_ncdf_dialog_y < 0) || (m_ncdf_dialog_y > m_display_height))
                  m_ncdf_dialog_y = 5;

			pConf->Read(_T("AreaChoice"), &m_choice, 0);

            pConf->SetPath ( _T ( "/Directories" ) );
			wxStandardPathsBase &std = wxStandardPathsBase::Get(); 
			pConf->Read ( _T ( "ncdfDirectory" ), &m_ncdf_dir, std.GetDocumentsDir() );

            return true;
      }
      else
            return false;
}

bool ncdf_pi::SaveConfig(void)
{
      wxFileConfig *pConf = (wxFileConfig *)m_pconfig;

      if(pConf)
      {
            pConf->SetPath ( _T ( "/Settings" ) );

            pConf->Write ( _T ( "ncdfUseHiDef" ), m_bncdfUseHiDef );
            pConf->Write ( _T ( "ShowncdfIcon" ), m_bncdfShowIcon );

            pConf->Write ( _T ( "ncdfDialogSizeX" ),  m_ncdf_dialog_sx );
            pConf->Write ( _T ( "ncdfDialogSizeY" ),  m_ncdf_dialog_sy );
            pConf->Write ( _T ( "ncdfDialogPosX" ),   m_ncdf_dialog_x );
            pConf->Write ( _T ( "ncdfDialogPosY" ),   m_ncdf_dialog_y );

			pConf->Write(_T("AreaChoice"), m_choice);

            pConf->SetPath ( _T ( "/Directories" ) );
            pConf->Write ( _T ( "ncdfDirectory" ), m_ncdf_dir );

            return true;
      }
      else
            return false;
}
