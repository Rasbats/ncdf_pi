/******************************************************************************
 * $Id: ncdf_pi.h,v 1.8 2010/06/21 01:54:37 bdbcat Exp $
 *
 * Project:  OpenCPN
 * Purpose:  ncdf Plugin
 * Author:   David Register
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

#ifndef _ncdfPI_H_
#define _ncdfPI_H_

#include "wx/wxprec.h"

#ifndef  WX_PRECOMP
  #include "wx/wx.h"
#endif //precompiled headers

#define     PLUGIN_VERSION_MAJOR    0
#define     PLUGIN_VERSION_MINOR    2

#define     MY_API_VERSION_MAJOR    1
#define     MY_API_VERSION_MINOR    7

#include "ocpn_plugin.h"

#include "ncdfoverlayfactory.h"

//----------------------------------------------------------------------------------------------------------
//    The PlugIn Class Definition
//----------------------------------------------------------------------------------------------------------

#define ncdf_TOOL_POSITION    -1          // Request default positioning of toolbar tool

class ncdf_pi : public opencpn_plugin_17
{
public:
      ncdf_pi(void *ppimgr);

//    The required PlugIn Methods
      int Init(void);
      bool DeInit(void);

      int GetAPIVersionMajor();
      int GetAPIVersionMinor();
      int GetPlugInVersionMajor();
      int GetPlugInVersionMinor();
      wxBitmap *GetPlugInBitmap();
      wxString GetCommonName();
      wxString GetShortDescription();
      wxString GetLongDescription();

//    The required override PlugIn Methods
      bool RenderOverlay(wxDC &pmdc, PlugIn_ViewPort *vp);
	  bool RenderGLOverlay(wxGLContext *pcontext, PlugIn_ViewPort *vp);
      void SetCursorLatLon(double lat, double lon);


      void SetDefaults(void);
      int GetToolbarToolCount(void);     
      void OnToolbarToolCallback(int id);
      void OnncdfDialogClose();      
      ncdfOverlayFactory *GetncdfOverlayFactory(){ return m_pncdfOverlayFactory; }

      wxString         m_ncdf_dir;
      
private:
      bool LoadConfig(void);
      bool SaveConfig(void);

      wxFileConfig     *m_pconfig;
      wxWindow         *m_parent_window;

      MainDialog		*m_pncdfDialog;
      ncdfOverlayFactory *m_pncdfOverlayFactory;

      int              m_display_width, m_display_height;
      int              m_leftclick_tool_id;

      int              m_ncdf_dialog_x, m_ncdf_dialog_y;
      int              m_ncdf_dialog_sx, m_ncdf_dialog_sy;

      bool              m_bncdfUseHiDef;
      bool              m_bncdfShowIcon;


      //    Controls added to Preferences panel
      wxCheckBox              *m_pncdfShowIcon;
      wxCheckBox              *m_pncdfUseHiDef;

};

#endif



