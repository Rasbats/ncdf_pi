///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep  8 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __gui__
#define __gui__

#include <wx/intl.h>

#include <wx/string.h>
#include <wx/textctrl.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/bmpbuttn.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/treectrl.h>
#include <wx/stattext.h>
#include <wx/statline.h>
#include <wx/listbox.h>
#include <wx/checkbox.h>
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/notebook.h>
#include <wx/dialog.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class ncdfDialog
///////////////////////////////////////////////////////////////////////////////
class ncdfDialog : public wxDialog 
{
	private:
	
	protected:
		wxNotebook* m_notebook1;
		wxPanel*    m_panel1;
		wxStaticLine* m_staticline1;
		wxBitmapButton* m_bpPrev;
		wxBitmapButton* m_bpNext;
		wxChoice*     m_choiceArea;
		wxStaticLine* m_staticline11;		
		wxStaticText* m_staticText34;
		wxStaticText* m_staticText24;
		
		wxStaticText* m_staticTextWaveHeight;
		wxStaticText* m_staticText37;
		
		wxStaticText* m_staticText32;
		wxStaticText* m_staticText33;
		wxStaticText* m_staticText331;
		wxStaticLine* m_staticline6;
		
		wxStaticText* m_staticText35;
		wxStaticText* m_staticText332;
		
		wxStaticText* m_staticText3321;
		
		wxStaticText* m_staticText26;
		wxStaticLine* m_staticline7;
		
		wxStaticText* m_staticText333;
		wxStaticText* m_staticText341;
		
		
		wxStaticText* m_staticText40;
		wxStaticText* m_staticText41;
		wxPanel* m_panel2;
		wxStaticText* m_staticText6;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onCloseDialog( wxCloseEvent& event ) { event.Skip(); }
		virtual void OnCharNoteBook1( wxKeyEvent& event ) { event.Skip(); }
		virtual void onPageChanged( wxNotebookEvent& event ) { event.Skip(); }
		virtual void onDirChanged( wxCommandEvent& event ) { event.Skip(); }
		virtual void onFileButtonClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void onAreaChange(wxCommandEvent& event) { event.Skip(); }
		virtual void onTreeItemRightClick( wxTreeEvent& event ) { event.Skip(); }
		virtual void onTreeSelectionChanged( wxTreeEvent& event ) { event.Skip(); }
		virtual void onDCurrentClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void onBmpCurrentForceClick( wxCommandEvent& event ) { event.Skip(); }
		virtual void onButtonSelectionReset( wxCommandEvent& event ) { event.Skip(); }
		virtual void onDownLoadOK( wxCommandEvent& event ) { event.Skip(); }
		virtual void onPrev(wxCommandEvent& event) { event.Skip(); }
		virtual void onNext(wxCommandEvent& event) { event.Skip(); }
		
	
	public:
		wxTextCtrl* m_textCtrlDir;
		wxBitmapButton* m_fileButton;
		wxTreeCtrl* m_treeCtrl;
		wxStaticText* m_staticTextDateTime;
		wxCheckBox* m_checkBoxDCurrent;
		wxTextCtrl* m_textCtrlCurrentDir;
		wxCheckBox* m_checkBoxBmpCurrentForce;
		wxTextCtrl* m_textCtrlCurrentForce;
		
		ncdfDialog( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = _("ncdf"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 270,460 ), long style =wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER );
		~ncdfDialog();	
};

#endif //__gui__
