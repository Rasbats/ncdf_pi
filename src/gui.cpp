///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep  8 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "gui.h"

#include "folder.xpm"

///////////////////////////////////////////////////////////////////////////

ncdfDialog::ncdfDialog( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxFlexGridSizer* fgSizer3;
	fgSizer3 = new wxFlexGridSizer( 1, 1, 0, 0 );
	fgSizer3->SetFlexibleDirection( wxBOTH );
	fgSizer3->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_notebook1 = new wxNotebook( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );
	m_panel1 = new wxPanel( m_notebook1, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	m_panel1->SetBackgroundColour( wxSystemSettings::GetColour( wxSYS_COLOUR_3DLIGHT ) );
	
	wxFlexGridSizer* fgSizer1;
	fgSizer1 = new wxFlexGridSizer( 3, 1, 0, 0 );
	fgSizer1->SetFlexibleDirection( wxBOTH );
	fgSizer1->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_NONE );
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxHORIZONTAL );
	
	m_textCtrlDir = new wxTextCtrl( m_panel1, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY );
	m_textCtrlDir->SetMinSize( wxSize( 180,-1 ) );
	
	bSizer2->Add( m_textCtrlDir, 0, wxALL, 5 );
	
	m_fileButton = new wxBitmapButton( m_panel1, wxID_ANY, wxBitmap(folder), wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW );
	bSizer2->Add( m_fileButton, 0, wxALL, 5 );
	
	fgSizer1->Add( bSizer2, 1, wxEXPAND, 5 );
	
	m_treeCtrl = new wxTreeCtrl( m_panel1, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTR_DEFAULT_STYLE|wxHSCROLL|wxSUNKEN_BORDER );
	m_treeCtrl->SetFont( wxFont( 9, 74, 90, 90, false, wxT("Sans Bold") ) );
	m_treeCtrl->SetMinSize( wxSize( -1,150 ) );
	
	fgSizer1->Add( m_treeCtrl, 0, wxALL|wxEXPAND, 5 );
	
	m_staticTextDateTime = new wxStaticText( m_panel1, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	m_staticTextDateTime->Wrap( -1 );
	fgSizer1->Add(m_staticTextDateTime, 1, wxALL, 5);


	wxFlexGridSizer* fgSizer7;
	fgSizer7 = new wxFlexGridSizer(1, 2, 0, 0);
	fgSizer7->AddGrowableRow(1);
	fgSizer7->SetFlexibleDirection(wxHORIZONTAL);
	fgSizer7->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_ALL);

	wxBoxSizer* bSizer15;
	bSizer15 = new wxBoxSizer(wxHORIZONTAL);


	bSizer15->Add(40, 0, 1, wxEXPAND, 5);

	m_bpPrev = new wxBitmapButton(m_panel1, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW);
	bSizer15->Add(m_bpPrev, 0, wxALL, 5);


	bSizer15->Add(24, 0, 1, wxEXPAND, 5);

	m_bpNext = new wxBitmapButton(m_panel1, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW);
	bSizer15->Add(m_bpNext, 0, wxALL, 5);


	fgSizer7->Add(bSizer15, 1, wxEXPAND, 5);


	fgSizer1->Add(fgSizer7, 1, wxEXPAND, 5);

	
	m_staticline1 = new wxStaticLine( m_panel1, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	fgSizer1->Add( m_staticline1, 0, wxEXPAND | wxALL, 5 );
	
//	wxBoxSizer* bSizer131;
//	bSizer131 = new wxBoxSizer(wxHORIZONTAL);

	wxString m_areaChoices[] = { _T("English Channel"), _T("South Brittany"), _T("Irish Sea"), 
		_T("Western Ireland"),	_T("Western Scotland"), _T("North Sea - South"), 
		_T("North Sea - North"), _T("Biscay East"),	_T("Biscay South"), 
		_T("Orkney/Shetlands")};

	int m_areaNChoices = sizeof(m_areaChoices) / sizeof(wxString);

	m_choiceArea = new wxChoice(m_panel1, wxID_ANY, wxDefaultPosition, wxDefaultSize, m_areaNChoices, m_areaChoices, 0);
	m_choiceArea->SetSelection(0);

	fgSizer1->Add(m_choiceArea, 0, wxALL | wxEXPAND, 5);

	m_staticline11 = new wxStaticLine(m_panel1, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL);
	fgSizer1->Add(m_staticline11, 0, wxEXPAND | wxALL, 5);

	//fgSizer1->Add(bSizer131, wxEXPAND, 5);

	wxFlexGridSizer* fgSizer2;
	fgSizer2 = new wxFlexGridSizer( 5, 1, 0, 0 );
	fgSizer2->SetFlexibleDirection( wxBOTH );
	fgSizer2->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxHORIZONTAL );
	bSizer12->Add( 24, 0, 0, wxEXPAND, 0 );

	wxBoxSizer* bSizer13;
	bSizer13 = new wxBoxSizer( wxHORIZONTAL );	
	bSizer13->Add( 24, 0, 0, wxEXPAND, 0 );

	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer( wxHORIZONTAL );
	bSizer11->Add(24, 0, 0, wxEXPAND, 0);

	m_checkBoxDCurrent = new wxCheckBox( m_panel1, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	bSizer11->Add( m_checkBoxDCurrent, 0, wxALL|wxALIGN_CENTER_VERTICAL, 0 );
	m_checkBoxDCurrent->SetValue(true);

	m_staticText333 = new wxStaticText( m_panel1, wxID_ANY, _("  Current"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText333->Wrap( -1 );
	m_staticText333->SetMinSize( wxSize( 80,-1 ) );
	
	bSizer11->Add( m_staticText333, 0, wxALL|wxALIGN_CENTER_VERTICAL, 0 );
	
	m_textCtrlCurrentDir = new wxTextCtrl( m_panel1, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_READONLY|wxTE_RIGHT );
	bSizer11->Add( m_textCtrlCurrentDir, 0, wxALL, 0 );
	
	m_staticText341 = new wxStaticText( m_panel1, wxID_ANY, _("Deg"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText341->Wrap( -1 );
	bSizer11->Add( m_staticText341, 0, wxALL, 5 );
	
	fgSizer2->Add( bSizer11, 1, wxEXPAND, 5 );
	
	
	fgSizer2->Add( 0, 0, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxHORIZONTAL );	
	bSizer14->Add( 24, 0, 0, wxEXPAND, 0 );
	
	m_checkBoxBmpCurrentForce = new wxCheckBox( m_panel1, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	bSizer14->Add( m_checkBoxBmpCurrentForce, 0, wxALL, 0 );
	
	m_staticText40 = new wxStaticText( m_panel1, wxID_ANY, _("  Force"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText40->Wrap( -1 );
	m_staticText40->SetMinSize( wxSize( 80,-1 ) );
	
	bSizer14->Add( m_staticText40, 0, wxALL|wxALIGN_CENTER_VERTICAL, 0 );
	
	m_textCtrlCurrentForce = new wxTextCtrl( m_panel1, wxID_ANY, wxEmptyString, wxDefaultPosition, wxSize( 70,-1 ), wxTE_READONLY|wxTE_RIGHT );
	bSizer14->Add( m_textCtrlCurrentForce, 0, wxALL, 0 );
	
	m_staticText41 = new wxStaticText( m_panel1, wxID_ANY, _("kts"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText41->Wrap( -1 );
	bSizer14->Add( m_staticText41, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	fgSizer2->Add( bSizer14, 1, wxEXPAND, 5 );
	
	fgSizer1->Add( fgSizer2, 1, wxEXPAND, 0 );
	
	m_panel1->SetSizer( fgSizer1 );
	m_panel1->Layout();
	fgSizer1->Fit( m_panel1 );
	m_notebook1->AddPage( m_panel1, _("Data"), true );
	m_panel2 = new wxPanel( m_notebook1, wxID_ANY, wxPoint( -1,-1 ), wxDefaultSize, wxTAB_TRAVERSAL );
	m_panel2->SetBackgroundColour( wxSystemSettings::GetColour( wxSYS_COLOUR_3DLIGHT ) );
	
	wxFlexGridSizer* fgSizer4;
	fgSizer4 = new wxFlexGridSizer( 1, 1, 0, 0 );
	fgSizer4->SetFlexibleDirection( wxBOTH );
	fgSizer4->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	wxFlexGridSizer* fgSizer6;
	fgSizer6 = new wxFlexGridSizer( 2, 3, 0, 0 );
	fgSizer6->SetFlexibleDirection( wxBOTH );
	fgSizer6->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	m_panel2->SetSizer( fgSizer4 );
	m_panel2->Layout();
	fgSizer4->Fit( m_panel2 );
	m_notebook1->AddPage( m_panel2, _("Download"), false );
	
	fgSizer3->Add( m_notebook1, 1, wxEXPAND | wxALL, 5 );
	
	this->SetSizer( fgSizer3 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( ncdfDialog::onCloseDialog ) );
	m_notebook1->Connect( wxEVT_CHAR, wxKeyEventHandler( ncdfDialog::OnCharNoteBook1 ), NULL, this );
	m_notebook1->Connect( wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGED, wxNotebookEventHandler( ncdfDialog::onPageChanged ), NULL, this );
	m_textCtrlDir->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( ncdfDialog::onDirChanged ), NULL, this );
	m_fileButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ncdfDialog::onFileButtonClick ), NULL, this );
	m_treeCtrl->Connect( wxEVT_COMMAND_TREE_ITEM_RIGHT_CLICK, wxTreeEventHandler( ncdfDialog::onTreeItemRightClick ), NULL, this );
	m_treeCtrl->Connect( wxEVT_COMMAND_TREE_SEL_CHANGED, wxTreeEventHandler( ncdfDialog::onTreeSelectionChanged ), NULL, this );
	m_checkBoxDCurrent->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ncdfDialog::onDCurrentClick ), NULL, this );
	m_checkBoxBmpCurrentForce->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ncdfDialog::onBmpCurrentForceClick ), NULL, this );
	m_bpPrev->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(ncdfDialog::onPrev), NULL, this);
	m_bpNext->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(ncdfDialog::onNext), NULL, this);

}

ncdfDialog::~ncdfDialog()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( ncdfDialog::onCloseDialog ) );
	m_notebook1->Disconnect( wxEVT_CHAR, wxKeyEventHandler( ncdfDialog::OnCharNoteBook1 ), NULL, this );
	m_notebook1->Disconnect( wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGED, wxNotebookEventHandler( ncdfDialog::onPageChanged ), NULL, this );
	m_textCtrlDir->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( ncdfDialog::onDirChanged ), NULL, this );
	m_fileButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ncdfDialog::onFileButtonClick ), NULL, this );
	m_treeCtrl->Disconnect( wxEVT_COMMAND_TREE_ITEM_RIGHT_CLICK, wxTreeEventHandler( ncdfDialog::onTreeItemRightClick ), NULL, this );
	m_treeCtrl->Disconnect( wxEVT_COMMAND_TREE_SEL_CHANGED, wxTreeEventHandler( ncdfDialog::onTreeSelectionChanged ), NULL, this );
	m_checkBoxDCurrent->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ncdfDialog::onDCurrentClick ), NULL, this );
	m_checkBoxBmpCurrentForce->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ncdfDialog::onBmpCurrentForceClick ), NULL, this );	
	m_bpPrev->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(ncdfDialog::onPrev), NULL, this);
	m_bpNext->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(ncdfDialog::onNext), NULL, this);


}
