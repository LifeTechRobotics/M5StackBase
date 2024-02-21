// CSearchDlg.cpp : 実装ファイル
//

#include "pch.h"
#include "ESPracer.h"
#include "CSearchDlg.h"
#include "afxdialogex.h"

// CSearchDlg ダイアログ

IMPLEMENT_DYNAMIC(CSearchDlg, CDialogEx)

CSearchDlg::CSearchDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_SEARCH_DIALOG, pParent)
{
	m_fSearching = false;

	m_iIP1 = 0;
	m_iIP2 = 0;
	m_iIP3 = 0;
	m_iIP4 = 0;
	m_iIP5 = 0;
	m_iIPVal = 0;
}

CSearchDlg::~CSearchDlg()
{
}

void CSearchDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST1, m_ListIP);
	DDX_Control(pDX, IDC_EDIT1, m_EditIP1);
	DDX_Control(pDX, IDC_EDIT2, m_EditIP2);
	DDX_Control(pDX, IDC_EDIT7, m_EditIP3);
	DDX_Control(pDX, IDC_EDIT8, m_EditIP4);
	DDX_Control(pDX, IDC_EDIT9, m_EditIP5);
	DDX_Control(pDX, IDOK, m_BtnSearch);
	DDX_Control(pDX, IDCANCEL, m_BtnClose);
	DDX_Control(pDX, IDC_EDIT3, m_EditSearchIP);
}


BEGIN_MESSAGE_MAP(CSearchDlg, CDialogEx)
	ON_BN_CLICKED(IDCANCEL, &CSearchDlg::OnBnClickedCancel)
	ON_BN_CLICKED(IDOK, &CSearchDlg::OnBnClickedOk)
END_MESSAGE_MAP()


// CSearchDlg メッセージ ハンドラー


void CSearchDlg::OnBnClickedCancel()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	// Close
	//CDialogEx::OnCancel();
	GetParent()->EnableWindow(TRUE);

	m_fSearching = false;
	ShowWindow(SW_HIDE);
}

static char* REQDATA = "abcdefghijklmnop";

void CSearchDlg::OnBnClickedOk()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	// 検索
//	CDialogEx::OnOK();
	CString str;

	m_EditIP1.GetWindowTextW(str);
	m_iIP1 = _tstoi(str);
	m_EditIP2.GetWindowTextW(str);
	m_iIP2 = _tstoi(str);
	m_EditIP3.GetWindowTextW(str);
	m_iIP3 = _tstoi(str);
	m_EditIP4.GetWindowTextW(str);
	m_iIP4 = _tstoi(str);
	m_EditIP5.GetWindowTextW(str);
	m_iIP5 = _tstoi(str);

	m_iIPVal = m_iIP4;

	m_ListIP.ResetContent();

	// 検索開始フラグ設定
	// メインタイマー内で検索する
	m_fSearching = !m_fSearching;

	m_BtnSearch.EnableWindow(FALSE);
}
