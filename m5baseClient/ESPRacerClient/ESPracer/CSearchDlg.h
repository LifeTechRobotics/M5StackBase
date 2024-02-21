#pragma once


// CSearchDlg ダイアログ

class CSearchDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CSearchDlg)

public:
	CSearchDlg(CWnd* pParent = nullptr);   // 標準コンストラクター
	virtual ~CSearchDlg();

// ダイアログ データ
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SEARCH_DIALOG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV サポート

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedCancel();
	afx_msg void OnBnClickedOk();

	CListBox m_ListIP;
	CEdit m_EditIP1;
	CEdit m_EditIP2;
	CEdit m_EditIP3;
	CEdit m_EditIP4;
	CEdit m_EditIP5;
	CEdit m_EditSearchIP;
	CButton m_BtnSearch;
	CButton m_BtnClose;

	bool m_fSearching;
	int m_iIP1;
	int m_iIP2;
	int m_iIP3;
	int m_iIP4;
	int m_iIP5;
	int m_iIPVal;
};
