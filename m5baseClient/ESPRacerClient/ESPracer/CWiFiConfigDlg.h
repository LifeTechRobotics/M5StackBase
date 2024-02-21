#pragma once


// CWiFiConfigDlg ダイアログ

class CWiFiConfigDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CWiFiConfigDlg)

public:
	CWiFiConfigDlg(CWnd* pParent = nullptr);   // 標準コンストラクター
	virtual ~CWiFiConfigDlg();

// ダイアログ データ
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_CONFIG_DIALOG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV サポート

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton5();
	afx_msg void OnBnClickedCancel();

	CIPAddressCtrl m_IPAddress;
	CIPAddressCtrl m_GatewayAddress;
	CButton m_BtnIP;
	CButton m_BtnGateway;
	CButton m_BtnReset;
	CButton m_BtnErase;
	CButton m_BtnClose;

	bool m_fSending;
	bool m_fSendingIP;
	bool m_fSendingGateway;
	bool m_fSendingReset;
	bool m_fSendingErase;

	int m_iIP1;
	int m_iIP2;
	int m_iIP3;
	int m_iIP4;
};
