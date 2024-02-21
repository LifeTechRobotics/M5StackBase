
// ESPracerDlg.h : ヘッダー ファイル
//

#pragma once

#include "CSearchDlg.h"
#include "CWiFiConfigDlg.h"


// CESPracerDlg ダイアログ
class CESPracerDlg : public CDialogEx
{
// コンストラクション
public:
	CESPracerDlg(CWnd* pParent = nullptr);	// 標準コンストラクター

// ダイアログ データ
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ESPRACER_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV サポート


// 実装
protected:
	HICON m_hIcon;

	// 生成された、メッセージ割り当て関数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	CEdit m_EditESPRacerStatus;
	CIPAddressCtrl m_IPAddress;
	CEdit m_EditSpeedR;
	CEdit m_EditSpeedL;
	CEdit m_EditLed1;
	CEdit m_EditLed2;
	CButton m_BtnConnect;
	CButton m_BtnSearch;
	CButton m_BtnConfig;
	CButton m_BtnUpdate;
	CButton m_CheckJoystick;
	CButton m_CheckRevUpDown;
	CButton m_CheckRevRightLeft;

	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnDestroy();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnTimer(UINT_PTR nIDEvent);

	CWinThread* m_pThreadESPClient;
	BOOL m_Terminate;
	BOOL m_ConnectFlag;
	int m_iSpeedR;
	int m_iSpeedL;
	int m_iLed1;
	int m_iLed2;
	char m_sIPAddress[32];

	CSearchDlg m_DlgSearch;
	CWiFiConfigDlg m_DlgConfig;

	// 以下ジョイスティックデータ DS版
	bool m_JoyStickActive;				// ジョイスティック接続
	int m_LX;							// X軸
	int m_LY;							// Y軸
	int m_RX;							// Z軸
	int m_RY;
	BOOL m_bStaticL1;					// L
	BOOL m_bStaticL2;
	BOOL m_bStaticR1;					// R
	BOOL m_bStaticR2;
	BOOL m_bStaticShare;
	BOOL m_bStaticOption;
	BOOL m_bStaticUP;
	BOOL m_bStaticLeft;
	BOOL m_bStaticRight;
	BOOL m_bStaticDown;
	BOOL m_bStaticSqa;				// ボタン1
	BOOL m_bStaticCross;			// ボタン2
	BOOL m_bStaticCir;				// ボタン3
	BOOL m_bStaticTri;				// ボタン4
	BOOL m_bStaticPS;
	BOOL m_bStaticPad;
	DWORD m_Pad;
};
