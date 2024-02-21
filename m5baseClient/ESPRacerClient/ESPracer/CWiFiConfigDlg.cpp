// CWiFiConfigDlg.cpp : 実装ファイル
//

#include "pch.h"
#include "ESPracer.h"
#include "CWiFiConfigDlg.h"
#include "afxdialogex.h"


// CWiFiConfigDlg ダイアログ

IMPLEMENT_DYNAMIC(CWiFiConfigDlg, CDialogEx)

CWiFiConfigDlg::CWiFiConfigDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_CONFIG_DIALOG, pParent)
{
	m_fSending = false;
}

CWiFiConfigDlg::~CWiFiConfigDlg()
{
}

void CWiFiConfigDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_IPADDRESS1, m_IPAddress);
	DDX_Control(pDX, IDC_IPADDRESS2, m_GatewayAddress);
	DDX_Control(pDX, IDC_BUTTON1, m_BtnIP);
	DDX_Control(pDX, IDC_BUTTON2, m_BtnGateway);
	DDX_Control(pDX, IDC_BUTTON4, m_BtnReset);
	DDX_Control(pDX, IDC_BUTTON5, m_BtnErase);
	DDX_Control(pDX, IDCANCEL, m_BtnClose);
}


BEGIN_MESSAGE_MAP(CWiFiConfigDlg, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON1, &CWiFiConfigDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CWiFiConfigDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON4, &CWiFiConfigDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &CWiFiConfigDlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDCANCEL, &CWiFiConfigDlg::OnBnClickedCancel)
END_MESSAGE_MAP()


// CWiFiConfigDlg メッセージ ハンドラー


void CWiFiConfigDlg::OnBnClickedButton1()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	// IP設定
	if (m_fSending) {
		return;
	}
	BYTE ip1, ip2, ip3, ip4;
	m_IPAddress.GetAddress(ip1, ip2, ip3, ip4);

	m_iIP1 = ip1;
	m_iIP2 = ip2;
	m_iIP3 = ip3;
	m_iIP4 = ip4;

	m_fSending = true;
	m_fSendingIP = true;
}


void CWiFiConfigDlg::OnBnClickedButton2()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	// Gateway設定
	if (m_fSending) {
		return;
	}

	BYTE ip1, ip2, ip3, ip4;
	m_GatewayAddress.GetAddress(ip1, ip2, ip3, ip4);

	m_iIP1 = ip1;
	m_iIP2 = ip2;
	m_iIP3 = ip3;
	m_iIP4 = ip4;

	m_fSending = true;
	m_fSendingGateway = true;
}


void CWiFiConfigDlg::OnBnClickedButton4()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	// Reset
	if (m_fSending) {
		return;
	}

	m_fSending = true;
	m_fSendingReset = true;
}


void CWiFiConfigDlg::OnBnClickedButton5()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	// WiFi Erase
	if (m_fSending) {
		return;
	}

	m_fSending = true;
	m_fSendingErase = true;
}


void CWiFiConfigDlg::OnBnClickedCancel()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	// 閉じる
	//CDialogEx::OnCancel();
	GetParent()->EnableWindow(TRUE);
	ShowWindow(SW_HIDE);
}
