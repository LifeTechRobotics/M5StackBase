
// ESPracerDlg.cpp : 実装ファイル
//
// 2022/08/25版
//

#include "pch.h"
#include "framework.h"
#include "ESPracer.h"
#include "ESPracerDlg.h"
#include "afxdialogex.h"
#include "WinSock2.h"
#include "IPHlpApi.h"
#include "IcmpAPI.h"
#include "utilJoyStick.h"



#pragma comment(lib, "iphlpapi.lib")


#ifdef _DEBUG
#define new DEBUG_NEW
#endif



#define SERVER_PORT					10200			// 通信ポート
#define DEF_IP						"192.168.0.1"	// サーバーIP

#define TIMER_ID_IP					101				// タイマーID
#define TIMER_ID_JOYSTICK			102				// タイマーID

//
// 排他処理
//
CRITICAL_SECTION                    csLock;


// 通信処理用のスレッド
UINT ESPClientThread(LPVOID p)
{
	CESPracerDlg* pAppDlg = (CESPracerDlg*)p;
	unsigned short port = SERVER_PORT;
	SOCKET sockClient = INVALID_SOCKET;
	struct sockaddr_in sockAddrInServer;


	TRACE("ESPClientThread Start ....\n");

	while (!(pAppDlg->m_Terminate)) {
		if (pAppDlg->m_ConnectFlag) {
			pAppDlg->m_EditESPRacerStatus.SetWindowTextW(_T("接続確認中・・・"));
			pAppDlg->m_EditESPRacerStatus.RedrawWindow();

			memset(&sockAddrInServer, 0, sizeof(sockAddrInServer));
			sockAddrInServer.sin_port = htons(port);
			sockAddrInServer.sin_family = AF_INET;
			sockAddrInServer.sin_addr.s_addr = inet_addr(pAppDlg->m_sIPAddress);

			sockClient = socket(AF_INET, SOCK_STREAM, 0);
			if (sockClient == INVALID_SOCKET) {
				pAppDlg->m_ConnectFlag = false;
				pAppDlg->m_BtnConnect.EnableWindow(TRUE);
				pAppDlg->m_BtnConnect.SetWindowTextW(_T("接続"));
				pAppDlg->m_BtnConnect.RedrawWindow();
				pAppDlg->m_EditESPRacerStatus.SetWindowTextW(_T("接続エラー"));
				pAppDlg->m_EditESPRacerStatus.RedrawWindow();
				TRACE("ESPClientThread socket Error !!\n");
				pAppDlg->m_BtnSearch.EnableWindow(TRUE);
				pAppDlg->m_BtnSearch.RedrawWindow();
				continue;
			}

			if (SOCKET_ERROR == connect(sockClient, (struct sockaddr*)&sockAddrInServer, sizeof(sockAddrInServer))) {
				TRACE("ESPClientThread connect Error !!\n");
				closesocket(sockClient);
				pAppDlg->m_ConnectFlag = false;
				sockClient = INVALID_SOCKET;
				pAppDlg->m_BtnConnect.EnableWindow(TRUE);
				pAppDlg->m_BtnConnect.SetWindowTextW(_T("接続"));
				pAppDlg->m_BtnConnect.RedrawWindow();
				pAppDlg->m_EditESPRacerStatus.SetWindowTextW(_T("接続エラー"));
				pAppDlg->m_EditESPRacerStatus.RedrawWindow();
				pAppDlg->m_BtnSearch.EnableWindow(TRUE);
				pAppDlg->m_BtnSearch.RedrawWindow();
				continue;
			}

			TRACE("ESPClientThread Connected Client\n");
			pAppDlg->m_BtnConnect.EnableWindow(TRUE);
			pAppDlg->m_BtnConnect.SetWindowTextW(_T("切断"));
			pAppDlg->m_BtnConnect.RedrawWindow();
			pAppDlg->m_EditESPRacerStatus.SetWindowTextW(_T("接続中"));
			pAppDlg->m_EditESPRacerStatus.RedrawWindow();
			pAppDlg->m_BtnSearch.EnableWindow(FALSE);
			pAppDlg->m_BtnSearch.RedrawWindow();

			pAppDlg->m_BtnConfig.EnableWindow(TRUE);
			pAppDlg->m_BtnConfig.RedrawWindow();
		}
		else {
			if (sockClient != INVALID_SOCKET) {
				closesocket(sockClient);
				pAppDlg->m_ConnectFlag = false;
				pAppDlg->m_BtnConnect.EnableWindow(TRUE);
				pAppDlg->m_BtnConnect.SetWindowTextW(_T("接続"));
				pAppDlg->m_BtnConnect.RedrawWindow();
				pAppDlg->m_BtnSearch.EnableWindow(TRUE);
				pAppDlg->m_BtnSearch.RedrawWindow();
			}
		}

		while (!(pAppDlg->m_Terminate)) {
			if (sockClient != INVALID_SOCKET && pAppDlg->m_ConnectFlag) {
				char buff[256];

				if (pAppDlg->m_DlgConfig.m_fSending) {
					if (pAppDlg->m_DlgConfig.m_fSendingIP) {
						sprintf_s(buff, 255, "###,SIP,%03d,%03d,%03d,%03d\r", pAppDlg->m_DlgConfig.m_iIP1, pAppDlg->m_DlgConfig.m_iIP2, pAppDlg->m_DlgConfig.m_iIP3, pAppDlg->m_DlgConfig.m_iIP4);
						if (send(sockClient, buff, ((int)strlen(buff) + 0), 0) < 0) {
							// 送信エラー
							TRACE("ESPServerThread send Error !!\n");
							pAppDlg->m_ConnectFlag = false;
							pAppDlg->m_BtnConnect.SetWindowTextW(_T("接続"));
							pAppDlg->m_BtnConnect.RedrawWindow();
							pAppDlg->m_BtnSearch.EnableWindow(TRUE);
							pAppDlg->m_BtnSearch.RedrawWindow();

							pAppDlg->m_DlgConfig.m_fSending = false;
							pAppDlg->m_DlgConfig.m_fSendingIP = false;
							break;
						}

						memset(buff, 0, sizeof(buff));
						int len = recv(sockClient, buff, 7, 0);
						TRACE("ESPClientThread recv [%s]", buff);
						pAppDlg->m_DlgConfig.m_fSending = false;
						pAppDlg->m_DlgConfig.m_fSendingIP = false;

						if (len <= 0) {
							TRACE("ESPClientThread recv Error !!\n");
							pAppDlg->m_ConnectFlag = false;
							pAppDlg->m_BtnConnect.SetWindowTextW(_T("接続"));
							pAppDlg->m_BtnConnect.RedrawWindow();
							pAppDlg->m_BtnSearch.EnableWindow(TRUE);
							pAppDlg->m_BtnSearch.RedrawWindow();
							break;
						}
					}
					else if (pAppDlg->m_DlgConfig.m_fSendingGateway) {
						sprintf_s(buff, 255, "###,SGW,%3d,%3d,%03d,%03d\r", pAppDlg->m_DlgConfig.m_iIP1, pAppDlg->m_DlgConfig.m_iIP2, pAppDlg->m_DlgConfig.m_iIP3, pAppDlg->m_DlgConfig.m_iIP4);
						if (send(sockClient, buff, ((int)strlen(buff) + 0), 0) < 0) {
							// 送信エラー
							TRACE("ESPServerThread send Error !!\n");
							pAppDlg->m_ConnectFlag = false;
							pAppDlg->m_BtnConnect.SetWindowTextW(_T("接続"));
							pAppDlg->m_BtnConnect.RedrawWindow();
							pAppDlg->m_BtnSearch.EnableWindow(TRUE);
							pAppDlg->m_BtnSearch.RedrawWindow();

							pAppDlg->m_DlgConfig.m_fSending = false;
							pAppDlg->m_DlgConfig.m_fSendingGateway = false;
							break;
						}

						memset(buff, 0, sizeof(buff));
						int len = recv(sockClient, buff, 7, 0);
						TRACE("ESPClientThread recv [%s]", buff);
						pAppDlg->m_DlgConfig.m_fSending = false;
						pAppDlg->m_DlgConfig.m_fSendingGateway = false;

						if (len <= 0) {
							TRACE("ESPClientThread recv Error !!\n");
							pAppDlg->m_ConnectFlag = false;
							pAppDlg->m_BtnConnect.SetWindowTextW(_T("接続"));
							pAppDlg->m_BtnConnect.RedrawWindow();
							pAppDlg->m_BtnSearch.EnableWindow(TRUE);
							pAppDlg->m_BtnSearch.RedrawWindow();
							break;
						}
					}
					else if (pAppDlg->m_DlgConfig.m_fSendingReset) {
						sprintf_s(buff, 255, "###,RST\r");
						if (send(sockClient, buff, ((int)strlen(buff) + 0), 0) < 0) {
							// 送信エラー
							TRACE("ESPServerThread send Error !!\n");
						}

						pAppDlg->m_DlgConfig.m_fSending = false;
						pAppDlg->m_DlgConfig.m_fSendingReset = false;

						// 切断する
						pAppDlg->m_ConnectFlag = false;
						pAppDlg->m_BtnConnect.SetWindowTextW(_T("接続"));
						pAppDlg->m_BtnConnect.RedrawWindow();
						pAppDlg->m_BtnSearch.EnableWindow(TRUE);
						pAppDlg->m_BtnSearch.RedrawWindow();
						break;
					}
					else if (pAppDlg->m_DlgConfig.m_fSendingErase) {
						sprintf_s(buff, 255, "###,WES\r");
						if (send(sockClient, buff, ((int)strlen(buff) + 0), 0) < 0) {
							// 送信エラー
							TRACE("ESPServerThread send Error !!\n");
						}

						pAppDlg->m_DlgConfig.m_fSending = false;
						pAppDlg->m_DlgConfig.m_fSendingErase = false;

						// 切断する
						pAppDlg->m_ConnectFlag = false;
						pAppDlg->m_BtnConnect.SetWindowTextW(_T("接続"));
						pAppDlg->m_BtnConnect.RedrawWindow();
						pAppDlg->m_BtnSearch.EnableWindow(TRUE);
						pAppDlg->m_BtnSearch.RedrawWindow();
						break;
					}
				}
				else {
					EnterCriticalSection(&csLock);
					sprintf_s(buff, 255, "###,%3d,%3d,%03d,%03d\r", pAppDlg->m_iSpeedR, pAppDlg->m_iSpeedL, pAppDlg->m_iLed1, pAppDlg->m_iLed2);
					LeaveCriticalSection(&csLock);

					if (send(sockClient, buff, ((int)strlen(buff) + 0), 0) < 0) {
						// 送信エラー
						TRACE("ESPServerThread send Error !!\n");
						break;
					}

					memset(buff, 0, sizeof(buff));
					int len = recv(sockClient, buff, 7, 0);
					TRACE("ESPClientThread recv [%s]", buff);
					if (len <= 0) {
						TRACE("ESPClientThread recv Error !!\n");
						pAppDlg->m_ConnectFlag = false;
						pAppDlg->m_BtnConnect.SetWindowTextW(_T("接続"));
						pAppDlg->m_BtnConnect.RedrawWindow();
						pAppDlg->m_BtnSearch.EnableWindow(TRUE);
						pAppDlg->m_BtnSearch.RedrawWindow();
						break;
					}
				}
			}
			else {
				pAppDlg->m_BtnConfig.EnableWindow(FALSE);
				pAppDlg->m_BtnConfig.RedrawWindow();
				pAppDlg->m_EditESPRacerStatus.SetWindowTextW(_T("切断していません"));
				pAppDlg->m_EditESPRacerStatus.RedrawWindow();
				break;
			}
			Sleep(25);					// 通信待ち時間は適宜調整
		}

		// ソケットを閉じる
		if (sockClient != INVALID_SOCKET) {
			closesocket(sockClient);
		}
		Sleep(25);					// 通信待ち時間は適宜調整
	}
	return 0;
}


// アプリケーションのバージョン情報に使われる CAboutDlg ダイアログ

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// ダイアログ データ
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV サポート

// 実装
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CESPracerDlg ダイアログ



CESPracerDlg::CESPracerDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_ESPRACER_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_Terminate = false;
	m_ConnectFlag = false;
	m_pThreadESPClient = NULL;
	m_iSpeedR = 0;
	m_iSpeedL = 0;
	m_iLed1 = 0;
	m_iLed2 = 0;
	sprintf_s(m_sIPAddress, 32, DEF_IP);
}

void CESPracerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT1, m_EditESPRacerStatus);
	DDX_Control(pDX, IDC_IPADDRESS1, m_IPAddress);
	DDX_Control(pDX, IDC_EDIT3, m_EditSpeedR);
	DDX_Control(pDX, IDC_EDIT6, m_EditSpeedL);
	DDX_Control(pDX, IDC_EDIT4, m_EditLed1);
	DDX_Control(pDX, IDC_EDIT5, m_EditLed2);
	DDX_Control(pDX, IDC_BUTTON1, m_BtnConnect);
	DDX_Control(pDX, IDC_BUTTON3, m_BtnSearch);
	DDX_Control(pDX, IDC_BUTTON4, m_BtnConfig);
	DDX_Control(pDX, IDC_CHECK1, m_CheckJoystick);
	DDX_Control(pDX, IDC_BUTTON2, m_BtnUpdate);
	DDX_Control(pDX, IDC_CHECK2, m_CheckRevUpDown);
	DDX_Control(pDX, IDC_CHECK3, m_CheckRevRightLeft);
}

BEGIN_MESSAGE_MAP(CESPracerDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1, &CESPracerDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CESPracerDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDCANCEL, &CESPracerDlg::OnBnClickedCancel)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_BUTTON3, &CESPracerDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &CESPracerDlg::OnBnClickedButton4)
	ON_WM_TIMER()
END_MESSAGE_MAP()


// CESPracerDlg メッセージ ハンドラー

BOOL CESPracerDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// "バージョン情報..." メニューをシステム メニューに追加します。

	// IDM_ABOUTBOX は、システム コマンドの範囲内になければなりません。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// このダイアログのアイコンを設定します。アプリケーションのメイン ウィンドウがダイアログでない場合、
	//  Framework は、この設定を自動的に行います。
	SetIcon(m_hIcon, TRUE);			// 大きいアイコンの設定
	SetIcon(m_hIcon, FALSE);		// 小さいアイコンの設定

	// TODO: 初期化をここに追加します。
	// 通信初期化
	WSADATA wsadata;
	if (0 != WSAStartup(MAKEWORD(2, 0), &wsadata))
	{
		printf("WSAStartup Faild.\n");
		return -1;
	}
	{
		int ip1 = 0;
		int ip2 = 0;
		int ip3 = 0;
		int ip4 = 0;
		sscanf_s(DEF_IP, "%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4);
		m_IPAddress.SetAddress(ip1, ip2, ip3, ip4);
	}
	m_EditSpeedR.SetWindowTextW(_T("0"));
	m_EditSpeedL.SetWindowTextW(_T("0"));
	m_EditLed1.SetWindowTextW(_T("0"));
	m_EditLed2.SetWindowTextW(_T("0"));
	m_EditESPRacerStatus.SetWindowTextW(_T("未接続"));
	m_CheckJoystick.SetCheck(BST_UNCHECKED);

	m_BtnConfig.EnableWindow(FALSE);

	// モードレスダイアログ作成
	m_DlgConfig.Create(IDD_CONFIG_DIALOG, this);
	m_DlgConfig.ShowWindow(SW_HIDE);
	m_DlgSearch.Create(IDD_SEARCH_DIALOG, this);
	m_DlgSearch.ShowWindow(SW_HIDE);

	// 排他制御用
	InitializeCriticalSection(&csLock);

	// JoyStick
	InitJoyStick(theApp.m_hInstance, this->m_hWnd, this);

	// スレッド起動
	m_pThreadESPClient = AfxBeginThread(ESPClientThread, this, 0);

	// 検索用タイマー起動
	SetTimer(TIMER_ID_IP, 1000, NULL);
	SetTimer(TIMER_ID_JOYSTICK, 100, NULL);

	return TRUE;  // フォーカスをコントロールに設定した場合を除き、TRUE を返します。
}

void CESPracerDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// ダイアログに最小化ボタンを追加する場合、アイコンを描画するための
//  下のコードが必要です。ドキュメント/ビュー モデルを使う MFC アプリケーションの場合、
//  これは、Framework によって自動的に設定されます。

void CESPracerDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 描画のデバイス コンテキスト

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// クライアントの四角形領域内の中央
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// アイコンの描画
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// ユーザーが最小化したウィンドウをドラッグしているときに表示するカーソルを取得するために、
//  システムがこの関数を呼び出します。
HCURSOR CESPracerDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CESPracerDlg::OnBnClickedButton1()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	BYTE ip1, ip2, ip3, ip4;
	m_IPAddress.GetAddress(ip1, ip2, ip3, ip4);
	sprintf_s(m_sIPAddress, "%d.%d.%d.%d", ip1, ip2, ip3, ip4);
	m_BtnSearch.EnableWindow(FALSE);
	m_BtnConnect.EnableWindow(FALSE);
	m_ConnectFlag = !m_ConnectFlag;
}


void CESPracerDlg::OnBnClickedButton2()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	CString strSpeedR;
	CString strSpeedL;
	CString strLed1;
	CString strLed2;

	m_EditSpeedR.GetWindowTextW(strSpeedR);
	m_EditSpeedL.GetWindowTextW(strSpeedL);
	m_EditLed1.GetWindowTextW(strLed1);
	m_EditLed2.GetWindowTextW(strLed2);

	EnterCriticalSection(&csLock);
	m_iSpeedR = _tstoi(strSpeedR);
	m_iSpeedL = _tstoi(strSpeedL);
	m_iLed1 = _tstoi(strLed1);
	m_iLed2 = _tstoi(strLed2);

	if (m_iSpeedR < -63) {
		m_iSpeedR = -63;
	}
	else if (m_iSpeedR > 63) {
		m_iSpeedR = 63;
	}
	if (m_iSpeedL < -63) {
		m_iSpeedL = -63;
	}
	else if (m_iSpeedL > 63) {
		m_iSpeedL = 63;
	}
	if (m_iLed1 < 0) {
		m_iLed1 = 0;
	}
	else if (m_iLed1 > 255) {
		m_iLed1 = 255;
	}
	if (m_iLed2 < 0) {
		m_iLed2 = 0;
	}
	else if (m_iLed2 > 255) {
		m_iLed2 = 255;
	}
	LeaveCriticalSection(&csLock);
}


void CESPracerDlg::OnBnClickedCancel()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	CDialogEx::OnCancel();
}


void CESPracerDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO: ここにメッセージ ハンドラー コードを追加します。
	// 通信処理の終了
	WSACleanup();

	// スレッド終了
	if (m_pThreadESPClient) {
		m_Terminate = TRUE;

		if (WaitForSingleObject(m_pThreadESPClient, 10000) == WAIT_TIMEOUT)
		{
			// 終了待ちから指定時間たっても終了しない場合スレッド強制停止
			TerminateThread(m_pThreadESPClient, false);
		}
		m_pThreadESPClient = NULL;
		m_Terminate = FALSE;
	}

	// 排他制御解放
	DeleteCriticalSection(&csLock);
}


void CESPracerDlg::OnBnClickedButton3()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	CString str;
	BYTE ip1, ip2, ip3, ip4;
	m_IPAddress.GetAddress(ip1, ip2, ip3, ip4);

	str.Format(_T("%d"), ip1);
	m_DlgSearch.m_EditIP1.SetWindowTextW(str);
	str.Format(_T("%d"), ip2);
	m_DlgSearch.m_EditIP2.SetWindowTextW(str);
	str.Format(_T("%d"), ip3);
	m_DlgSearch.m_EditIP3.SetWindowTextW(str);
	str.Format(_T("%d"), ip4);
	m_DlgSearch.m_EditIP4.SetWindowTextW(str);
	m_DlgSearch.m_EditIP5.SetWindowTextW(_T("255"));

	m_DlgSearch.ShowWindow(SW_SHOW);

	EnableWindow(FALSE);
}


void CESPracerDlg::OnBnClickedButton4()
{
	// TODO: ここにコントロール通知ハンドラー コードを追加します。
	m_DlgConfig.ShowWindow(SW_SHOW);

	EnableWindow(FALSE);
}


static char* REQDATA = "abcdefghijklmnop";

void CESPracerDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: ここにメッセージ ハンドラー コードを追加するか、既定の処理を呼び出します。
	if (nIDEvent == TIMER_ID_JOYSTICK) {
		if (m_JoyStickActive) {
			if (!m_CheckJoystick.IsWindowEnabled()) {
				m_CheckJoystick.EnableWindow(TRUE);
			}

			if (m_CheckJoystick.GetCheck() == BST_CHECKED) {
				CString str;
				m_BtnUpdate.EnableWindow(FALSE);
				m_EditSpeedR.EnableWindow(FALSE);
				m_EditSpeedL.EnableWindow(FALSE);

				EnterCriticalSection(&csLock);
				if (m_CheckRevRightLeft.GetCheck() == BST_CHECKED) {
					m_iSpeedL = (int)(63 * (double)((double)m_RY / 1000.0));
					m_iSpeedR = (int)(63 * (double)((double)m_LY / 1000.0));
				}
				else {
					m_iSpeedR = (int)(63 * (double)((double)m_RY / 1000.0));
					m_iSpeedL = (int)(63 * (double)((double)m_LY / 1000.0));
				}

				if (m_CheckRevUpDown.GetCheck() == BST_CHECKED) {
					m_iSpeedR = -1 * m_iSpeedR;
					m_iSpeedL = -1 * m_iSpeedL;
				}

				if (m_iSpeedR < -63) {
					m_iSpeedR = -63;
				}
				else if (m_iSpeedR > 63) {
					m_iSpeedR = 63;
				}
				if (m_iSpeedL < -63) {
					m_iSpeedL = -63;
				}
				else if (m_iSpeedL > 63) {
					m_iSpeedL = 63;
				}
				str.Format(_T("%d"), m_iSpeedR);
				m_EditSpeedR.SetWindowTextW(str);
				str.Format(_T("%d"), m_iSpeedL);
				m_EditSpeedL.SetWindowTextW(str);

				LeaveCriticalSection(&csLock);
			}
			else {
				if (!m_BtnUpdate.IsWindowEnabled()) {
					m_BtnUpdate.EnableWindow(TRUE);
				}
				if (!m_EditSpeedR.IsWindowEnabled()) {
					m_EditSpeedR.EnableWindow(TRUE);
				}
				if (!m_EditSpeedL.IsWindowEnabled()) {
					m_EditSpeedL.EnableWindow(TRUE);
				}
			}
		}
		else {
			m_CheckJoystick.EnableWindow(FALSE);
			m_CheckJoystick.SetCheck(BST_UNCHECKED);
			if (!m_BtnUpdate.IsWindowEnabled()) {
				m_BtnUpdate.EnableWindow(TRUE);
			}
			if (!m_EditSpeedR.IsWindowEnabled()) {
				m_EditSpeedR.EnableWindow(TRUE);
			}
			if (!m_EditSpeedL.IsWindowEnabled()) {
				m_EditSpeedL.EnableWindow(TRUE);
			}
		}

		CDialogEx::OnTimer(nIDEvent);
		return;
	}

	static bool fLockTimer = false;
	if (fLockTimer) {
		CDialogEx::OnTimer(nIDEvent);
		return;
	}
	fLockTimer = true;

	if (m_DlgSearch.m_fSearching) {
		IPAddr ipaddr;
		HANDLE hIcmp = NULL;
		DWORD ret;
		char* pReply = NULL;
		DWORD cbReply = 0;
		WORD cbRequest = (WORD)strlen(REQDATA);

		m_DlgSearch.m_BtnSearch.EnableWindow(TRUE);
		m_DlgSearch.m_BtnSearch.SetWindowTextW(_T("検索中止"));
		m_DlgSearch.m_BtnSearch.RedrawWindow();
		m_DlgSearch.m_BtnClose.EnableWindow(FALSE);
		m_DlgSearch.m_BtnClose.RedrawWindow();

		// Destination Address
		char sbuf[32];
		sprintf_s(sbuf, 32, "%d.%d.%d.%d", m_DlgSearch.m_iIP1, m_DlgSearch.m_iIP2, m_DlgSearch.m_iIP3, m_DlgSearch.m_iIPVal);
		ipaddr = inet_addr((const char*)sbuf);
		CString strIP;
		strIP.Format(_T("検索中:%d.%d.%d.%d"), m_DlgSearch.m_iIP1, m_DlgSearch.m_iIP2, m_DlgSearch.m_iIP3, m_DlgSearch.m_iIPVal);
		m_DlgSearch.m_EditSearchIP.SetWindowTextW(strIP);
		m_DlgSearch.m_EditSearchIP.RedrawWindow();

		// Reply Buffer
		cbReply = sizeof(ICMP_ECHO_REPLY) + cbRequest;
		pReply = (char*)malloc(cbReply);
		ASSERT(pReply);

		// Sending Icmp Request
		hIcmp = IcmpCreateFile();
		ASSERT(INVALID_HANDLE_VALUE != hIcmp);

		TRACE(sbuf);
		ret = IcmpSendEcho(
			hIcmp,
			ipaddr,
			REQDATA,
			cbRequest,
			NULL,
			pReply,
			cbReply,
			5000);

		if (ret) {
			PICMP_ECHO_REPLY p = (PICMP_ECHO_REPLY)pReply;
			TRACE("Status: %d\n", p->Status);
			TRACE("Round Trip Time: %d\n", p->RoundTripTime);
			TRACE("Data Size: %d\n", p->DataSize);

			if (p->Status == 0) {
				unsigned short port = SERVER_PORT;
				SOCKET sockClient = INVALID_SOCKET;
				struct sockaddr_in sockAddrInServer;

				BeginWaitCursor();

				strIP.Format(_T("発見:%d.%d.%d.%d ... 確認中"), m_DlgSearch.m_iIP1, m_DlgSearch.m_iIP2, m_DlgSearch.m_iIP3, m_DlgSearch.m_iIPVal);
				m_DlgSearch.m_EditSearchIP.SetWindowTextW(strIP);
				m_DlgSearch.m_EditSearchIP.RedrawWindow();

				memset(&sockAddrInServer, 0, sizeof(sockAddrInServer));
				sockAddrInServer.sin_port = htons(port);
				sockAddrInServer.sin_family = AF_INET;
				sockAddrInServer.sin_addr.s_addr = ipaddr;

				sockClient = socket(AF_INET, SOCK_STREAM, 0);
				if (sockClient != INVALID_SOCKET) {
					if (SOCKET_ERROR == connect(sockClient, (struct sockaddr*)&sockAddrInServer, sizeof(sockAddrInServer))) {
						TRACE("ESPClientThread connect Error !!\n");
						closesocket(sockClient);
						sockClient = INVALID_SOCKET;
					}
					else {
						char buff[32];
						sprintf_s(buff, 32, "###,xxx\r");
						if (send(sockClient, buff, ((int)strlen(buff) + 0), 0) < 0) {
							// 送信エラー
							TRACE("ESPServerThread send Error !!\n");
						}
						else {
							memset(buff, 0, sizeof(buff));
							int len = recv(sockClient, buff, 7, 0);
							TRACE("ESPClientThread recv [%s]", buff);
							if (buff[0] == '$' && buff[1] == '$' && buff[2] == '$') {
								// 車があった場合！
								TRACE("====> ESP Server Find !!! [%s]", sbuf);
								strIP.Format(_T("%d.%d.%d.%d"), m_DlgSearch.m_iIP1, m_DlgSearch.m_iIP2, m_DlgSearch.m_iIP3, m_DlgSearch.m_iIPVal);
								m_DlgSearch.m_ListIP.AddString(strIP);
								m_DlgSearch.m_ListIP.RedrawWindow();
							}
						}

						closesocket(sockClient);
						sockClient = INVALID_SOCKET;
					}
				}

				EndWaitCursor();
			}
		}

		free(pReply);
		IcmpCloseHandle(hIcmp);


		// Next
		if (m_DlgSearch.m_iIP5 > m_DlgSearch.m_iIPVal) {
			m_DlgSearch.m_iIPVal++;
		}
		else {
			m_DlgSearch.m_fSearching = false;
		}
	}
	else {
		m_DlgSearch.m_BtnSearch.EnableWindow(TRUE);
		m_DlgSearch.m_EditSearchIP.SetWindowTextW(_T(""));
		m_DlgSearch.m_BtnClose.EnableWindow(TRUE);
		m_DlgSearch.m_BtnSearch.SetWindowTextW(_T("検索"));
	}

	fLockTimer = false;
	CDialogEx::OnTimer(nIDEvent);
}
