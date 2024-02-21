//
// Joy Stick用ヘッダ
//
#pragma once

#include <fcntl.h>

#define DIRECTINPUT_VERSION     0x0800          // DirectInputのバージョン指定
#include <dinput.h>

#define MAX_JOYSTICK_RAGE	1000

struct _JOYSTICK_DATA {
	union {
		struct {
			long m_LX;
			long m_LY;
			long m_RX;
			long m_RY;
		} ds;
		struct {
			long m_AX;
			long m_AY;
			long m_AZ;
			long m_AW;
		} ax;
	};
	
	BOOL m_bStaticL1;
	BOOL m_bStaticL2;
	BOOL m_bStaticR1;
	BOOL m_bStaticR2;
	BOOL m_bStaticShare;
	BOOL m_bStaticOption;
	BOOL m_bStaticUP;
	BOOL m_bStaticLeft;
	BOOL m_bStaticRight;
	BOOL m_bStaticDown;
	BOOL m_bStaticTri;
	BOOL m_bStaticSqa;
	BOOL m_bStaticCir;
	BOOL m_bStaticCross;
	BOOL m_bStaticPS;
	BOOL m_bStaticPad;

	DWORD m_Pad;

};

extern CRITICAL_SECTION		gCsJoyStick;		// 排他処理ジョイスティック
extern LPDIRECTINPUT8		lpDI;				// IDirectInput8
extern LPDIRECTINPUTDEVICE8	lpJoystick;			// ジョイスティックデバイス

extern "C" {
	void InitJoyStick(HINSTANCE instance, HWND hwnd, LPVOID process);
	void EndJoyStick();
	BOOL PASCAL EnumJoyDeviceProc(LPCDIDEVICEINSTANCE lpddi, LPVOID pvRef);
	BOOL GetJoyStickDevice(int mode);
	BOOL GetJoyStick(int mode);
	UINT JoyStickThread(LPVOID p);
}

