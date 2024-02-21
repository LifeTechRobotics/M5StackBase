
// ESPracer.h : PROJECT_NAME アプリケーションのメイン ヘッダー ファイルです
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH に対してこのファイルをインクルードする前に 'pch.h' をインクルードしてください"
#endif

#include "resource.h"		// メイン シンボル


// CESPracerApp:
// このクラスの実装については、ESPracer.cpp を参照してください
//

class CESPracerApp : public CWinApp
{
public:
	CESPracerApp();

// オーバーライド
public:
	virtual BOOL InitInstance();

// 実装

	DECLARE_MESSAGE_MAP()
};

extern CESPracerApp theApp;
