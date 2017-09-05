//#include "vld.h"
#include "QTApp.h"

#ifdef _WIN32
#include <windows.h>
#endif // _WIN32



int main(int argc, char** argv)
{
	//禁止双开
#ifdef _WIN32
	HANDLE hObject = ::CreateMutex(NULL, FALSE, "Mutex_measureTool");
	if (GetLastError() == ERROR_ALREADY_EXISTS)
	{
		CloseHandle(hObject);
		MessageBox(NULL, "应用程序已经在运行！", "提示", MB_ICONERROR | MB_OK);
		return -1;
	}
#endif

	QTApp::GetInstance().Init(argc, argv);

	return QTApp::GetInstance().Run();
}