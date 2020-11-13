#include <stdio.h>
#include <Windows.h>
#include <iostream>

using namespace std;

int main()
{
	POINT cursorPosition;
	int mX = 0, mY = 0;
	bool dontRepeat = false;

	while (1)
	{
		if (GetKeyState(VK_CAPITAL))
		{
			SetCursorPos(mX, mY);
			dontRepeat = false;
			while (GetKeyState(VK_CAPITAL))
			{
				GetCursorPos(&cursorPosition);
				if (cursorPosition.x != mX || cursorPosition.y != mY)
				{
					mX = cursorPosition.x;
					mY = cursorPosition.y;
					cout << mX << " , " << mY << endl;
				}
			}
		}
		if (!dontRepeat)
		{
			cout << "Your capsLock is turned off, please turn it on to continue program" << endl;
			dontRepeat = true;
		}
	}
	return 0;
}
