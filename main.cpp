#include <windows.h>

#include <cstdint>
#include <chrono>
#include <iostream>

#include "engine.h"

static bool Running;

static BITMAPINFO BitmapInfo;
static void *BitmapMemory;
static int BitmapWidth;
static int BitmapHeight;
static bool BitmapChanged;
static bool HasFocus;
static std::vector<int> KeyPresses;
const int BytesPerPixel = 4;

static void
Win32Resize(int Width, int Height)
{
    if (BitmapMemory)
    {
        VirtualFree(BitmapMemory, 0, MEM_RELEASE);
    }

    BitmapWidth = Width;
    BitmapHeight = Height;

    BitmapInfo.bmiHeader.biSize = sizeof(BitmapInfo.bmiHeader);
    BitmapInfo.bmiHeader.biWidth = BitmapWidth;
    BitmapInfo.bmiHeader.biHeight = -BitmapHeight;
    BitmapInfo.bmiHeader.biPlanes = 1;
    BitmapInfo.bmiHeader.biBitCount = 32;
    BitmapInfo.bmiHeader.biCompression = BI_RGB;

    int BitmapMemorySize = (Width * Height) * BytesPerPixel;
    BitmapMemory = VirtualAlloc(0, BitmapMemorySize, MEM_COMMIT, PAGE_READWRITE);
    BitmapChanged = true;
}

std::pair<int, int>
Win32GetClientSize(HWND Window)
{
    RECT ClientRect;
    GetClientRect(Window, &ClientRect);
    int WindowWidth = ClientRect.right - ClientRect.left;
    int WindowHeight = ClientRect.bottom - ClientRect.top;
    return {WindowWidth, WindowHeight};
}

void Win32CenterCursor(HWND Window)
{
    POINT cursor;
    auto [Width, Height] = Win32GetClientSize(Window);
    cursor.x = Width / 2; cursor.y = Height / 2;
    ClientToScreen(Window, &cursor);
    SetCursorPos(cursor.x, cursor.y);
}

LRESULT CALLBACK
Win32MainWindowCallback(
    HWND Window,
    UINT Message,
    WPARAM WParam,
    LPARAM LParam)
{
    LRESULT Result = 0;

    switch (Message)
    {
        case WM_SIZE:
        {
            RECT ClientRect;
            GetClientRect(Window, &ClientRect);
            int Width = ClientRect.right - ClientRect.left;
            int Height = ClientRect.bottom - ClientRect.top;
            Win32Resize(Width, Height);
            break;
        }

        case WM_KEYDOWN: KeyPresses.push_back(WParam); break;
        case WM_DESTROY: Running = false; break;
        case WM_CLOSE: Running = false; break;
        case WM_SETFOCUS: ShowCursor(false); HasFocus = true; break;
        case WM_KILLFOCUS: ShowCursor(true); HasFocus = false; break;
        default: Result = DefWindowProc(Window, Message, WParam, LParam); break;
    }

    return Result;
}

int CALLBACK
WinMain(
    HINSTANCE Instance,
    HINSTANCE PrevInstance,
    LPSTR CommandLine,
    int ShowCode)
{
    WNDCLASS WindowClass = {};
    WindowClass.style = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;
    WindowClass.lpfnWndProc = Win32MainWindowCallback;
    WindowClass.lpszClassName = "BobWindowClass";

    if (RegisterClass(&WindowClass))
    {
        HWND Window = CreateWindowEx(
            0,
            WindowClass.lpszClassName,
            "Bob",
            WS_OVERLAPPEDWINDOW | WS_VISIBLE,
            CW_USEDEFAULT, CW_USEDEFAULT, 800, 600,
            0, 0, Instance, 0);

        if (Window)
        {
            // Borderless Flags
            // LONG lStyle = GetWindowLong(Window, GWL_STYLE);
            // lStyle &= ~(WS_CAPTION | WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU);
            // SetWindowLongPtr(Window, GWL_STYLE, lStyle);

            // LONG lExStyle = GetWindowLongPtr(Window, GWL_EXSTYLE);
            // lExStyle &= ~(WS_EX_DLGMODALFRAME | WS_EX_CLIENTEDGE | WS_EX_STATICEDGE);
            // SetWindowLongPtr(Window, GWL_EXSTYLE, lExStyle);
            // SetWindowPos(Window, NULL, 0, 0, 0, 0, SWP_FRAMECHANGED | SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOOWNERZORDER);

            Engine engine(BitmapWidth, BitmapHeight, 90);
            engine.loadObj("obj/bob.obj");

            Running = true;

            auto t1 = std::chrono::system_clock::now();
            auto t2 = std::chrono::system_clock::now();

            while (Running)
            {
                t2 = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed = t2 - t1;
                t1 = t2;

                if (HasFocus)
                {
                    for (const auto& KeyPress : KeyPresses)
                    {
                        if (KeyPress == 'Q')
                        {
                            engine.wireframe = !engine.wireframe;
                        }
                    }

                    KeyPresses.clear();

                    if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
                    {
                        Running = false;
                    }

                    POINT cursor;
                    GetCursorPos(&cursor);
                    ScreenToClient(Window, &cursor);
                    int cursorx = cursor.x;
                    int cursory = cursor.y;
                    Win32CenterCursor(Window);

                    if (GetAsyncKeyState('W') & 0x8000)
                    {
                        engine.camera.pos -= engine.camera.forwardDir * (40.f * elapsed.count());
                    }

                    if (GetAsyncKeyState('A') & 0x8000)
                    {
                        engine.camera.pos -= engine.camera.rightDir * (40.f * elapsed.count());
                    }

                    if (GetAsyncKeyState('S') & 0x8000)
                    {
                        engine.camera.pos += engine.camera.forwardDir * (40.f * elapsed.count());
                    }

                    if (GetAsyncKeyState('D') & 0x8000)
                    {
                        engine.camera.pos += engine.camera.rightDir * (40.f * elapsed.count());
                    }

                    engine.camera.updateTranslationRotation(
                        cursorx - BitmapWidth / 2,
                        cursory - BitmapHeight / 2);
                }

                MSG Message;
                while (PeekMessage(&Message, 0, 0, 0, PM_REMOVE))
                {
                    if (Message.message == WM_QUIT)
                    {
                        Running = false;
                    }

                    TranslateMessage(&Message);
                    DispatchMessageA(&Message);
                }

                if (HasFocus)
                {
                    std::cout << "camera pos: " << engine.camera.pos << std::endl;
                    std::cout << "yaw: " << (engine.camera.yaw * 180 / 3.141592) << std::endl;
                    std::cout << "pitch: " << (engine.camera.pitch * 180 / 3.141592) << std::endl;
                    std::cout << "fps: " << 1.0f / elapsed.count() << std::endl;

                    HDC DeviceContext = GetDC(Window);
                    auto [WindowWidth, WindowHeight] = Win32GetClientSize(Window);

                    if (BitmapChanged)
                    {
                        BitmapChanged = false;
                        engine.width = BitmapWidth;
                        engine.height = BitmapHeight;
                        engine.mem = static_cast<uint32_t *>(BitmapMemory);
                        engine.depthBuffer = std::vector<float>(BitmapWidth * BitmapHeight, engine.camera.far_);
                        float aspectRatio = static_cast<float>(BitmapWidth) / static_cast<float>(BitmapHeight);
                        engine.camera.update(90, 1, 1000, aspectRatio);
                        engine.camera.updateTranslationRotation(BitmapWidth / 2, BitmapHeight / 2);
                    }

                    engine.render();

                    StretchDIBits(DeviceContext,
                        0, 0, BitmapWidth, BitmapHeight,
                        0, 0, WindowWidth, WindowHeight,
                        BitmapMemory, &BitmapInfo,
                        DIB_RGB_COLORS, SRCCOPY);

                    ReleaseDC(Window, DeviceContext);
                }
            }
        }
    }

    return 0;
}
