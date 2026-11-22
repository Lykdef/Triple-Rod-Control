#pragma once

#include "imgui.h"
#include <GLFW/glfw3.h>

class Window {
public:
    Window(
        int width,
        int height,
        const char* title);
    
    ~Window();

    Window(const Window&) = delete;
    Window& operator = (const Window&) = delete;
    
    Window(Window&&) = delete;
    Window& operator = (Window&&) = delete;

    bool shouldClose() const;
    void pollEvents() const;

    void beginFrame();
    void endFrame();

    ImVec2 displaySize() const;
    double time() const;

private:
    void shutdown();

private:
    GLFWwindow* handle_ = nullptr;
    bool glfwInitialized_ = false;
    bool imguiContextCreated_ = false;
    bool glfwBackendInitialized_ = false;
    bool openGLBackendInitialized_ = false;
};