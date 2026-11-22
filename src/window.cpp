#include "triple/window.hpp"

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <stdexcept>

Window::Window(
    int width,
    int height,
    const char* title) {
    try {
        // 1. 初始化 GLFW。
        if (!glfwInit()) {
            throw std::runtime_error(
                "Failed to initialize GLFW.");
        }

        glfwInitialized_ = true;

        // 2. 创建 OpenGL 3.3 Core 上下文。
        glfwWindowHint(
            GLFW_CONTEXT_VERSION_MAJOR,
            3);

        glfwWindowHint(
            GLFW_CONTEXT_VERSION_MINOR,
            3);

        glfwWindowHint(
            GLFW_OPENGL_PROFILE,
            GLFW_OPENGL_CORE_PROFILE);

        handle_ = glfwCreateWindow(
            width,
            height,
            title,
            nullptr,
            nullptr);

        if (handle_ == nullptr) {
            throw std::runtime_error(
                "Failed to create GLFW window.");
        }

        glfwMakeContextCurrent(handle_);

        // 开启垂直同步。
        glfwSwapInterval(1);

        // 3. 创建 Dear ImGui 上下文。
        IMGUI_CHECKVERSION();

        if (ImGui::CreateContext() == nullptr) {
            throw std::runtime_error(
                "Failed to create ImGui context.");
        }

        imguiContextCreated_ = true;

        ImGui::StyleColorsDark();

        // 4. 初始化 ImGui 的 GLFW 后端。
        if (!ImGui_ImplGlfw_InitForOpenGL(
                handle_,
                true)) {
            throw std::runtime_error(
                "Failed to initialize ImGui GLFW backend.");
        }

        glfwBackendInitialized_ = true;

        // 5. 初始化 ImGui 的 OpenGL3 后端。
        // 注意必须包含 '#'，不能只写 "version 130"。
        if (!ImGui_ImplOpenGL3_Init("#version 130")) {
            throw std::runtime_error(
                "Failed to initialize ImGui OpenGL3 backend.");
        }

        openGLBackendInitialized_ = true;
    } catch (...) {
        // 构造函数失败时，析构函数不会自动执行，
        // 因此需要在这里清理已经初始化的部分。
        shutdown();
        throw;
    }
}

Window::~Window() {
    shutdown();
}

bool Window::shouldClose() const {
    return handle_ == nullptr
        || glfwWindowShouldClose(handle_);
}

void Window::pollEvents() const {
    glfwPollEvents();
}

void Window::beginFrame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void Window::endFrame() {
    ImGui::Render();

    int framebufferWidth = 0;
    int framebufferHeight = 0;

    glfwGetFramebufferSize(
        handle_,
        &framebufferWidth,
        &framebufferHeight);

    // 窗口最小化时， framebuffer 可能暂时为 0。
    if (framebufferWidth <= 0
        || framebufferHeight <= 0) {
        return;
    }

    glViewport(
        0,
        0,
        framebufferWidth,
        framebufferHeight);

    glClearColor(
        0.04f,
        0.04f,
        0.05f,
        1.00f);

    glClear(GL_COLOR_BUFFER_BIT);

    ImGui_ImplOpenGL3_RenderDrawData(
        ImGui::GetDrawData());

    glfwSwapBuffers(handle_);
}

ImVec2 Window::displaySize() const {
    return ImGui::GetIO().DisplaySize;
}

double Window::time() const {
    return glfwGetTime();
}

void Window::shutdown() {
    // 清理顺序与初始化顺序相反。
    if (openGLBackendInitialized_) {
        ImGui_ImplOpenGL3_Shutdown();
        openGLBackendInitialized_ = false;
    }

    if (glfwBackendInitialized_) {
        ImGui_ImplGlfw_Shutdown();
        glfwBackendInitialized_ = false;
    }

    if (imguiContextCreated_) {
        ImGui::DestroyContext();
        imguiContextCreated_ = false;
    }

    if (handle_ != nullptr) {
        glfwDestroyWindow(handle_);
        handle_ = nullptr;
    }

    if (glfwInitialized_) {
        glfwTerminate();
        glfwInitialized_ = false;
    }
}