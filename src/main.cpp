#include "triple/integrator.hpp"
#include "triple/scene.hpp"
#include "triple/sensors.hpp"
#include "triple/simulation.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <iostream>
#include <algorithm>

// Main function to run the simulation
int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(
        1280, 720, "Inverted Pendulum Simulation", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGuiIO& io = ImGui::GetIO(); 
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 120");

    Parameters p;

    SensorNoise noise;
    Simulation simulation(p, noise);

    double lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const double currentTime = glfwGetTime();
        const double frameTime = std::min(currentTime - lastTime, 0.1);
        
        lastTime = currentTime;
        simulation.update(frameTime);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        // 先绘制物理场景到背景层
        drawScene(
            simulation.state(),
            simulation.parameters(),
            io.DisplaySize);

        // 再绘制左侧控制面板
        ImGui::SetNextWindowPos(
            ImVec2(10.0f, 10.0f),
            ImGuiCond_Always);

        ImGui::SetNextWindowSize(
            ImVec2(300.0f, io.DisplaySize.y - 20.0f),
            ImGuiCond_Always);

        ImGui::Begin(
            "Controls",
            nullptr,
            ImGuiWindowFlags_NoResize
            | ImGuiWindowFlags_NoMove
            | ImGuiWindowFlags_NoSavedSettings);

        ImGui::Text("Simulation");
        ImGui::Separator();

        bool running = simulation.isRunning();

        if (ImGui::Checkbox("Running", &running)) {
            simulation.setRunning(running);
        }

        bool controllerEnabled =
            simulation.isControllerEnabled();

        if (ImGui::Checkbox(
                "Controller Enabled",
                &controllerEnabled)) {
            simulation.setControllerEnabled(
                controllerEnabled);
        }

        float manualForce = static_cast<float>(
            simulation.manualForce());

        if (ImGui::SliderFloat(
                "Cart Force (N)",
                &manualForce,
                -30.0f,
                30.0f)) {
            simulation.setManualForce(manualForce);
        }

        if (ImGui::Button("Reset")) {
            simulation.reset();
        }

        const Vec4& state = simulation.state();

        ImGui::Separator();

        ImGui::Text(
            "Cart Position: %.5f m",
            state(0));

        ImGui::Text(
            "Rod Angle: %.5f rad",
            state(1));

        ImGui::Text(
            "Cart Velocity: %.5f m/s",
            state(2));

        ImGui::Text(
            "Angular Velocity: %.5f rad/s",
            state(3));

        ImGui::Text(
            "Applied Force: %.5f N",
            simulation.appliedForce());

        ImGui::Text(
            "Energy: %.6f J",
            totalEnergy(
                simulation.state(),
                simulation.parameters()));

        ImGui::End();

        // 渲染
        ImGui::Render();

        int displayWidth = 0;
        int displayHeight = 0;

        glfwGetFramebufferSize(
            window,
            &displayWidth,
            &displayHeight);

        glViewport(0, 0, displayWidth, displayHeight);

        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(
            ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}