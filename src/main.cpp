#include "triple/model.hpp"
#include "triple/integrator.hpp"
#include "triple/collision.hpp"
#include "triple/scene.hpp"
#include "triple/sensors.hpp"
#include "triple/pid.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <iostream>
#include <algorithm>

// Main function to run the simulation
int main() {
    SensorSuite sensors;

    PID positionPid(
        1.0,   // kp
        0.0,   // ki
        2.0,   // kd
        10.0); // 积分限幅

    PID anglePid(
        20.0,  // kp
        0.0,   // ki
        4.0,   // kd
        10.0);

    bool controllerEnabled = false;
    const double maxControlForce = 30.0;
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

    Vec4 y(0.0, 0.15, 0.0, 0.0); // Initial state: [cart position, rod angle, cart velocity, rod angular velocity]
    
    bool running = true;
    float manualForce = 0.0f;

    double dt = 0.001; // Time step
    double accumulator = 0.0;
    double lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const double currentTime = glfwGetTime();
        const double frameTime = std::min(currentTime - lastTime, 0.1);
        
        lastTime = currentTime;
        if (running) {
            accumulator += frameTime;
            while (accumulator >= dt) {
                double controlForce =
                    static_cast<double>(manualForce);

                if (controllerEnabled) {
                    const Measurement measured =
                        sensors.read(y);

                    const double positionError =
                        -measured.x;

                    const double positionErrorRate =
                        -measured.cartVelocity;

                    double angleError =
                        -measured.theta;

                    angleError = std::atan2(
                        std::sin(angleError),
                        std::cos(angleError));

                    const double angleErrorRate =
                        -measured.omega;

                    const double positionOutput =
                        positionPid.update(
                            positionError,
                            positionErrorRate,
                            dt);

                    const double angleOutput =
                        anglePid.update(
                            angleError,
                            angleErrorRate,
                            dt);

                    controlForce =
                        - angleOutput;
                        // positionOutput - angleOutput;
                }

                // 无论是否启用控制器，都限制最终执行器推力
                controlForce = std::clamp(
                    controlForce,
                    -maxControlForce,
                    maxControlForce);

                // 必须使用最终合成的 controlForce
                rk4Step(y, dt, controlForce, p);

                solveWallCollision(y, p);

                accumulator -= dt;
            }
        } else {
            accumulator = 0.0;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        // 先绘制物理场景到背景层
        drawScene(y, p, io.DisplaySize);

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

        ImGui::Checkbox("Running", &running);

        if (ImGui::Button("Reset")) {
            y << 0.0, 0.15, 0.0, 0.0;
            accumulator = 0.0;
        }

        ImGui::SameLine();

        if (ImGui::Button("Zero Force")) { 
            manualForce = 0.0f; 
        }
        ImGui::SliderFloat("Cart Force (N)", &manualForce, -30.0f, 30.0f);

        ImGui::Separator();
        ImGui::Text("State");

        ImGui::Text("x:       %.5f m", y(0));
        ImGui::Text("theta:   %.5f rad", y(1));
        ImGui::Text("x_dot:   %.5f m/s", y(2));
        ImGui::Text("omega:   %.5f rad/s", y(3));

        ImGui::Separator();

        ImGui::Text(
            "Total energy: %.7f J",
            totalEnergy(y, p));

        ImGui::Text("Render: %.1f FPS", io.Framerate);
        
        ImGui::Checkbox(
            "Controller Enabled",
            &controllerEnabled);
        if (ImGui::Button("Reset")) {
            y << 0.0, 0.15, 0.0, 0.0;
            accumulator = 0.0;

            positionPid.reset();
            anglePid.reset();
        }

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