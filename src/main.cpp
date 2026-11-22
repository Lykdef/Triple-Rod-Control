#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>
#include <Eigen/Dense>

using Vec4 = Eigen::Vector4d;

struct Parameters {
    double cartMass = 1.0;
    double rodMass = 0.2;
    double rodLength = 0.6;
    double gravity = 9.82;

    double trackMin = -1.5;
    double trackMax = 1.5;
    double cartHalfWidth = 0.28;
    double restitution = 1.0;
};

// Function to compute the derivative of the state vector
Vec4 derivative(const Vec4& y, double force, const Parameters& p) {
    const double theta = y(1);
    const double omega = y(3);

    const double M = p.cartMass;
    const double m = p.rodMass;
    const double L = p.rodLength;
    const double g = p.gravity;

    const double a = L * 0.5;
    const double I = m * L * L / 12.0;
    const double J = I + m * a * a;
    
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    const double A = M + m;
    const double B = m * a * c;
    const double R = force + m * a * omega * omega * s;
    const double G = m * g * a * s;

    const double determinant = A * J - B * B;
    
    const double cartAcceleration = (J * R - B * G) / determinant;
    const double angularAcceleration = (-B * R + A * G) / determinant;

    Vec4 dy;
    dy << y(2), y(3), cartAcceleration, angularAcceleration;
    return dy;
}
// Function to compute the total energy of the system
double totalEnergy(const Vec4& y, const Parameters& p) {
    const double theta = y(1);
    const double cartVelocity = y(2);
    const double omega = y(3);

    const double M = p.cartMass;
    const double m = p.rodMass;
    const double L = p.rodLength;
    const double g = p.gravity;

    const double a = L * 0.5;
    const double I = m * L * L / 12.0;

    const double comVelocityX = cartVelocity + a * omega * std::cos(theta);
    const double comVelocityY = - a * omega * std::sin(theta);
    const double cartKinetic = 0.5 * M * cartVelocity * cartVelocity;
    const double rodTranslationalKinetic = 
        0.5 * m * (comVelocityX * comVelocityX + comVelocityY * comVelocityY);
    const double rodRotationalKinetic = 0.5 * I * omega * omega;
    const double potential = m * g * a * std::cos(theta);

    return cartKinetic + rodTranslationalKinetic + rodRotationalKinetic + potential;
}
// Function to perform a single RK4 step
void rk4Step(Vec4& y, double dt, double force, const Parameters& p) {
    Vec4 k1 = derivative(y, force, p);
    Vec4 k2 = derivative(y + 0.5 * dt * k1, force, p);
    Vec4 k3 = derivative(y + 0.5 * dt * k2, force, p);
    Vec4 k4 = derivative(y + dt * k3, force, p);

    y += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
// Function to draw the scene using ImGui
void drawScene(const Vec4& y, const Parameters& p, const ImVec2& displaySize) {
    ImDrawList* drawList = ImGui::GetBackgroundDrawList();

    // 整个窗口的背景
    drawList->AddRectFilled(
        ImVec2(0.0f, 0.0f),
        displaySize,
        IM_COL32(22, 26, 34, 255));

    const float scale = 180.0f;

    // 世界坐标原点在窗口中下方
    const ImVec2 origin(
        displaySize.x * 0.55f,
        displaySize.y * 0.75f);

    auto worldToScreen = [&](double worldX, double worldY) {
        return ImVec2(
            origin.x + static_cast<float>(worldX * scale),
            origin.y - static_cast<float>(worldY * scale));
    };

    const double cartX = y(0);
    const double theta = y(1);

    const ImVec2 pivot = worldToScreen(cartX, 0.0);

    const ImVec2 tip = worldToScreen(
        cartX + p.rodLength * std::sin(theta),
        p.rodLength * std::cos(theta));
    
    // 轨道
    const ImVec2 trackLeft =
    worldToScreen(p.trackMin, 0.0);

    const ImVec2 trackRight =
        worldToScreen(p.trackMax, 0.0);

    const float groundY = pivot.y + 45.0f;

    drawList->AddLine(
        ImVec2(trackLeft.x, groundY),
        ImVec2(trackRight.x, groundY),
        IM_COL32(110, 120, 140, 255),
        4.0f);

    drawList->AddRectFilled(
        ImVec2(trackLeft.x - 12.0f, groundY - 65.0f),
        ImVec2(trackLeft.x + 4.0f, groundY),
        IM_COL32(170, 75, 70, 255));

    drawList->AddRectFilled(
        ImVec2(trackRight.x - 4.0f, groundY - 65.0f),
        ImVec2(trackRight.x + 12.0f, groundY),
        IM_COL32(170, 75, 70, 255));

    // 滑块
    drawList->AddRectFilled(
        ImVec2(pivot.x - 50.0f, pivot.y),
        ImVec2(pivot.x + 50.0f, pivot.y + 32.0f),
        IM_COL32(70, 140, 230, 255),
        5.0f);

    // 杆
    drawList->AddLine(
        pivot,
        tip,
        IM_COL32(250, 190, 60, 255),
        10.0f);

    // 连接点
    drawList->AddCircleFilled(
        pivot,
        9.0f,
        IM_COL32(255, 255, 255, 255));

    // 杆末端
    drawList->AddCircleFilled(
        tip,
        12.0f,
        IM_COL32(240, 90, 70, 255));

    // 用于确认绘制函数确实执行
    drawList->AddText(
        ImVec2(origin.x - 70.0f, origin.y + 75.0f),
        IM_COL32(180, 190, 205, 255),
        "Scene active");
}
// Function to handle wall collisions and update the state vector accordingly
void solveWallCollision(Vec4& y, const Parameters& p) {
    const double leftLimit =
        p.trackMin + p.cartHalfWidth;

    const double rightLimit =
        p.trackMax - p.cartHalfWidth;

    bool collided = false;

    // 左墙
    if (y(0) < leftLimit) {
        y(0) = leftLimit;

        // 只有仍在向墙内运动时才处理碰撞
        if (y(2) < 0.0) {
            collided = true;
        }
    }
    // 右墙
    else if (y(0) > rightLimit) {
        y(0) = rightLimit;

        if (y(2) > 0.0) {
            collided = true;
        }
    }

    if (!collided) {
        return;
    }

    const double theta = y(1);
    const double oldCartVelocity = y(2);
    const double oldOmega = y(3);

    const double M = p.cartMass;
    const double m = p.rodMass;
    const double L = p.rodLength;

    const double a = L * 0.5;
    const double I = m * L * L / 12.0;

    const double A = M + m;
    const double B = m * a * std::cos(theta);
    const double J = I + m * a * a;
    const double determinant = A * J - B * B;

    const double e = p.restitution;

    // 墙面施加在滑块上的法向冲量
    const double impulse =
        -(1.0 + e)
        * determinant / J
        * oldCartVelocity;

    // 由 M * delta_v = [impulse, 0] 更新速度
    y(2) =
        oldCartVelocity
        + J / determinant * impulse;

    y(3) =
        oldOmega
        - B / determinant * impulse;
}
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
                rk4Step(y, dt, static_cast<double>(manualForce), p);
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