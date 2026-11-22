#include "triple/integrator.hpp"
#include "triple/scene.hpp"
#include "triple/sensors.hpp"
#include "triple/simulation.hpp"
#include "triple/window.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <iostream>
#include <algorithm>

// Main function to run the simulation
int main() {
    try {
        Window window(1280, 720, "Triple");
        Parameters p;

        SensorNoise noise;
        Simulation simulation(p, noise);

        double lastTime = window.time();

    while (!window.shouldClose()) {
        window.pollEvents();
        const double currentTime = window.time();
        const double frameTime = std::min(currentTime - lastTime, 0.1);
        
        lastTime = currentTime;
        simulation.update(frameTime);
        window.beginFrame();
        
        drawScene(
            simulation.state(),
            simulation.parameters(),
            window.displaySize());

        ImGui::Begin("Controls");

        bool running =
            simulation.isRunning();

        if (ImGui::Checkbox(
                "Running",
                &running)) {
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

        float manualForce =
            static_cast<float>(
                simulation.manualForce());

        if (ImGui::SliderFloat(
                "Cart Force (N)",
                &manualForce,
                -30.0f,
                30.0f)) {
            simulation.setManualForce(
                manualForce);
        }

        if (ImGui::Button("Reset")) {
            simulation.reset();
        }

        const Vec4& state =
            simulation.state();

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

        // 渲染并交换缓冲区。
        window.endFrame();
    }
    } catch (const std::exception& error) {
        std::cerr
            << "Application error: "
            << error.what()
            << '\n';

        return 1;
    }
    return 0;
}