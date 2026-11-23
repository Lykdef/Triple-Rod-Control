#include "triple/scene.hpp"
#include "triple/sensors.hpp"
#include "triple/simulation.hpp"
#include "triple/window.hpp"

#include "imgui.h"

#include <algorithm>
#include <iostream>

int main() {
    try {
        Window window(1280, 720, "Triple");
        Parameters parameters;
        SensorNoise noise;
        Simulation simulation(parameters, noise);
        double lastTime = window.time();

        while (!window.shouldClose()) {
            window.pollEvents();
            const double currentTime = window.time();
            const double frameTime = std::min(currentTime - lastTime, 0.1);
            lastTime = currentTime;

            simulation.update(frameTime);
            window.beginFrame();
            drawScene(
                simulation.state(), simulation.parameters(), window.displaySize());

            ImGui::Begin("Controls");

            bool running = simulation.isRunning();
            if (ImGui::Checkbox("Running", &running)) {
                simulation.setRunning(running);
            }

            bool controllerEnabled = simulation.isControllerEnabled();
            if (ImGui::Checkbox("Controller Enabled", &controllerEnabled)) {
                simulation.setControllerEnabled(controllerEnabled);
            }

            int links = static_cast<int>(simulation.parameters().linkCount()) - 1;
            if (ImGui::Combo("Rod Count", &links, "1\0 2\0 3\0\0")) {
                simulation.setLinkCount(static_cast<std::size_t>(links + 1));
            }

            float manualForce = static_cast<float>(simulation.manualForce());
            if (ImGui::SliderFloat("Cart Force (N)", &manualForce, -30.0f, 30.0f)) {
                simulation.setManualForce(manualForce);
            }
            if (ImGui::Button("Reset Manual Force")) {
                simulation.setManualForce(0.0);
            }

            if (ImGui::Button("Reset Upright")) {
                simulation.reset();
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset Hanging")) {
                simulation.resetHanging();
            }

            const State& state = simulation.state();
            const std::size_t n = simulation.parameters().linkCount();
            ImGui::Separator();
            ImGui::Text(
                "Control Mode: %s",
                simulation.isSwingingUp() ? "Swing Up" : "Balance");
            ImGui::Text("Cart Position: %.5f m", state(0));
            ImGui::Text("Cart Velocity: %.5f m/s", state(n + 1));
            for (std::size_t rod = 0; rod < n; ++rod) {
                ImGui::Text("Rod %zu Angle: %.5f rad", rod + 1, state(1 + rod));
                ImGui::Text(
                    "Rod %zu Angular Velocity: %.5f rad/s",
                    rod + 1, state(n + 2 + rod));
            }
            ImGui::Text("Applied Force: %.5f N", simulation.appliedForce());
            ImGui::Text(
                "Energy: %.6f J",
                totalEnergy(state, simulation.parameters()));
            ImGui::End();

            window.endFrame();
        }
    } catch (const std::exception& error) {
        std::cerr << "Application error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
