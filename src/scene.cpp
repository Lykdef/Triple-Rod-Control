#include "triple/scene.hpp"

#include <cmath>

void drawScene(const State& y, const Parameters& p, const ImVec2& displaySize) {
    ImDrawList* drawList = ImGui::GetBackgroundDrawList();
    drawList->AddRectFilled(
        ImVec2(0.0f, 0.0f), displaySize, IM_COL32(22, 26, 34, 255));

    const float scale = 150.0f;
    const ImVec2 origin(displaySize.x * 0.55f, displaySize.y * 0.72f);
    auto worldToScreen = [&](double worldX, double worldY) {
        return ImVec2(
            origin.x + static_cast<float>(worldX * scale),
            origin.y - static_cast<float>(worldY * scale));
    };

    const ImVec2 pivot = worldToScreen(y(0), 0.0);
    const ImVec2 trackLeft = worldToScreen(p.trackMin, 0.0);
    const ImVec2 trackRight = worldToScreen(p.trackMax, 0.0);
    const float groundY = pivot.y + 45.0f;

    drawList->AddLine(
        ImVec2(trackLeft.x, groundY), ImVec2(trackRight.x, groundY),
        IM_COL32(110, 120, 140, 255), 4.0f);
    drawList->AddRectFilled(
        ImVec2(trackLeft.x - 12.0f, groundY - 65.0f),
        ImVec2(trackLeft.x + 4.0f, groundY),
        IM_COL32(170, 75, 70, 255));
    drawList->AddRectFilled(
        ImVec2(trackRight.x - 4.0f, groundY - 65.0f),
        ImVec2(trackRight.x + 12.0f, groundY),
        IM_COL32(170, 75, 70, 255));

    drawList->AddRectFilled(
        ImVec2(pivot.x - 50.0f, pivot.y),
        ImVec2(pivot.x + 50.0f, pivot.y + 32.0f),
        IM_COL32(70, 140, 230, 255), 5.0f);

    ImVec2 joint = pivot;
    double jointX = y(0);
    double jointY = 0.0;
    for (std::size_t rod = 0; rod < p.linkCount(); ++rod) {
        const double theta = y(1 + rod);
        jointX += p.rodLengths[rod] * std::sin(theta);
        jointY += p.rodLengths[rod] * std::cos(theta);
        const ImVec2 next = worldToScreen(jointX, jointY);

        drawList->AddLine(
            joint, next, IM_COL32(250, 190, 60, 255), 10.0f);
        drawList->AddCircleFilled(joint, 9.0f, IM_COL32(255, 255, 255, 255));
        joint = next;
    }
    drawList->AddCircleFilled(joint, 12.0f, IM_COL32(240, 90, 70, 255));
}
