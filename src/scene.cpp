#include "triple/scene.hpp"
#include <cmath>

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
