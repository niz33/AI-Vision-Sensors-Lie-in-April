#include <raylib.h>
#include "vision_algo.hpp"
#include <string>
#include <cstdio>
#include <cstring>

int activeInputId = -1;
char inputText[32] = "";

// Custom slider with an adjacent input field
bool GuiSlider(int id, Rectangle bounds, const char *text, float *value, float minValue, float maxValue) {
    bool valueChanged = false;
    
    // Calculate layout
    int labelWidth = MeasureText(text, 10);
    DrawText(text, bounds.x, bounds.y + bounds.height / 2 - 5, 10, BLACK);
    
    Rectangle sliderBounds = {bounds.x + labelWidth + 10, bounds.y, bounds.width, bounds.height};
    Rectangle inputBounds = {sliderBounds.x + sliderBounds.width + 10, bounds.y, 40, bounds.height};
    
    Vector2 mousePos = GetMousePosition();
    bool sliderHovered = CheckCollisionPointRec(mousePos, sliderBounds);
    bool inputHovered = CheckCollisionPointRec(mousePos, inputBounds);

    // Input Box Logic
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        if (inputHovered) {
            activeInputId = id;
            snprintf(inputText, sizeof(inputText), "%.2f", *value);
        } else if (activeInputId == id) {
            // Apply if clicked outside
            try { *value = std::stof(inputText); valueChanged = true; } catch(...) {}
            activeInputId = -1;
        }
    }

    if (activeInputId == id) {
        int key = GetCharPressed();
        while (key > 0) {
            if ((key >= 32) && (key <= 125) && (strlen(inputText) < 31)) {
                int len = strlen(inputText);
                inputText[len] = (char)key;
                inputText[len+1] = '\0';
            }
            key = GetCharPressed();
        }

        if (IsKeyPressed(KEY_BACKSPACE)) {
            int len = strlen(inputText);
            if (len > 0) inputText[len-1] = '\0';
        }

        if (IsKeyPressed(KEY_ENTER)) {
            try { *value = std::stof(inputText); valueChanged = true; } catch(...) {}
            activeInputId = -1;
        }
    }

    // Slider Logic
    if (sliderHovered && IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
        *value = minValue + ((mousePos.x - sliderBounds.x) / sliderBounds.width) * (maxValue - minValue);
        valueChanged = true;
    }

    if (*value < minValue) *value = minValue;
    if (*value > maxValue) *value = maxValue;

    // Draw Slider
    DrawRectangleRec(sliderBounds, LIGHTGRAY);
    DrawRectangle(sliderBounds.x, sliderBounds.y, (int)(((*value - minValue) / (maxValue - minValue)) * sliderBounds.width), sliderBounds.height, DARKGRAY);
    DrawRectangleLinesEx(sliderBounds, 1, BLACK);

    // Draw Input Field
    DrawRectangleRec(inputBounds, RAYWHITE);
    DrawRectangleLinesEx(inputBounds, 1, (activeInputId == id) ? RED : BLACK);

    if (activeInputId == id) {
        DrawText(TextFormat("%s_", inputText), inputBounds.x + 4, inputBounds.y + inputBounds.height / 2 - 5, 10, RED);
    } else {
        DrawText(TextFormat("%.2f", *value), inputBounds.x + 4, inputBounds.y + inputBounds.height / 2 - 5, 10, BLACK);
    }

    return valueChanged;
}

int main() {
    // Initialize Raylib Window
    const int screenWidth = 1200;
    const int screenHeight = 600;
    InitWindow(screenWidth, screenHeight, "AprilTag Projection Algorithm Visualization");
    SetTargetFPS(60);

    CameraSettings cam;
    cam.resX = 640;
    cam.resY = 480;
    cam.fov = 60.0 * PI / 180.0; // ~60 degrees in radians

    Vec3 location = {0.5, -2.0, 1.0};
    Vec3 lookDirection = {0.0, 1.0, -0.3};

    float locX = location.x, locY = location.y, locZ = location.z;
    float dirX = lookDirection.x, dirY = lookDirection.y, dirZ = lookDirection.z;
    float fovDeg = 60.0f;
    float resXF = 640.0f, resYF = 480.0f;

    while (!WindowShouldClose()) {
        location = {locX, locY, locZ};
        lookDirection = {dirX, dirY, dirZ};
        lookDirection = normalize(lookDirection); // Keep it a unit vector
        cam.fov = fovDeg * PI / 180.0f;
        cam.resX = (int)resXF;
        cam.resY = (int)resYF;
        
        // Update intrinsics with the latest GUI values
        cam.updateIntrinsics();

        // Calculate Projection
        auto coords = projectAprilTag(location, lookDirection, cam);

        // --- DRAWING ---
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // --- GUI Controls (Moved to Top) ---
        int uiX = 20, uiY = 40;
        DrawText("Camera Settings:", uiX, uiY - 30, 20, DARKGRAY);
        GuiSlider(1, {(float)uiX, (float)uiY, 100, 15}, "Loc X", &locX, -5.0f, 5.0f);
        GuiSlider(2, {(float)uiX + 220, (float)uiY, 100, 15}, "Loc Y", &locY, -5.0f, 5.0f);
        GuiSlider(3, {(float)uiX + 440, (float)uiY, 100, 15}, "Loc Z", &locZ, 0.1f, 5.0f);
        GuiSlider(8, {(float)uiX + 660, (float)uiY, 100, 15}, "Res X", &resXF, 100.0f, 1200.0f);
        
        GuiSlider(4, {(float)uiX, (float)uiY + 25, 100, 15}, "Dir X", &dirX, -1.0f, 1.0f);
        GuiSlider(5, {(float)uiX + 220, (float)uiY + 25, 100, 15}, "Dir Y", &dirY, -1.0f, 1.0f);
        GuiSlider(6, {(float)uiX + 440, (float)uiY + 25, 100, 15}, "Dir Z", &dirZ, -1.0f, 1.0f);
        GuiSlider(9, {(float)uiX + 660, (float)uiY + 25, 100, 15}, "Res Y", &resYF, 100.0f, 1200.0f);

        GuiSlider(7, {(float)uiX, (float)uiY + 50, 100, 15}, "FOV (deg)", &fovDeg, 10.0f, 150.0f);

        // Show Computed Screen Coords
        DrawText("Projected Screen Coordinates:", uiX + 220, uiY + 55, 10, DARKGRAY);
        for (int i = 0; i < 4; ++i) {
            DrawText(TextFormat("P%d: (%.1f, %.1f)", i, coords[i].x, coords[i].y), uiX + 390 + (i * 120), uiY + 55, 12, RED);
        }

        // 1. Draw Simulated Camera Screen (Moved Down)
        int screenOffsetX = 20;
        int screenOffsetY = 140;
        DrawText(TextFormat("Camera View (%dx%d)", cam.resX, cam.resY), screenOffsetX, screenOffsetY - 25, 20, DARKGRAY);
        DrawRectangleLines(screenOffsetX - 1, screenOffsetY - 1, cam.resX + 2, cam.resY + 2, BLACK);
        DrawRectangle(screenOffsetX, screenOffsetY, cam.resX, cam.resY, LIGHTGRAY);

        for(int i = 0; i < 4; ++i) {
            Vector2 p1 = {(float)coords[i].x + screenOffsetX, (float)coords[i].y + screenOffsetY};
            Vector2 p2 = {(float)coords[(i+1)%4].x + screenOffsetX, (float)coords[(i+1)%4].y + screenOffsetY};
            
            // Only draw if roughly within/near bounds
            DrawLineEx(p1, p2, 3.0f, RED);
            DrawCircleV(p1, 5.0f, DARKBLUE);
            DrawText(TextFormat("P%d", i), (int)p1.x + 10, (int)p1.y - 10, 10, DARKGRAY);
        }

        // 2. Draw Top-Down View (Moved Down)
        int topDownOffsetX = 950;
        int topDownOffsetY = 390; // Center coordinate
        float scale = 50.0f; // Pixels per unit
        
        DrawText("Top-Down View (XY Plane)", topDownOffsetX - 20, screenOffsetY - 25, 20, DARKGRAY);
        
        // Draw Grid
        for(int i = -5; i <= 5; ++i) {
            DrawLine(topDownOffsetX + i * scale, topDownOffsetY - 5 * scale, topDownOffsetX + i * scale, topDownOffsetY + 5 * scale, Fade(LIGHTGRAY, 0.5f));
            DrawLine(topDownOffsetX - 5 * scale, topDownOffsetY + i * scale, topDownOffsetX + 5 * scale, topDownOffsetY + i * scale, Fade(LIGHTGRAY, 0.5f));
        }

        // Draw AprilTag (1x1 at origin)
        DrawRectangle(topDownOffsetX, topDownOffsetY - scale, scale, scale, Fade(BLACK, 0.5f));

        // Draw Camera
        Vector2 camPos2D = { topDownOffsetX + (float)location.x * scale, topDownOffsetY - (float)location.y * scale };
        DrawCircleV(camPos2D, 6.0f, BLUE);
        
        // Draw Look Direction Vector
        Vector2 lookEnd2D = { camPos2D.x + (float)lookDirection.x * scale * 2, camPos2D.y - (float)lookDirection.y * scale * 2 };
        DrawLineEx(camPos2D, lookEnd2D, 2.0f, RED);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}