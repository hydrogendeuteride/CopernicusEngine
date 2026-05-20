// ImGui debug UI entry point for VulkanEngine.

#include "engine.h"
#include "core/engine_ui/panels.h"

#include "imgui.h"
#include "ImGuizmo.h"

namespace
{
    bool g_show_debug_window = false;
} // namespace

void vk_engine_draw_debug_ui(VulkanEngine *eng)
{
    if (!eng) return;

    ImGuizmo::BeginFrame();

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("Engine Debug", nullptr, &g_show_debug_window);
            ImGui::EndMenu();
        }

        ImGui::Separator();
        ImGui::Text("%.1f ms | %d tris | %d draws",
                    eng->stats.frametime,
                    eng->stats.triangle_count,
                    eng->stats.drawcall_count);

        ImGui::EndMainMenuBar();
    }

    if (!g_show_debug_window)
    {
        return;
    }

    namespace ui = vk_engine::debug_ui;

    ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Engine Debug", &g_show_debug_window))
    {
        if (ImGui::BeginTabBar("DebugTabs", ImGuiTabBarFlags_None))
        {
            if (ImGui::BeginTabItem("Overview")) { ui::ui_overview(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Scene Editor")) { ui::ui_scene_editor(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Lights")) { ui::ui_lights(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Picking & Gizmo")) { ui::ui_picking_gizmo(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Camera")) { ui::ui_camera(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Planets")) { ui::ui_planets(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Render Graph")) { ui::ui_render_graph(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Pipelines")) { ui::ui_pipelines(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Shadows")) { ui::ui_shadows(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("IBL")) { ui::ui_ibl(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("PostFX")) { ui::ui_postfx(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Background")) { ui::ui_background(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Particles")) { ui::ui_particles(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Debug Draw")) { ui::ui_debug_draw(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Window")) { ui::ui_window(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Textures")) { ui::ui_textures(eng); ImGui::EndTabItem(); }
            if (ImGui::BeginTabItem("Async Assets")) { ui::ui_async_assets(eng); ImGui::EndTabItem(); }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();
}
