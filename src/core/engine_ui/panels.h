#pragma once

class VulkanEngine;

namespace vk_engine::debug_ui
{
    void ui_window(VulkanEngine *eng);
    void ui_background(VulkanEngine *eng);
    void ui_particles(VulkanEngine *eng);
    void ui_ibl(VulkanEngine *eng);
    void ui_overview(VulkanEngine *eng);
    void ui_camera(VulkanEngine *eng);
    void ui_textures(VulkanEngine *eng);
    void ui_async_assets(VulkanEngine *eng);
    void ui_shadows(VulkanEngine *eng);
    void ui_render_graph(VulkanEngine *eng);
    void ui_pipelines(VulkanEngine *eng);
    void ui_postfx(VulkanEngine *eng);
    void ui_scene_editor(VulkanEngine *eng);
    void ui_lights(VulkanEngine *eng);
    void ui_picking_gizmo(VulkanEngine *eng);
    void ui_planets(VulkanEngine *eng);
    void ui_debug_draw(VulkanEngine *eng);
}
