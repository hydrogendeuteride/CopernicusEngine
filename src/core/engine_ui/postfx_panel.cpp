#include "core/engine_ui/common.h"

namespace vk_engine::debug_ui
{
    void ui_postfx(VulkanEngine *eng)
    {
        if (!eng) return;
        if (!eng->_context) return;

        EngineContext *ctx = eng->_context.get();

        ImGui::TextUnformatted("Reflections");
        bool ssrEnabled = ctx->enableSSR;
        if (ImGui::Checkbox("Enable Screen-Space Reflections", &ssrEnabled))
        {
            ctx->enableSSR = ssrEnabled;
        }

        int reflMode = static_cast<int>(ctx->reflectionMode);
        ImGui::TextUnformatted("Reflection Mode");
        ImGui::RadioButton("SSR only", &reflMode, 0);
        ImGui::SameLine();
        ImGui::RadioButton("SSR + RT fallback", &reflMode, 1);
        ImGui::SameLine();
        ImGui::RadioButton("RT only", &reflMode, 2);

        const bool rq = eng->_deviceManager->supportsRayQuery();
        const bool as = eng->_deviceManager->supportsAccelerationStructure();
        if (!(rq && as) && reflMode != 0)
        {
            reflMode = 0; // guard for unsupported HW
        }
        ctx->reflectionMode = static_cast<uint32_t>(reflMode);

        ImGui::Separator();
        ImGui::TextUnformatted("Volumetrics");
        bool voxEnabled = ctx->enableVolumetrics;
        if (ImGui::Checkbox("Enable Voxel Volumetrics (Cloud/Smoke/Flame)", &voxEnabled))
        {
            ctx->enableVolumetrics = voxEnabled;
        }

        const char *typeLabels[] = {"Clouds", "Smoke", "Flame"};

        for (uint32_t i = 0; i < EngineContext::MAX_VOXEL_VOLUMES; ++i)
        {
            VoxelVolumeSettings &vs = ctx->voxelVolumes[i];

            std::string header = "Voxel Volume " + std::to_string(i);
            if (!ImGui::TreeNode(header.c_str()))
            {
                continue;
            }

            std::string id = "##vox" + std::to_string(i);
            ImGui::Checkbox(("Enabled" + id).c_str(), &vs.enabled);

            int type = static_cast<int>(vs.type);
            if (ImGui::Combo(("Type" + id).c_str(), &type, typeLabels, IM_ARRAYSIZE(typeLabels)))
            {
                type = std::clamp(type, 0, 2);
                vs.type = static_cast<VoxelVolumeType>(type);
            }

            ImGui::Checkbox(("Follow Camera XZ" + id).c_str(), &vs.followCameraXZ);
            ImGui::Checkbox(("Animate Voxels" + id).c_str(), &vs.animateVoxels);

            if (vs.followCameraXZ)
            {
                ImGui::InputFloat3(("Volume Offset (local)" + id).c_str(), &vs.volumeCenterLocal.x);
            }
            else
            {
                ImGui::InputFloat3(("Volume Center (local)" + id).c_str(), &vs.volumeCenterLocal.x);
            }
            ImGui::InputFloat3(("Volume Velocity (local)" + id).c_str(), &vs.volumeVelocityLocal.x);
            ImGui::InputFloat3(("Volume Half Extents" + id).c_str(), &vs.volumeHalfExtents.x);
            vs.volumeHalfExtents.x = std::max(vs.volumeHalfExtents.x, 0.01f);
            vs.volumeHalfExtents.y = std::max(vs.volumeHalfExtents.y, 0.01f);
            vs.volumeHalfExtents.z = std::max(vs.volumeHalfExtents.z, 0.01f);

            ImGui::SliderFloat(("Density Scale" + id).c_str(), &vs.densityScale, 0.0f, 6.0f);
            ImGui::SliderFloat(("Coverage" + id).c_str(), &vs.coverage, 0.0f, 0.95f);
            ImGui::SliderFloat(("Extinction" + id).c_str(), &vs.extinction, 0.0f, 8.0f);
            ImGui::SliderInt(("Steps" + id).c_str(), &vs.stepCount, 8, 256);

            int gridRes = static_cast<int>(vs.gridResolution);
            if (ImGui::SliderInt(("Grid Resolution" + id).c_str(), &gridRes, 16, 128))
            {
                vs.gridResolution = static_cast<uint32_t>(std::max(4, gridRes));
            }

            if (vs.animateVoxels)
            {
                ImGui::InputFloat3(("Wind Velocity (local)" + id).c_str(), &vs.windVelocityLocal.x);
                ImGui::SliderFloat(("Dissipation" + id).c_str(), &vs.dissipation, 0.0f, 6.0f);
                ImGui::SliderFloat(("Noise Strength" + id).c_str(), &vs.noiseStrength, 0.0f, 6.0f);
                ImGui::SliderFloat(("Noise Scale" + id).c_str(), &vs.noiseScale, 0.25f, 32.0f);
                ImGui::SliderFloat(("Noise Speed" + id).c_str(), &vs.noiseSpeed, 0.0f, 8.0f);

                if (vs.type != VoxelVolumeType::Clouds)
                {
                    ImGui::InputFloat3(("Emitter UVW" + id).c_str(), &vs.emitterUVW.x);
                    ImGui::SliderFloat(("Emitter Radius" + id).c_str(), &vs.emitterRadius, 0.01f, 0.5f);
                }
            }

            ImGui::ColorEdit3(("Albedo/Tint" + id).c_str(), &vs.albedo.x);
            ImGui::SliderFloat(("Scatter Strength" + id).c_str(), &vs.scatterStrength, 0.0f, 2.0f);

            if (vs.type == VoxelVolumeType::Flame)
            {
                ImGui::ColorEdit3(("Emission Color" + id).c_str(), &vs.emissionColor.x);
                ImGui::SliderFloat(("Emission Strength" + id).c_str(), &vs.emissionStrength, 0.0f, 25.0f);
            }
            else
            {
                vs.emissionStrength = 0.0f;
            }

            ImGui::TreePop();
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Rocket Plumes");
        bool plumesEnabled = ctx->enableRocketPlumes;
        if (ImGui::Checkbox("Enable Rocket Plumes (Analytic Raymarch)", &plumesEnabled))
        {
            ctx->enableRocketPlumes = plumesEnabled;
        }

        ImGui::SliderInt("Plume Steps##rocketplume", &ctx->rocketPlumeSteps, 8, 256);

        {
            // Noise texture selection (relative to assets/).
            static const char *noiseOptions[] = {"vfx/simplex.ktx2", "vfx/perlin.ktx2"};
            int cur = -1;
            for (int i = 0; i < IM_ARRAYSIZE(noiseOptions); ++i)
            {
                if (ctx->rocketPlumeNoiseTexturePath == noiseOptions[i])
                {
                    cur = i;
                    break;
                }
            }

            const char *curLabel = (cur >= 0) ? noiseOptions[cur] : "Custom";
            if (ImGui::BeginCombo("Noise Preset##rocketplume", curLabel))
            {
                for (int i = 0; i < IM_ARRAYSIZE(noiseOptions); ++i)
                {
                    const bool selected = (cur == i);
                    if (ImGui::Selectable(noiseOptions[i], selected))
                    {
                        ctx->rocketPlumeNoiseTexturePath = noiseOptions[i];
                    }
                    if (selected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            static char plumeNoisePathBuf[256]{};
            const std::string &pathRef = ctx->rocketPlumeNoiseTexturePath;
            if (std::strncmp(plumeNoisePathBuf, pathRef.c_str(), sizeof(plumeNoisePathBuf)) != 0)
            {
                std::snprintf(plumeNoisePathBuf, sizeof(plumeNoisePathBuf), "%s", pathRef.c_str());
            }
            if (ImGui::InputText("Noise Path (assets/)##rocketplume", plumeNoisePathBuf, sizeof(plumeNoisePathBuf)))
            {
                ctx->rocketPlumeNoiseTexturePath = plumeNoisePathBuf;
            }
        }

        for (uint32_t i = 0; i < EngineContext::MAX_ROCKET_PLUMES; ++i)
        {
            RocketPlumeSettings &ps = ctx->rocketPlumes[i];

            std::string header = "Rocket Plume " + std::to_string(i);
            if (!ImGui::TreeNode(header.c_str()))
            {
                continue;
            }

            std::string id = "##rplume" + std::to_string(i);

            ImGui::Checkbox(("Enabled" + id).c_str(), &ps.enabled);

            ImGui::SliderFloat(("Length" + id).c_str(), &ps.length, 0.1f, 250.0f);
            ImGui::SliderFloat(("Nozzle Radius" + id).c_str(), &ps.nozzleRadius, 0.0f, 5.0f);

            float angDeg = glm::degrees(ps.expansionAngleRad);
            if (ImGui::SliderFloat(("Expansion Angle (deg)" + id).c_str(), &angDeg, 0.0f, 89.0f))
            {
                ps.expansionAngleRad = glm::radians(angDeg);
            }
            ImGui::SliderFloat(("Radius Exp" + id).c_str(), &ps.radiusExp, 0.0f, 4.0f);

            ImGui::SliderFloat(("Intensity" + id).c_str(), &ps.intensity, 0.0f, 50.0f);
            ImGui::ColorEdit3(("Core Color" + id).c_str(), &ps.coreColor.x);
            ImGui::ColorEdit3(("Plume Color" + id).c_str(), &ps.plumeColor.x);
            ImGui::SliderFloat(("Core Length" + id).c_str(), &ps.coreLength, 0.0f, 50.0f);
            ImGui::SliderFloat(("Core Strength" + id).c_str(), &ps.coreStrength, 0.0f, 10.0f);

            ImGui::SliderFloat(("Radial Falloff" + id).c_str(), &ps.radialFalloff, 0.0f, 16.0f);
            ImGui::SliderFloat(("Axial Falloff" + id).c_str(), &ps.axialFalloff, 0.0f, 16.0f);

            ImGui::SliderFloat(("Noise Strength" + id).c_str(), &ps.noiseStrength, 0.0f, 2.0f);
            ImGui::SliderFloat(("Noise Scale" + id).c_str(), &ps.noiseScale, 0.1f, 32.0f);
            ImGui::SliderFloat(("Noise Speed" + id).c_str(), &ps.noiseSpeed, 0.0f, 8.0f);

            ImGui::SliderFloat(("Shock Strength" + id).c_str(), &ps.shockStrength, 0.0f, 2.0f);
            ImGui::SliderFloat(("Shock Frequency" + id).c_str(), &ps.shockFrequency, 0.0f, 64.0f);

            ImGui::SliderFloat(("Soft Absorption" + id).c_str(), &ps.softAbsorption, 0.0f, 10.0f);

            ImGui::TreePop();
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Atmosphere");
        bool atmEnabled = ctx->enableAtmosphere;
        if (ImGui::Checkbox("Enable Atmosphere Scattering", &atmEnabled))
        {
            ctx->enableAtmosphere = atmEnabled;
        }

        AtmosphereSettings &atm = ctx->atmosphere;
        if (ImGui::BeginCombo("Planet", atm.bodyName.empty() ? "Auto (closest)" : atm.bodyName.c_str()))
        {
            const bool autoSelected = atm.bodyName.empty();
            if (ImGui::Selectable("Auto (closest)", autoSelected))
            {
                atm.bodyName.clear();
            }

            if (ctx->scene)
            {
                if (PlanetSystem *planets = ctx->scene->get_planet_system())
                {
                    for (const PlanetSystem::PlanetBody &b : planets->bodies())
                    {
                        const bool selected = (!atm.bodyName.empty() && atm.bodyName == b.name);
                        if (ImGui::Selectable(b.name.c_str(), selected))
                        {
                            atm.bodyName = b.name;
                        }
                    }
                }
            }

            ImGui::EndCombo();
        }

        if (ImGui::Button("Reset Earth Params"))
        {
            std::string keepName = atm.bodyName;
            atm = AtmosphereSettings{};
            atm.bodyName = std::move(keepName);
        }

        float atmHeightKm = atm.atmosphereHeightM / 1000.0f;
        float HrKm = atm.rayleighScaleHeightM / 1000.0f;
        float HmKm = atm.mieScaleHeightM / 1000.0f;

        if (ImGui::SliderFloat("Atmosphere Height (km)", &atmHeightKm, 1.0f, 200.0f, "%.2f"))
        {
            atm.atmosphereHeightM = std::max(0.0f, atmHeightKm * 1000.0f);
        }
        if (ImGui::SliderFloat("Rayleigh Scale Height (km)", &HrKm, 1.0f, 20.0f, "%.2f"))
        {
            atm.rayleighScaleHeightM = std::max(1.0f, HrKm * 1000.0f);
        }
        if (ImGui::SliderFloat("Mie Scale Height (km)", &HmKm, 0.1f, 10.0f, "%.2f"))
        {
            atm.mieScaleHeightM = std::max(1.0f, HmKm * 1000.0f);
        }

        ImGui::SliderFloat("Mie g", &atm.mieG, -0.2f, 0.99f, "%.3f");
        ImGui::SliderFloat("Intensity", &atm.intensity, 0.0f, 4.0f, "%.2f");
        ImGui::ColorEdit3("Absorption Color", &atm.absorptionColor.x);
        float absMicro = atm.absorptionStrength * 1.0e6f;
        if (ImGui::SliderFloat("Absorption Strength (x1e-6 1/m)", &absMicro, 0.0f, 100.0f, "%.2f"))
        {
            atm.absorptionStrength = std::max(0.0f, absMicro) * 1.0e-6f;
        }
        ImGui::SliderFloat("Sun Disk", &atm.sunDiskIntensity, 0.0f, 10.0f, "%.2f");
        ImGui::SliderFloat("Sun Halo", &atm.sunHaloIntensity, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("Halo Radius (deg)", &atm.sunHaloRadiusDeg, 0.0f, 20.0f, "%.2f");
        ImGui::Checkbox("Anamorphic Streak", &atm.sunAnamorphicStreakEnabled);
        ImGui::SliderFloat("Sun Starburst", &atm.sunStarburstIntensity, 0.0f, 4.0f, "%.2f");
        ImGui::SliderFloat("Starburst Radius (deg)", &atm.sunStarburstRadiusDeg, 0.0f, 30.0f, "%.2f");
        int spikes = std::clamp(atm.sunStarburstSpikes, 2, 64);
        if (ImGui::SliderInt("Starburst Spikes", &spikes, 2, 32))
        {
            // Keep it even by default to produce a symmetrical ray pattern.
            spikes = std::max(2, spikes);
            spikes &= ~1;
            atm.sunStarburstSpikes = spikes;
        }
        ImGui::SliderFloat("Starburst Sharpness", &atm.sunStarburstSharpness, 1.0f, 64.0f, "%.1f");
        ImGui::SliderFloat("Jitter", &atm.jitterStrength, 0.0f, 1.0f, "%.2f");
        static char jitterPathBuf[256]{};
        const std::string &jitterPathRef = atm.jitterTexturePath;
        if (std::strncmp(jitterPathBuf, jitterPathRef.c_str(), sizeof(jitterPathBuf)) != 0)
        {
            std::snprintf(jitterPathBuf, sizeof(jitterPathBuf), "%s", jitterPathRef.c_str());
        }
        if (ImGui::InputText("Jitter Noise Path (assets/)", jitterPathBuf, sizeof(jitterPathBuf)))
        {
            atm.jitterTexturePath = jitterPathBuf;
        }
        ImGui::SliderFloat("Planet Snap (m)", &atm.planetSurfaceSnapM, 0.0f, 2000.0f, "%.1f");
        ImGui::SliderInt("View Steps", &atm.viewSteps, 4, 64);
        ImGui::SliderInt("Light Steps", &atm.lightSteps, 2, 32);

        ImGui::Separator();
        ImGui::TextUnformatted("Planet Clouds");
        bool planetCloudsEnabled = ctx->enablePlanetClouds;
        if (ImGui::Checkbox("Enable Planet Clouds", &planetCloudsEnabled))
        {
            ctx->enablePlanetClouds = planetCloudsEnabled;
        }

        PlanetCloudSettings &clouds = ctx->planetClouds;
        if (ImGui::Button("Reset Cloud Params"))
        {
            clouds = PlanetCloudSettings{};
        }

        static char overlayPathBuf[256]{};
        const std::string &pathRef = clouds.overlayTexturePath;
        if (std::strncmp(overlayPathBuf, pathRef.c_str(), sizeof(overlayPathBuf)) != 0)
        {
            std::snprintf(overlayPathBuf, sizeof(overlayPathBuf), "%s", pathRef.c_str());
        }

        if (ImGui::InputText("Overlay Path (assets/)", overlayPathBuf, sizeof(overlayPathBuf)))
        {
            clouds.overlayTexturePath = overlayPathBuf;
        }

        static char noisePathBuf[256]{};
        const std::string &noisePathRef = clouds.noiseTexturePath;
        if (std::strncmp(noisePathBuf, noisePathRef.c_str(), sizeof(noisePathBuf)) != 0)
        {
            std::snprintf(noisePathBuf, sizeof(noisePathBuf), "%s", noisePathRef.c_str());
        }
        if (ImGui::InputText("Noise Path (assets/)", noisePathBuf, sizeof(noisePathBuf)))
        {
            clouds.noiseTexturePath = noisePathBuf;
        }

        static char noise3DPathBuf[256]{};
        const std::string &noise3DPathRef = clouds.noiseTexture3DPath;
        if (std::strncmp(noise3DPathBuf, noise3DPathRef.c_str(), sizeof(noise3DPathBuf)) != 0)
        {
            std::snprintf(noise3DPathBuf, sizeof(noise3DPathBuf), "%s", noise3DPathRef.c_str());
        }
        if (ImGui::InputText("Noise 3D Path (assets/)", noise3DPathBuf, sizeof(noise3DPathBuf)))
        {
            clouds.noiseTexture3DPath = noise3DPathBuf;
        }
        ImGui::TextUnformatted("Overlay drives macro coverage; 2D noise shapes weather; optional 3D noise breaks up internal repetition.");

        float rotDeg = glm::degrees(clouds.overlayRotationRad);
        if (ImGui::SliderFloat("Overlay Rotation (deg)", &rotDeg, -180.0f, 180.0f, "%.1f"))
        {
            clouds.overlayRotationRad = glm::radians(rotDeg);
        }
        ImGui::Checkbox("Overlay Flip V", &clouds.overlayFlipV);

        float baseKm = clouds.baseHeightM / 1000.0f;
        float thickKm = clouds.thicknessM / 1000.0f;
        if (ImGui::SliderFloat("Base Height (km)", &baseKm, 0.0f, 50.0f, "%.2f"))
        {
            clouds.baseHeightM = std::max(0.0f, baseKm * 1000.0f);
        }
        if (ImGui::SliderFloat("Thickness (km)", &thickKm, 0.1f, 50.0f, "%.2f"))
        {
            clouds.thicknessM = std::max(0.0f, thickKm * 1000.0f);
        }

        ImGui::SliderFloat("Density Scale", &clouds.densityScale, 0.0f, 8.0f, "%.2f");
        clouds.densityScale = std::max(0.0f, clouds.densityScale);
        ImGui::ColorEdit3("Cloud Color", &clouds.color.x);
        clouds.color = glm::clamp(clouds.color, glm::vec3(0.0f), glm::vec3(1.0f));

        ImGui::SliderFloat("Coverage", &clouds.coverage, 0.0f, 0.99f, "%.3f");
        clouds.coverage = std::clamp(clouds.coverage, 0.0f, 0.999f);

        ImGui::SliderFloat("Weather Scale", &clouds.noiseScale, 0.05f, 16.0f, "%.3f");
        clouds.noiseScale = std::max(0.001f, clouds.noiseScale);
        ImGui::SliderFloat("Internal Detail Scale", &clouds.detailScale, 0.25f, 64.0f, "%.3f");
        clouds.detailScale = std::max(0.001f, clouds.detailScale);
        ImGui::SliderFloat("Noise Blend", &clouds.noiseBlend, 0.0f, 1.0f, "%.3f");
        clouds.noiseBlend = std::clamp(clouds.noiseBlend, 0.0f, 1.0f);
        ImGui::SliderFloat("Internal Erosion", &clouds.detailErode, 0.0f, 1.0f, "%.3f");
        clouds.detailErode = std::clamp(clouds.detailErode, 0.0f, 1.0f);

        ImGui::SliderFloat("Wind Speed (m/s)", &clouds.windSpeed, -200.0f, 200.0f, "%.1f");
        float windDeg = glm::degrees(clouds.windAngleRad);
        if (ImGui::SliderFloat("Wind Angle (deg)", &windDeg, 0.0f, 360.0f, "%.1f"))
        {
            clouds.windAngleRad = glm::radians(windDeg);
        }

        ImGui::SliderInt("Cloud Steps", &clouds.cloudSteps, 4, 128);
        clouds.cloudSteps = std::clamp(clouds.cloudSteps, 4, 128);

        ImGui::Separator();
        if (auto *tm = eng->_renderPassManager ? eng->_renderPassManager->getPass<TonemapPass>() : nullptr)
        {
            AutoExposurePass *ae = eng->_renderPassManager->getPass<AutoExposurePass>();
            bool autoExpEnabled = (ae != nullptr) ? ae->enabled() : false;
            if (ae)
            {
                if (ImGui::Checkbox("Auto Exposure", &autoExpEnabled))
                {
                    ae->set_enabled(autoExpEnabled, tm->exposure());
                }
                if (autoExpEnabled)
                {
                    float comp = ae->compensation();
                    if (ImGui::SliderFloat("Exposure Compensation", &comp, 0.05f, 8.0f, "%.3f"))
                    {
                        ae->set_compensation(comp);
                    }
                    float key = ae->key_value();
                    if (ImGui::SliderFloat("Key (middle grey)", &key, 0.02f, 0.5f, "%.3f"))
                    {
                        ae->set_key_value(key);
                    }
                    float minExp = ae->min_exposure();
                    if (ImGui::SliderFloat("Min Exposure", &minExp, 0.01f, 8.0f, "%.3f"))
                    {
                        ae->set_min_exposure(minExp);
                    }
                    float maxExp = ae->max_exposure();
                    if (ImGui::SliderFloat("Max Exposure", &maxExp, 0.01f, 16.0f, "%.3f"))
                    {
                        ae->set_max_exposure(maxExp);
                    }
                    float speedUp = ae->speed_up();
                    if (ImGui::SliderFloat("Speed Up", &speedUp, 0.0f, 10.0f, "%.2f"))
                    {
                        ae->set_speed_up(speedUp);
                    }
                    float speedDown = ae->speed_down();
                    if (ImGui::SliderFloat("Speed Down", &speedDown, 0.0f, 10.0f, "%.2f"))
                    {
                        ae->set_speed_down(speedDown);
                    }

                    ImGui::Text("Lavg %.4f", ae->last_luminance());
                    ImGui::Text("Exposure %.3f (target %.3f)", ae->exposure(), ae->target_exposure());
                }
            }

            float exp = tm->exposure();
            int mode = tm->mode();
            if (!autoExpEnabled)
            {
                if (ImGui::SliderFloat("Exposure", &exp, 0.05f, 8.0f)) { tm->setExposure(exp); }
            }
            else
            {
                ImGui::Text("Exposure (auto) %.3f", exp);
            }
            ImGui::TextUnformatted("Operator");
            ImGui::SameLine();
            if (ImGui::RadioButton("Reinhard", mode == 0))
            {
                mode = 0;
                tm->setMode(mode);
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("ACES", mode == 1))
            {
                mode = 1;
                tm->setMode(mode);
            }

            // Bloom controls
            bool bloomEnabled = tm->bloomEnabled();
            if (ImGui::Checkbox("Bloom", &bloomEnabled))
            {
                tm->setBloomEnabled(bloomEnabled);
            }
            float bloomThreshold = tm->bloomThreshold();
            if (ImGui::SliderFloat("Bloom Threshold", &bloomThreshold, 0.0f, 5.0f))
            {
                tm->setBloomThreshold(bloomThreshold);
            }
            float bloomIntensity = tm->bloomIntensity();
            if (ImGui::SliderFloat("Bloom Intensity", &bloomIntensity, 0.0f, 2.0f))
            {
                tm->setBloomIntensity(bloomIntensity);
            }
        }
        else
        {
            ImGui::TextUnformatted("Tonemap pass not available");
        }
    }

} // namespace vk_engine::debug_ui
