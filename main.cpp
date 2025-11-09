#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#ifndef WINVER
#define WINVER _WIN32_WINNT
#endif
#endif

#include "httplib.h"
#include <libfreenect.h>
#include <SDL3/SDL.h>

#include <iostream>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <fstream>
#include <algorithm>
#include <cmath>


freenect_context* context = nullptr;
freenect_device* device = nullptr;

static const int W = 640;
static const int H = 480;

static std::vector<uint16_t> rawdepth(W* H, 0);
static std::vector<uint16_t> lastvalid(W* H, 0);
static std::vector<uint8_t>  depthbuffer(W* H * 3, 0);

static std::vector<float> water(W* H, 0.0f);
static std::vector<float> water_next(W* H, 0.0f);

std::mutex depth_mutex;

std::atomic<int>  minDepth{ 400 };
std::atomic<int>  maxDepth{ 1200 };
std::atomic<int>  contourStep{ 20 };
std::atomic<int>  waterLevel{ 700 };
std::atomic<bool> waterEnabled{ false };


std::string load_file(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return {};
    }
    std::string data((std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>());
    return data;
}

void height_to_color(int h, uint8_t& r, uint8_t& g, uint8_t& b) {
    if (h < 64) {
        r = 0; g = h * 4; b = 255;
    }
    else if (h < 128) {
        r = 0; g = 255; b = 255 - (h - 64) * 4;
    }
    else if (h < 192) {
        r = (h - 128) * 4; g = 255; b = 0;
    }
    else {
        r = 255; g = 255 - (h - 192) * 4; b = 0;
    }
}

void blur_depth()
{
    static std::vector<uint16_t> tmp(W * H);

    for (int y = 1; y < H - 1; ++y) {
        for (int x = 1; x < W - 1; ++x) {
            int i = y * W + x;
            int sum = 0;
            int count = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int j = (y + dy) * W + (x + dx);
                    uint16_t v = rawdepth[j];
                    if (v != 0) {
                        sum += v;
                        ++count;
                    }
                }
            }
            tmp[i] = count ? static_cast<uint16_t>(sum / count) : rawdepth[i];
        }
    }

    for (int x = 0; x < W; ++x) {
        tmp[x] = rawdepth[x];
        tmp[(H - 1) * W + x] = rawdepth[(H - 1) * W + x];
    }
    for (int y = 0; y < H; ++y) {
        tmp[y * W] = rawdepth[y * W];
        tmp[y * W + (W - 1)] = rawdepth[y * W + (W - 1)];
    }

    rawdepth.swap(tmp);
}

void update_water()
{
    const float flowFactor = 0.25f;

    std::fill(water_next.begin(), water_next.end(), 0.0f);

    for (int y = 1; y < H - 1; ++y) {
        for (int x = 1; x < W - 1; ++x) {
            int i = y * W + x;
            float w_here = water[i];
            if (w_here <= 0.0f) {
                continue;
            }

            float h_here = float(rawdepth[i]) + w_here;

            const int vx[4] = { -1, 1, 0, 0 };
            const int vy[4] = { 0, 0, -1, 1 };

            float remain = w_here;

            for (int k = 0; k < 4; ++k) {
                int nx = x + vx[k];
                int ny = y + vy[k];
                int j = ny * W + nx;

                float h_nei = float(rawdepth[j]) + water[j];
                float dh = h_here - h_nei;

                if (dh > 3.0f) {
                    float flow = dh * flowFactor;
                    if (flow > remain) flow = remain;

                    remain -= flow;
                    water_next[j] += flow;
                }
            }

            water_next[i] += remain;
        }
    }

    water.swap(water_next);

    int minD = minDepth.load();
    int maxD = maxDepth.load();
    for (int i = 0; i < W * H; ++i) {
        uint16_t v = rawdepth[i];
        if (v == 0 || v < minD || v > maxD) {
            water[i] = 0.0f;
        }
    }
}

void profundidade(freenect_device* /*dev*/, void* v_depth, uint32_t /*timestamp*/) {
    std::lock_guard<std::mutex> lock(depth_mutex);
    uint16_t* d16 = static_cast<uint16_t*>(v_depth);

    bool valid = false;
    for (int i = 0; i < W * H; i++) {
        if (d16[i] > 0) { valid = true; break; }
    }

    if (valid) {
        std::copy(d16, d16 + W * H, rawdepth.begin());
        lastvalid = rawdepth;
    }
    else {
        rawdepth = lastvalid;
    }
}

void process_frame() {
    std::lock_guard<std::mutex> lock(depth_mutex);

    blur_depth();
    if (waterEnabled.load()) {
        update_water();
    }

    const int minD = minDepth.load();
    const int maxD = maxDepth.load();
    const int step = contourStep.load();
    const int wLevel = waterLevel.load();
    const bool wOn = waterEnabled.load();

    for (int i = 0; i < W * H; i++) {
        uint16_t v = rawdepth[i];
        uint8_t r = 0, g = 0, b = 0;

        if (v == 0 || v < minD || v > maxD) {
            r = g = b = 0;
        }
        else {
            float t = float(v - minD) / float(maxD - minD);
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            t = powf(t, 0.7f);
            int h = int(t * 255.0f + 0.5f);

            height_to_color(h, r, g, b);

            if (step > 0 && (v % step) < 2) {
                r = g = b = 0;
            }

            if (wOn && v >= wLevel) {
                r = 0; g = 0; b = 200;
            }

            float w = water[i];
            if (wOn && w > 0.5f) {
                float alpha = std::min(w / 20.0f, 1.0f);
                uint8_t wr = 0, wg = 80, wb = 220;
                r = uint8_t(r * (1 - alpha) + wr * alpha);
                g = uint8_t(g * (1 - alpha) + wg * alpha);
                b = uint8_t(b * (1 - alpha) + wb * alpha);
            }

            if (i > W && i < (W * (H - 1))) {
                int dzdx = rawdepth[i - 1] - rawdepth[i + 1];
                int dzdy = rawdepth[i - W] - rawdepth[i + W];
                int shade = 128 + (dzdx + dzdy) / 16;
                if (shade < 50)  shade = 50;
                if (shade > 255) shade = 255;
                r = uint8_t((r * shade) / 255);
                g = uint8_t((g * shade) / 255);
                b = uint8_t((b * shade) / 255);
            }
        }

        depthbuffer[i * 3 + 0] = r;
        depthbuffer[i * 3 + 1] = g;
        depthbuffer[i * 3 + 2] = b;
    }
}


void run_web_server() {
    httplib::Server svr;

    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        std::string html = load_file("kinect_panel.html");
        if (html.empty()) {
            html = "<html><body>kinect_panel.html não encontrado.</body></html>";
        }
        res.set_content(html, "text/html; charset=UTF-8");
        });

    svr.Get("/state", [](const httplib::Request&, httplib::Response& res) {
        std::string json = "{";
        json += "\"minDepth\":" + std::to_string(minDepth.load()) + ",";
        json += "\"maxDepth\":" + std::to_string(maxDepth.load()) + ",";
        json += "\"contourStep\":" + std::to_string(contourStep.load()) + ",";
        json += "\"waterLevel\":" + std::to_string(waterLevel.load()) + ",";
        json += "\"waterEnabled\":" + std::string(waterEnabled.load() ? "true" : "false");
        json += "}";
        res.set_content(json, "application/json");
        });

    svr.Get("/set", [](const httplib::Request& req, httplib::Response& res) {
        auto set_int = [&](const char* name, std::atomic<int>& target) {
            if (req.has_param(name)) {
                try {
                    int v = std::stoi(req.get_param_value(name));
                    target.store(v);
                }
                catch (...) {}
            }
        };

        set_int("minDepth", minDepth);
        set_int("maxDepth", maxDepth);
        set_int("contourStep", contourStep);
        set_int("waterLevel", waterLevel);

        if (req.has_param("waterEnabled")) {
            std::string v = req.get_param_value("waterEnabled");
            waterEnabled.store(v == "1" || v == "true" || v == "on");
        }

        res.set_content("OK", "text/plain");
        });

    std::cout << "Servidor HTTP em http://0.0.0.0:8080/ \n";
    svr.listen("0.0.0.0", 8080);
}

int main() {
    std::thread web_thread(run_web_server);
    web_thread.detach();

    if (freenect_init(&context, nullptr) < 0) {
        std::cerr << "freenect_init falhou\n";
        return -1;
    }

    freenect_select_subdevices(context, FREENECT_DEVICE_CAMERA);

    if (freenect_open_device(context, &device, 0) < 0) {
        std::cerr << "freenect_open_device falhou\n";
        freenect_shutdown(context);
        return -1;
    }

    freenect_set_depth_callback(device, profundidade);
    freenect_set_depth_mode(
        device,
        freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM)
    );
    freenect_start_depth(device);

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL_Init falhou\n";
        freenect_close_device(device);
        freenect_shutdown(context);
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow("Sandbox", 1280, 720, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    SDL_Texture* texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_RGB24,
        SDL_TEXTUREACCESS_STREAMING,
        W, H
    );

    SDL_SetTextureScaleMode(texture, SDL_SCALEMODE_LINEAR);
    SDL_SetRenderLogicalPresentation(renderer, W, H, SDL_LOGICAL_PRESENTATION_LETTERBOX);

    bool running = true;
    SDL_Event evento;

    while (running) {
        if (freenect_process_events(context) < 0) break;

        while (SDL_PollEvent(&evento)) {
            if (evento.type == SDL_EVENT_QUIT)
                running = false;

            if (evento.type == SDL_EVENT_KEY_DOWN) {
                switch (evento.key.key) {
                case SDLK_UP:       maxDepth += 50;      break;
                case SDLK_DOWN:     maxDepth -= 50;      break;
                case SDLK_LEFT:     minDepth -= 50;      break;
                case SDLK_RIGHT:    minDepth += 50;      break;
                case SDLK_W: waterEnabled = !waterEnabled;        break;
                case SDLK_PAGEUP:   waterLevel += 20;    break;
                case SDLK_PAGEDOWN: waterLevel -= 20;    break;
                }
            }
        }

        process_frame();
        SDL_UpdateTexture(texture, nullptr, depthbuffer.data(), W * 3);
        SDL_RenderClear(renderer);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    freenect_stop_depth(device);
    freenect_close_device(device);
    freenect_shutdown(context);

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
