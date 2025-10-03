#include <libfreenect.h>
#include <SDL3/SDL.h>
#include <iostream>
#include <vector>
#include <mutex>

//variaveis
freenect_context* context;
freenect_device* device;
static std::vector<uint16_t> rawdepth(640 * 480, 0);
static std::vector<uint16_t> lastvalid(640 * 480, 0);
static std::vector<uint8_t> depthbuffer(640 * 480 * 3, 0); 
std::mutex depth_mutex;
int minDepth = 400; 
int maxDepth = 1200; 
int contourStep = 20;
int waterLevel = 700;
bool waterEnabled = false;

//normaliza mm pra rgb
void height_to_color(int h, uint8_t& r, uint8_t& g, uint8_t& b) {
    if (h < 64) { r = 0; g = h * 4; b = 255; }
    else if (h < 128) { r = 0; g = 255; b = 255 - (h - 64) * 4; }
    else if (h < 192) { r = (h - 128) * 4; g = 255; b = 0; }
    else { r = 255; g = 255 - (h - 192) * 4; b = 0; }
}

void profundidade(freenect_device* dev, void* v_depth, uint32_t timestamp) {
    std::lock_guard<std::mutex> lock(depth_mutex);
    uint16_t* d16 = (uint16_t*)v_depth;

    bool valid = false;
    for (int i = 0; i < 640 * 480; i++) {
        if (d16[i] > 0) { valid = true; break; }
    }

    if (valid) {
        std::copy(d16, d16 + 640 * 480, rawdepth.begin());
        lastvalid = rawdepth;
    }
    else {
        rawdepth = lastvalid;
    }
}

void process_frame() {
    std::lock_guard<std::mutex> lock(depth_mutex);

    for (int i = 0; i < 640 * 480; i++) {
        uint16_t v = rawdepth[i];
        uint8_t r = 0, g = 0, b = 0;

        if (v == 0 || v < minDepth || v > maxDepth) {
            r = g = b = 0;
        }
        else {
            int h = (v - minDepth) * 255 / (maxDepth - minDepth);
            if (h < 0) h = 0;
            if (h > 255) h = 255;

            height_to_color(h, r, g, b);

            if ((v % contourStep) < 2) {
                r = g = b = 0;
            }

            if (waterEnabled && v >= waterLevel) {
                r = 0; g = 0; b = 200;
            }

            if (i > 640 && i < (640 * 479)) {
                int dzdx = rawdepth[i - 1] - rawdepth[i + 1];
                int dzdy = rawdepth[i - 640] - rawdepth[i + 640];
                int shade = 128 + (dzdx + dzdy) / 16;
                if (shade < 50) shade = 50;
                if (shade > 255) shade = 255;
                r = (r * shade) / 255;
                g = (g * shade) / 255;
                b = (b * shade) / 255;
            }
        }

        depthbuffer[i * 3 + 0] = r;
        depthbuffer[i * 3 + 1] = g;
        depthbuffer[i * 3 + 2] = b;
    }
}

int main() {
    if (freenect_init(&context, NULL) < 0) return -1;
    freenect_select_subdevices(context, FREENECT_DEVICE_CAMERA);
    if (freenect_open_device(context, &device, 0) < 0) return -1;
    freenect_set_depth_callback(device, profundidade);
    freenect_set_depth_mode(device, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM));
    freenect_start_depth(device);

    if (SDL_Init(SDL_INIT_VIDEO) < 0)
        return -1;
    SDL_Window* window = SDL_CreateWindow("Sandbox", 640, 480, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    SDL_Texture* texture = SDL_CreateTexture(renderer,SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, 640, 480);

    bool running = true;
    SDL_Event evento;

    while (running) {
        if (freenect_process_events(context) < 0) break;

        while (SDL_PollEvent(&evento)) {
            if (evento.type == SDL_EVENT_QUIT) running = false;
            if (evento.type == SDL_EVENT_KEY_DOWN) {
                switch (evento.key.key) {
                case SDLK_UP:       maxDepth += 50; break;
                case SDLK_DOWN:     maxDepth -= 50; break;
                case SDLK_LEFT:     minDepth -= 50; break;
                case SDLK_RIGHT:    minDepth += 50; break;
                case SDLK_W:        waterEnabled = !waterEnabled; break;
                case SDLK_PAGEUP:   waterLevel += 20; break;
                case SDLK_PAGEDOWN: waterLevel -= 20; break;
                }
            }
        }

        process_frame();
        SDL_UpdateTexture(texture, NULL, depthbuffer.data(), 640 * 3);
        SDL_RenderClear(renderer);
        SDL_RenderTexture(renderer, texture, NULL, NULL);
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
