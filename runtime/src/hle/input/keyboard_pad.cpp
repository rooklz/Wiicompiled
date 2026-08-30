#include "keyboard_pad.h"

#include "runtime_config.h"
#include "runtime_log.h"

#include <SDL3/SDL_scancode.h>
#include <dolphin/pad.h>

namespace KeyboardPad {

void ApplyOnce() {
    static bool applied = false;
    if (applied) {
        return;
    }
    applied = true;
    const int32_t port = RuntimeConfigFile::KeyboardPort(-1);
    if (port < 0) {
        return;
    }
    if (port >= PAD_MAX_CONTROLLERS) {
        RT_LOG(RT_TAG_RUNTIME) << "[input] keyboard_port = " << port << " is not a controller port (0-3)" << std::endl;
        return;
    }
    PADKeyButtonBinding buttons[PAD_BUTTON_COUNT] = {
        {SDL_SCANCODE_X, PAD_BUTTON_A},         {SDL_SCANCODE_Z, PAD_BUTTON_B},
        {SDL_SCANCODE_C, PAD_BUTTON_X},         {SDL_SCANCODE_V, PAD_BUTTON_Y},
        {SDL_SCANCODE_RETURN, PAD_BUTTON_START}, {SDL_SCANCODE_LSHIFT, PAD_TRIGGER_Z},
        {SDL_SCANCODE_Q, PAD_TRIGGER_L},        {SDL_SCANCODE_E, PAD_TRIGGER_R},
        {SDL_SCANCODE_UP, PAD_BUTTON_UP},       {SDL_SCANCODE_DOWN, PAD_BUTTON_DOWN},
        {SDL_SCANCODE_LEFT, PAD_BUTTON_LEFT},   {SDL_SCANCODE_RIGHT, PAD_BUTTON_RIGHT},
    };
    PADKeyAxisBinding axes[PAD_AXIS_COUNT] = {
        {SDL_SCANCODE_D, PAD_AXIS_LEFT_X_POS, 0},  {SDL_SCANCODE_A, PAD_AXIS_LEFT_X_NEG, 0},
        {SDL_SCANCODE_W, PAD_AXIS_LEFT_Y_POS, 0},  {SDL_SCANCODE_S, PAD_AXIS_LEFT_Y_NEG, 0},
        {SDL_SCANCODE_L, PAD_AXIS_RIGHT_X_POS, 0}, {SDL_SCANCODE_J, PAD_AXIS_RIGHT_X_NEG, 0},
        {SDL_SCANCODE_I, PAD_AXIS_RIGHT_Y_POS, 0}, {SDL_SCANCODE_K, PAD_AXIS_RIGHT_Y_NEG, 0},
        {SDL_SCANCODE_Q, PAD_AXIS_TRIGGER_L, 0},   {SDL_SCANCODE_E, PAD_AXIS_TRIGGER_R, 0},
    };
    const auto uport = static_cast<uint32_t>(port);
    if (!PADSetKeyButtonBindings(uport, buttons) || !PADSetKeyAxisBindings(uport, axes)) {
        RT_LOG(RT_TAG_RUNTIME) << "keyboard port " << port << ": aurora rejected the key bindings" << std::endl;
        return;
    }
    PADSetKeyboardActive(uport, TRUE);
    RT_LOG(RT_TAG_RUNTIME) << "keyboard drives controller port " << port + 1
                           << " (WASD stick, arrows D-pad, IJKL C-stick, X/Z/C/V = A/B/X/Y, Q/E = L/R, "
                              "Shift = Z, Enter = Start)"
                           << std::endl;
}

} // namespace KeyboardPad
