#include "pad_script.h"

#include "runtime_log.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

namespace PadScript {
namespace {

struct Step {
    uint64_t frame = 0;
    PADStatus status{};
};

std::vector<Step> g_steps;
bool g_started = false;
bool g_finished = false;
uint64_t g_frames = 0;
uint64_t g_startFrame = 0;

bool ParseToken(const std::string& token, PADStatus& status) {
    struct Button { const char* name; uint16_t bit; };
    static constexpr Button kButtons[] = {
        {"A", PAD_BUTTON_A},     {"B", PAD_BUTTON_B},       {"X", PAD_BUTTON_X},       {"Y", PAD_BUTTON_Y},
        {"Z", PAD_TRIGGER_Z},    {"L", PAD_TRIGGER_L},      {"R", PAD_TRIGGER_R},      {"START", PAD_BUTTON_START},
        {"UP", PAD_BUTTON_UP},   {"DOWN", PAD_BUTTON_DOWN}, {"LEFT", PAD_BUTTON_LEFT}, {"RIGHT", PAD_BUTTON_RIGHT},
    };
    for (const Button& button : kButtons) {
        if (token == button.name) {
            status.button |= button.bit;
            if (button.bit == PAD_TRIGGER_L) status.triggerL = 255;
            if (button.bit == PAD_TRIGGER_R) status.triggerR = 255;
            return true;
        }
    }
    struct Axis { const char* name; int8_t* field; int8_t value; };
    const Axis kAxes[] = {
        {"SX+", &status.stickX, 100},    {"SX-", &status.stickX, -100},
        {"SY+", &status.stickY, 100},    {"SY-", &status.stickY, -100},
        {"CX+", &status.substickX, 100}, {"CX-", &status.substickX, -100},
        {"CY+", &status.substickY, 100}, {"CY-", &status.substickY, -100},
    };
    for (const Axis& axis : kAxes) {
        if (token == axis.name) {
            *axis.field = axis.value;
            return true;
        }
    }
    return false;
}

// Live control file (MKW_PAD_LIVE): a plain file whose whole content is one pad-state
// line in script-token syntax ("A", "A SX-60", "DOWN", empty = neutral). Reread every
// few frames and held until it changes. This exists because no synthetic host keyboard
// event reliably reaches the input path, while PADStatus injection has driven every
// successful automated run; an external classifier loop can steer menus closed-loop
// by rewriting one small file.
const char* g_livePath = nullptr;
PADStatus g_liveStatus{};
bool g_liveActive = false;
uint64_t g_liveCheckedFrame = 0;
long g_liveMtime = 0;

void PollLive() {
    // Independent of Load(): the live channel must work with no script configured.
    static bool inited = false;
    if (!inited) {
        inited = true;
        if (const char* live = std::getenv("MKW_PAD_LIVE")) {
            static std::string livePath = live;
            g_livePath = livePath.c_str();
            RT_LOG(RT_TAG_RUNTIME) << "pad live-control file: " << livePath << std::endl;
        }
    }
    if (g_livePath == nullptr || g_frames - g_liveCheckedFrame < 6) {
        return;
    }
    g_liveCheckedFrame = g_frames;
    std::ifstream file(g_livePath);
    if (!file) {
        g_liveActive = false;
        return;
    }
    std::string line;
    std::getline(file, line);
    PADStatus status{};
    std::istringstream stream(line);
    std::string token;
    bool any = false;
    while (stream >> token) {
        if (ParseToken(token, status)) {
            any = true;
        }
    }
    g_liveStatus = status;
    g_liveStatus.err = PAD_ERR_NONE;
    // An empty or token-free file still counts as an active neutral pad, so a
    // "release everything" state is expressible.
    g_liveActive = true;
    (void)any;
}


} // namespace

void Load(const std::string& path) {
    g_steps.clear();
    g_started = g_finished = false;
    if (path.empty()) {
        return;
    }
    std::ifstream file(path);
    if (!file) {
        RT_LOG(RT_TAG_RUNTIME) << "pad script: cannot read " << path << std::endl;
        return;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream in(line);
        Step step;
        // Two spellings for a step's position: "F<frames>" is exact; a bare number is seconds,
        // interpreted at the guest's 60 Hz (8.0 -> frame 480).
        std::string when;
        if (!(in >> when)) {
            continue;
        }
        try {
            if (when.size() > 1 && when[0] == 'F') {
                step.frame = std::stoull(when.substr(1));
            } else {
                step.frame = static_cast<uint64_t>(std::llround(std::stod(when) * 60.0));
            }
        } catch (...) {
            continue;
        }
        std::string token;
        while (in >> token) {
            if (!ParseToken(token, step.status)) {
                RT_LOG(RT_TAG_RUNTIME) << "pad script: unknown token '" << token << "' in " << path << std::endl;
            }
        }
        g_steps.push_back(step);
    }
    RT_LOG(RT_TAG_RUNTIME) << "pad script: " << path << " (" << g_steps.size() << " steps)" << std::endl;
    if (const char* live = std::getenv("MKW_PAD_LIVE")) {
        static std::string livePath = live;
        g_livePath = livePath.c_str();
        RT_LOG(RT_TAG_RUNTIME) << "pad live-control file: " << livePath << std::endl;
    }
}

PADStatus g_lastApplied{};

const PADStatus& LastApplied() {
    return g_lastApplied;
}

// One VI retrace = one guest frame. Counting frames instead of wall time keeps a script
// aligned with the simulation whatever the host does: a laggy frame delays the timeline with
// the game, and an unpaced (frame_limit = false) run replays it fast without skewing a step.

void Tick() {
    ++g_frames;
    PollLive();
}

bool Apply(PADStatus& status) {
    // The live channel outranks the scripted timeline while its file exists.
    if (g_liveActive) {
        status = g_liveStatus;
        g_lastApplied = status;
        return true;
    }
    if (g_steps.empty() || g_finished) {
        return false;
    }
    if (!g_started) {
        g_started = true;
        g_startFrame = g_frames;
    }
    const uint64_t elapsed = g_frames - g_startFrame;
    const Step* current = nullptr;
    for (const Step& step : g_steps) {
        if (step.frame <= elapsed) {
            current = &step;
        }
    }
    if (current == nullptr) {
        return false;  // before the first step
    }
    if (current == &g_steps.back() && elapsed > current->frame + 60) {
        g_finished = true;
        return false;
    }
    status = current->status;
    status.err = PAD_ERR_NONE;
    g_lastApplied = status;
    return true;
}

} // namespace PadScript
