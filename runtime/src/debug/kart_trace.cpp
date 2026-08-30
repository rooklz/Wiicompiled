// Kart state tracing, for calibrating a reimplementation against this build.
//
// The Luau port in ../../mkw-roblox reproduces Mario Kart Wii's handling model but has no
// way to know whether its numbers are right. This turns that into a measurement: record
// what the real game does frame by frame, replay the same inputs through the port, and
// compare. Without it, "does the drift feel right" is a matter of opinion.
//
// Finding the kart's position in memory is done by narrowing rather than by knowing the
// object layout, and by motion rather than by value. Seeding the search on the course's
// start point does not work: the course's own collision and model vertices cluster around
// the start line in their thousands and match any position test, while never moving.
//
// So the search is differential. Two snapshots of guest memory a frame apart are compared,
// and every place where three consecutive floats moved by a kart-like amount becomes a
// candidate. Static geometry is excluded by construction, because it does not move.
// Candidates are then tracked over several seconds and the one covering the most ground
// on a continuous path is the kart.
//
// Off unless MKW_TRACE is set, and costs nothing when off.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "memory.h"

namespace KartTrace {
namespace {

struct Candidate {
    uint32_t address;
    float lastX, lastY, lastZ;
    double travelled;      // total distance moved, to reject stationary scenery
    int implausible;       // strikes for jumps no kart could make
    int movingFrames;      // frames on which it actually moved
    int seenFrames;
};

bool g_enabled = false;
bool g_locked = false;             // true once a single candidate is chosen
uint32_t g_address = 0;
std::vector<Candidate> g_candidates;
std::FILE* g_out = nullptr;
int g_frame = 0;
int g_narrowFrames = 0;
int g_narrowUntil = 0;
int g_attempts = 0;
int g_beginAt = 600;
int g_pendingDiff = 0;

float g_startX = 0.0f, g_startY = 0.0f, g_startZ = 0.0f;

// Scanned regions. Both arenas: MEM1 holds the game heap, but MKW places plenty of scene
// state in MEM2, and restricting the search to MEM1 found almost nothing.
struct Region { uint32_t begin, end; };
constexpr Region kRegions[] = {
    {0x80000000u, 0x81800000u},   // MEM1
    {0x90000000u, 0x94000000u},   // MEM2
};

// A kart at top speed covers roughly 150 units per frame. The upper bound rejects values
// that are not positions; the lower one is deliberately loose, because a kart at the start
// line barely moves and the sustained-motion test does the real filtering.
constexpr float kMaxStep = 400.0f;
constexpr float kMinStep = 0.5f;

bool ReadFloat(uint32_t address, float& out) {
    if (!Memory::Contains(address, 4)) {
        return false;
    }
    out = Memory::ReadFloat32(address);
    return std::isfinite(out);
}

// A world position, loosely. Rejects counters, normals, colours and timers, which also
// move but are not somewhere a kart could be.
bool PlausiblePosition(float x, float y, float z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
    if (std::fabs(x) > 500000.0f || std::fabs(z) > 500000.0f) return false;
    if (y < -50000.0f || y > 100000.0f) return false;
    // Three equal components are a counter written three times, or a uniform scale - not
    // a place. The first version of this filter locked onto (28, 28, 28).
    if (x == y && y == z) return false;
    // Courses span tens of thousands of units, so a real position is far from the origin
    // on at least one horizontal axis. This rejects directions, colours and small counters.
    return std::fabs(x) > 1000.0f || std::fabs(z) > 1000.0f;
}

std::vector<uint8_t> g_snapshot;

std::vector<uint8_t> g_snapshot2;

void DetectSnapshotEndian();   // defined below; TakeSnapshot calls it once

void TakeSnapshot() {
    // One memcpy off the host view rather than millions of Read32 calls: at 24 MB a word
    // at a time the snapshot alone cost more than the frame it was taken in.
    for (size_t i = 0; i < 2; ++i) {
        const Region& region = kRegions[i];
        const size_t bytes = region.end - region.begin;
        std::vector<uint8_t>& into = (i == 0) ? g_snapshot : g_snapshot2;
        into.resize(bytes);
        const uint8_t* base = Memory::GetPointer(region.begin, bytes);
        if (base) {
            std::memcpy(into.data(), base, bytes);
        } else {
            into.clear();
        }
        if (g_attempts <= 1) {
            std::fprintf(stderr, "[kart-trace] snapshot 0x%08X..0x%08X: %s (%zu bytes)\n",
                         region.begin, region.end, base ? "ok" : "GetPointer refused",
                         into.size());
        }
    }
    static bool detected = false;
    if (!detected && !g_snapshot.empty()) {
        detected = true;
        DetectSnapshotEndian();
    }
}

// Whether the raw snapshot bytes need swapping to match the accessors. Determined by
// comparison rather than assumed: getting this wrong turns every previous position into
// noise, every candidate fails the plausibility test, and the scan silently finds nothing.
bool g_snapshotNeedsSwap = true;

float SnapshotFloat(uint32_t address) {
    const std::vector<uint8_t>& from =
        (address < kRegions[0].end) ? g_snapshot : g_snapshot2;
    const uint32_t base = (address < kRegions[0].end) ? kRegions[0].begin : kRegions[1].begin;
    uint32_t word;
    std::memcpy(&word, &from[address - base], 4);
    if (g_snapshotNeedsSwap) {
        word = __builtin_bswap32(word);
    }
    float value;
    std::memcpy(&value, &word, 4);
    return value;
}

// Compare both interpretations of the freshly taken snapshot against the live accessor,
// which is the authority, and keep whichever agrees.
void DetectSnapshotEndian() {
    int swapped = 0, direct = 0;
    for (uint32_t address = kRegions[0].begin + 0x100000;
         address < kRegions[0].begin + 0x300000; address += 4096) {
        if (!Memory::Contains(address, 4)) continue;
        const float live = Memory::ReadFloat32(address);
        if (!std::isfinite(live) || live == 0.0f) continue;
        g_snapshotNeedsSwap = true;
        if (SnapshotFloat(address) == live) ++swapped;
        g_snapshotNeedsSwap = false;
        if (SnapshotFloat(address) == live) ++direct;
    }
    g_snapshotNeedsSwap = swapped >= direct;
    std::fprintf(stderr, "[kart-trace] snapshot byte order: %s (%d swapped vs %d direct)\n",
                 g_snapshotNeedsSwap ? "swapped" : "direct", swapped, direct);
}

int g_statPlausible = 0;
int g_statMoved = 0;

void ScanForMotion() {
    g_candidates.clear();
    g_statPlausible = 0;
    g_statMoved = 0;
    if (g_snapshot.empty()) {
        return;
    }
    for (const Region& region : kRegions) {
        const std::vector<uint8_t>& snap =
            (region.begin == kRegions[0].begin) ? g_snapshot : g_snapshot2;
        if (snap.empty()) {
            continue;
        }
        for (uint32_t address = region.begin; address + 12 < region.end; address += 4) {
            float x, y, z;
            if (!ReadFloat(address, x) || !ReadFloat(address + 4, y)
                || !ReadFloat(address + 8, z)) {
                continue;
            }
            if (!PlausiblePosition(x, y, z)) {
                continue;
            }
            const float px = SnapshotFloat(address);
            const float py = SnapshotFloat(address + 4);
            const float pz = SnapshotFloat(address + 8);
            if (!PlausiblePosition(px, py, pz)) {
                continue;
            }
            ++g_statPlausible;
            const double dx = x - px, dy = y - py, dz = z - pz;
            const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (step > 0.0) {
                ++g_statMoved;
            }
            if (step < kMinStep || step > kMaxStep) {
                continue;
            }
            g_candidates.push_back(Candidate{address, x, y, z, 0.0, 0, 0, 0});
            if (g_candidates.size() >= 60000) {
                break;
            }
        }
    }
    std::fprintf(stderr, "[kart-trace] %zu moving float triples\n", g_candidates.size());
    for (size_t i = 0; i < g_candidates.size() && i < 6; ++i) {
        const Candidate& c = g_candidates[i];
        std::fprintf(stderr, "[kart-trace]   0x%08X now (%.1f, %.1f, %.1f)\n",
                     c.address, c.lastX, c.lastY, c.lastZ);
    }
    // Also report how many triples were positions at all, to tell "the filter is wrong"
    // apart from "nothing is moving".
    std::fprintf(stderr, "[kart-trace]   %d plausible positions seen, %d moved at all\n",
                 g_statPlausible, g_statMoved);
}

void Narrow() {
    std::vector<Candidate> kept;
    kept.reserve(g_candidates.size());
    for (Candidate& c : g_candidates) {
        float x, y, z;
        if (!ReadFloat(c.address, x) || !ReadFloat(c.address + 4, y) || !ReadFloat(c.address + 8, z)) {
            continue;
        }
        const double dx = x - c.lastX, dy = y - c.lastY, dz = z - c.lastZ;
        const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (step > kMaxStep) {
            if (++c.implausible > 2) {
                continue;   // teleports: a camera target, a spare copy, or unrelated data
            }
        }
        c.travelled += step;
        ++c.seenFrames;
        if (step > 1.0) {
            ++c.movingFrames;
        }
        c.lastX = x; c.lastY = y; c.lastZ = z;
        kept.push_back(c);
    }
    g_candidates.swap(kept);
}

void Lock() {
    // A kart moves on essentially every frame of a race, and covers real ground doing it.
    // Ranking on distance alone picked things that jumped once and sat still, so require
    // sustained motion first and then take the furthest travelled.
    auto moves = [](const Candidate& c) {
        return c.seenFrames > 0 && c.movingFrames * 100 >= c.seenFrames * 80;
    };
    const Candidate* best = nullptr;
    for (const Candidate& c : g_candidates) {
        if (!moves(c)) continue;
        if (!best || c.travelled > best->travelled) best = &c;
    }
    // Report the strongest few whether or not one qualifies: when nothing does, this is
    // the only way to see whether the search is looking at the right kind of thing.
    std::vector<const Candidate*> ranked;
    for (const Candidate& c : g_candidates) ranked.push_back(&c);
    std::sort(ranked.begin(), ranked.end(),
              [](const Candidate* a, const Candidate* b) { return a->travelled > b->travelled; });
    for (size_t i = 0; i < ranked.size() && i < 5; ++i) {
        const Candidate* c = ranked[i];
        std::fprintf(stderr,
                     "[kart-trace]   0x%08X travelled %.0f over %d/%d moving, at (%.0f, %.0f, %.0f)\n",
                     c->address, c->travelled, c->movingFrames, c->seenFrames,
                     c->lastX, c->lastY, c->lastZ);
    }

    if (!best || best->travelled < 2000.0) {
        std::fprintf(stderr, "[kart-trace] no candidate moved; still %zu after %d frames\n",
                     g_candidates.size(), g_frame);
        return;
    }
    g_address = best->address;
    g_locked = true;
    std::fprintf(stderr,
                 "[kart-trace] locked on 0x%08X: %.0f units over %d/%d moving frames "
                 "(%zu survivors), now at (%.0f, %.0f, %.0f)\n",
                 g_address, best->travelled, best->movingFrames, best->seenFrames,
                 g_candidates.size(), best->lastX, best->lastY, best->lastZ);

    const char* path = std::getenv("MKW_TRACE_OUT");
    g_out = std::fopen(path ? path : "/tmp/kart_trace.csv", "w");
    if (g_out) {
        std::fprintf(g_out, "frame,x,y,z\n");
    }
}

}  // namespace

void Initialize() {
    if (!std::getenv("MKW_TRACE")) {
        return;
    }
    if (const char* frames = std::getenv("MKW_TRACE_NARROW")) {
        g_narrowFrames = std::atoi(frames);
    }
    if (g_narrowFrames <= 0) {
        g_narrowFrames = 240;
    }
    if (const char* wait = std::getenv("MKW_TRACE_AFTER")) {
        g_beginAt = std::atoi(wait);
    }
    g_enabled = true;
    std::fprintf(stderr,
                 "[kart-trace] armed: differential scan from frame %d, narrowing over %d frames\n",
                 g_beginAt, g_narrowFrames);
}

void Tick() {
    if (!g_enabled) {
        return;
    }
    ++g_frame;

    if (g_locked) {
        float x, y, z;
        if (g_out && ReadFloat(g_address, x) && ReadFloat(g_address + 4, y)
            && ReadFloat(g_address + 8, z)) {
            std::fprintf(g_out, "%d,%.4f,%.4f,%.4f\n", g_frame, x, y, z);
            if ((g_frame % 300) == 0) {
                std::fflush(g_out);
            }
        }
        return;
    }

    // Wait for the game to be racing before looking: nothing useful moves on a menu.
    if (g_frame < g_beginAt) {
        return;
    }

    // Order matters here. The diff has to be checked before the "no candidates yet"
    // branch, or the scanner snapshots every frame and never gets round to comparing.
    if (g_pendingDiff != 0) {
        // Waiting for the second half of the diff. Without this the scanner falls through
        // to "no candidates yet" on the intervening frame and snapshots again, so the two
        // halves are never a frame apart and nothing is ever compared.
        if (g_frame < g_pendingDiff) {
            return;
        }
        ScanForMotion();
        g_pendingDiff = 0;
        return;
    }

    if (!g_candidates.empty() && g_frame < g_narrowUntil) {
        Narrow();
        return;
    }

    if (!g_candidates.empty()) {
        Lock();
        if (g_locked) {
            return;
        }
    }

    TakeSnapshot();
    g_pendingDiff = g_frame + 2;
    g_narrowUntil = g_frame + g_narrowFrames;
    if (++g_attempts > 8) {
        std::fprintf(stderr, "[kart-trace] giving up after %d attempts\n", g_attempts);
        g_enabled = false;
    }
}

void Shutdown() {
    if (g_out) {
        std::fclose(g_out);
        g_out = nullptr;
        std::fprintf(stderr, "[kart-trace] wrote %d frames\n", g_frame);
    }
}

}  // namespace KartTrace
