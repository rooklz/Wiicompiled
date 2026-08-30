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
#include <unordered_map>

#include "memory.h"
#include "pad_script.h"

namespace KartTrace {
namespace {

struct Candidate {
    uint32_t address;
    float lastX, lastY, lastZ;
    double travelled;      // total distance moved, to reject stationary scenery
    int implausible;       // strikes for jumps no kart could make
    int movingFrames;      // frames on which it actually moved
    int movedX, movedZ;    // frames on which each horizontal axis changed
    int seenFrames;
    // How often this object turns the way the scripted stick commands. Only the player's
    // kart does; every CPU sits at chance. This is what tells them apart - ranking on
    // distance alone locked onto a CPU, whose trace is useless for calibrating inputs.
    int steerAgree, steerTotal;
    int framesWithHeight;  // frames whose vertical component is off the floor
    float lastHeading;
    bool hasHeading;
};

bool g_enabled = false;
bool g_locked = false;             // true once a single candidate is chosen
uint32_t g_address = 0;
uint32_t g_pinnedAddress = 0;   // MKW_TRACE_ADDRESS: skip the scan and record this from g_beginAt

// History. The scan cannot lock before the kart moves, so a recording that begins with
// the countdown - the only kind that holds a standing start - cannot come from the scan
// alone. Instead, from g_beginAt every transform sitting near the grid is sampled every
// frame, and when the lock finally names one of them its samples are written out ahead of
// the live rows. The set is rebuilt a few times early on because the race scene may not
// have allocated the karts yet on the first pass. MKW_TRACE_GRID="x,z" enables it.
struct HistorySample {
    int frame;
    float x, y, z;
    float m[9];
    int8_t stickX, stickY;
    uint16_t button;
    uint8_t triggerL, triggerR;
};
std::unordered_map<uint32_t, std::vector<HistorySample>> g_history;
bool g_haveGrid = false;
float g_gridX = 0.0f, g_gridZ = 0.0f;
float g_gridRadius = 6000.0f;
int g_historyBuilds = 0;
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
// MEM1 is scanned from the start of the game's arena, not from the start of the address
// space. Below the arena is static data and BSS, which is where scratch transforms live -
// and a scratch transform is orthonormal, follows the player, and is rewritten for a
// different object about once a second, so it passes every test a real kart passes and
// then breaks the recording into fragments. The arena bounds are the ones the game itself
// reports at boot: MEM1 Arena 0x80394e00 - 0x817f0520.
constexpr Region kRegions[] = {
    {0x80394E00u, 0x817F0000u},   // MEM1 game arena
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

// A 3x4 transform: three orthonormal rotation rows, each followed by a translation
// component. This is how the console's matrix type is laid out, so rotation and position
// are one object rather than two things to find separately.
//
// Searching for this directly replaced finding the position first and hunting for a
// rotation near it. That worked only intermittently: the scan locks onto whichever copy
// of the position it happens to rank highest, and the rotation's distance from that copy
// differs between runs, so a search window that succeeded once found nothing the next
// time. A transform is a single, far more specific signature.
bool LooksLikeTransform(uint32_t base, float* out) {
    float m[12];
    for (int i = 0; i < 12; ++i) {
        if (!ReadFloat(base + i * 4, m[i])) {
            return false;
        }
    }
    // Rows 0,1,2 of the rotation are at 0-2, 4-6, 8-10; translation at 3, 7, 11.
    const int rows[3][3] = { {0, 1, 2}, {4, 5, 6}, {8, 9, 10} };
    for (const auto& row : rows) {
        const float a = m[row[0]], b = m[row[1]], c = m[row[2]];
        if (std::fabs(a) > 1.001f || std::fabs(b) > 1.001f || std::fabs(c) > 1.001f) {
            return false;
        }
        if (std::fabs(a * a + b * b + c * c - 1.0f) > 0.02f) {
            return false;
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
            float dot = 0;
            for (int k = 0; k < 3; ++k) {
                dot += m[rows[i][k]] * m[rows[j][k]];
            }
            if (std::fabs(dot) > 0.02f) {
                return false;
            }
        }
    }
    if (!PlausiblePosition(m[3], m[7], m[11])) {
        return false;
    }
    for (int i = 0; i < 12; ++i) {
        out[i] = m[i];
    }
    return true;
}

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
            float m[12];
            if (!LooksLikeTransform(address, m)) {
                continue;
            }
            const float x = m[3], y = m[7], z = m[11];
            const float px = SnapshotFloat(address + 12);
            const float py = SnapshotFloat(address + 28);
            const float pz = SnapshotFloat(address + 44);
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
            g_candidates.push_back(Candidate{address, x, y, z, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0f, false});
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
        if (!ReadFloat(c.address + 12, x) || !ReadFloat(c.address + 28, y)
            || !ReadFloat(c.address + 44, z)) {
            continue;
        }
        const double dx = x - c.lastX, dy = y - c.lastY, dz = z - c.lastZ;
        const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (step > kMaxStep) {
            if (++c.implausible > 2) {
                continue;   // teleports: a camera target, a spare copy, or unrelated data
            }
            // Do not credit the jump. Without this, things that teleport once accumulate
            // more "distance" than a kart covers in a whole lap and win the ranking.
            c.lastX = x; c.lastY = y; c.lastZ = z;
            ++c.seenFrames;
            kept.push_back(c);
            continue;
        }
        c.travelled += step;
        ++c.seenFrames;
        if (step > 1.0) {
            ++c.movingFrames;
        }
        if (std::fabs(dx) > 0.01) ++c.movedX;
        if (std::fabs(dz) > 0.01) ++c.movedZ;
        if (std::fabs(y) > 50.0f) ++c.framesWithHeight;

        // Correlate heading change against the commanded stick.
        const double horizontal = std::sqrt(dx * dx + dz * dz);
        if (horizontal > 1.0) {
            const float heading = std::atan2(static_cast<float>(dx), static_cast<float>(dz));
            if (c.hasHeading) {
                float turn = heading - c.lastHeading;
                while (turn > 3.14159265f) turn -= 6.28318531f;
                while (turn < -3.14159265f) turn += 6.28318531f;
                const int stick = PadScript::LastApplied().stickX;
                if (std::abs(stick) > 90 && std::fabs(turn) > 1e-4f) {
                    ++c.steerTotal;
                    if ((stick > 0) == (turn > 0)) {
                        ++c.steerAgree;
                    }
                }
            }
            c.lastHeading = heading;
            c.hasHeading = true;
        }
        c.lastX = x; c.lastY = y; c.lastZ = z;
        kept.push_back(c);
    }
    g_candidates.swap(kept);
}

// Rotation, found next to the position rather than by searching all of memory again.
//
// A kart's rotation and its position live in the same object, so once the position is
// known the rotation is a few hundred bytes away at most. Mario Kart Wii stores it as a
// quaternion, which is easy to recognise: four consecutive floats whose sum of squares
// stays at 1 while the values themselves change. Nothing else in a kart's neighbourhood
// looks like that for long.
//
// This is what closes the turn rate. Deriving it from displacement measures the direction
// the kart is travelling, which during a drift is not the direction it is facing - and
// that discrepancy is the drift, so it cannot be filtered away.
uint32_t g_rotationAddress = 0;

// Three consecutive unit vectors that are mutually perpendicular: a rotation matrix.
// Checked as well as the quaternion form because the two are equally likely and cost the
// same to test, and a window that found no quaternion may still contain a matrix.
bool LooksOrthonormal(uint32_t address) {
    float m[9];
    for (int i = 0; i < 9; ++i) {
        if (!ReadFloat(address + i * 4, m[i]) || std::fabs(m[i]) > 1.001f) {
            return false;
        }
    }
    for (int row = 0; row < 3; ++row) {
        const float* r = m + row * 3;
        const float length = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        if (std::fabs(length - 1.0f) > 0.02f) {
            return false;
        }
    }
    for (int a = 0; a < 3; ++a) {
        for (int b = a + 1; b < 3; ++b) {
            const float* ra = m + a * 3;
            const float* rb = m + b * 3;
            if (std::fabs(ra[0] * rb[0] + ra[1] * rb[1] + ra[2] * rb[2]) > 0.02f) {
                return false;
            }
        }
    }
    return true;
}

bool LooksNormalised(uint32_t address) {
    float q[4];
    for (int i = 0; i < 4; ++i) {
        // ReadFloat already rejects NaN and infinity; a candidate that passes once and
        // then goes NaN is why this is verified over time rather than at one instant.
        if (!ReadFloat(address + i * 4, q[i])) {
            return false;
        }
        if (std::fabs(q[i]) > 1.001f) {
            return false;
        }
    }
    const float norm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    return std::fabs(norm - 1.0f) < 0.01f;
}

struct RotationCandidate {
    uint32_t address;
    float last[4];
    int changed;      // checks on which any component moved
};
std::vector<RotationCandidate> g_rotationCandidates;
int g_rotationChecks = 0;

// Drop any candidate that has stopped being a unit quaternion. Called for a while after
// the position lock, because a single instant is not evidence: several unrelated quads
// near the object are momentarily normalised and one of them went NaN a frame later.
void VerifyRotation() {
    std::vector<RotationCandidate> kept;
    for (RotationCandidate& c : g_rotationCandidates) {
        if (!LooksNormalised(c.address) && !LooksOrthonormal(c.address)) {
            continue;
        }
        float q[4];
        bool moved = false;
        for (int i = 0; i < 4; ++i) {
            ReadFloat(c.address + i * 4, q[i]);
            if (std::fabs(q[i] - c.last[i]) > 1e-5f) {
                moved = true;
            }
            c.last[i] = q[i];
        }
        // A constant unit quaternion is not a rotation. Several sit near the object -
        // padding, an identity transform - and they pass a normality test perfectly.
        if (moved) {
            ++c.changed;
        }
        kept.push_back(c);
    }
    g_rotationCandidates.swap(kept);
    ++g_rotationChecks;
}

void FindRotation(uint32_t positionAddress) {
    // Candidates that are normalised right now; the caller confirms they stay that way.
    // Widened after a 512-byte window found no turning rotation: the position that the
    // scanner locks onto is often a copy held by a sub-object, so the transform it belongs
    // to can be some way off.
    constexpr int32_t kSearchBytes = 8192;
    std::vector<RotationCandidate> candidates;
    const uint32_t low = positionAddress > kSearchBytes ? positionAddress - kSearchBytes : 0;
    for (uint32_t a = low; a <= positionAddress + kSearchBytes; a += 4) {
        if (LooksNormalised(a) || LooksOrthonormal(a)) {
            RotationCandidate c{a, {0, 0, 0, 0}, 0};
            for (int i = 0; i < 4; ++i) {
                ReadFloat(a + i * 4, c.last[i]);
            }
            candidates.push_back(c);
        }
    }
    std::fprintf(stderr,
                 "[kart-trace] %zu rotation-shaped candidates within %d bytes of the position\n",
                 candidates.size(), kSearchBytes);
    g_rotationCandidates = candidates;
}

// Settle on a rotation once the candidates have been watched for a while.
void ChooseRotation(uint32_t positionAddress) {
    // Only ones that actually turned. A kart cornering changes its rotation on most
    // frames; padding does not change at all.
    std::vector<const RotationCandidate*> live;
    for (const RotationCandidate& c : g_rotationCandidates) {
        if (c.changed * 100 >= g_rotationChecks * 40) {
            live.push_back(&c);
        }
    }
    if (live.empty()) {
        std::fprintf(stderr,
                     "[kart-trace] no rotation survived: %zu stayed normalised, none turned\n",
                     g_rotationCandidates.size());
        return;
    }
    // Nearest the position: rotation and position are fields of the same object.
    const RotationCandidate* best = live.front();
    int32_t bestDistance = std::abs(static_cast<int32_t>(best->address) - static_cast<int32_t>(positionAddress));
    for (const RotationCandidate* c : live) {
        const int32_t distance = std::abs(static_cast<int32_t>(c->address) - static_cast<int32_t>(positionAddress));
        if (distance < bestDistance) {
            best = c;
            bestDistance = distance;
        }
    }
    g_rotationAddress = best->address;
    std::fprintf(stderr,
                 "[kart-trace] rotation at 0x%08X (offset %+d, turned on %d of %d checks, "
                 "%zu candidates turned at all)\n",
                 g_rotationAddress,
                 static_cast<int>(g_rotationAddress) - static_cast<int>(positionAddress),
                 best->changed, g_rotationChecks, live.size());
}

void BuildHistorySet() {
    int added = 0;
    // MEM1 only: every kart transform found so far has lived in the MEM1 arena, and MEM2
    // is three times the size to walk. This runs often enough that the walk has to be cheap.
    const Region& region = kRegions[0];
    {
        for (uint32_t address = region.begin; address + 48 < region.end; address += 4) {
            float m[12];
            if (!LooksLikeTransform(address, m)) {
                continue;
            }
            const float dx = m[3] - g_gridX, dz = m[11] - g_gridZ;
            if (dx * dx + dz * dz > g_gridRadius * g_gridRadius) {
                continue;
            }
            if (g_history.find(address) == g_history.end()) {
                g_history[address].reserve(1200);
                ++added;
            }
            if (g_history.size() >= 4000) {
                break;
            }
        }
    }
    if (added > 0) {
        std::fprintf(stderr, "[kart-trace] history set: %zu transforms near the grid (%d new) at frame %d\n",
                     g_history.size(), added, g_frame);
    }
}

void SampleHistory() {
    const PADStatus& pad = PadScript::LastApplied();
    for (auto& entry : g_history) {
        HistorySample sample;
        sample.frame = g_frame;
        if (!ReadFloat(entry.first + 12, sample.x) || !ReadFloat(entry.first + 28, sample.y)
            || !ReadFloat(entry.first + 44, sample.z)) {
            continue;
        }
        const int rotationFloats[9] = {0, 1, 2, 4, 5, 6, 8, 9, 10};
        for (int i = 0; i < 9; ++i) {
            ReadFloat(entry.first + rotationFloats[i] * 4, sample.m[i]);
        }
        sample.stickX = pad.stickX;
        sample.stickY = pad.stickY;
        sample.button = pad.button;
        sample.triggerL = pad.triggerL;
        sample.triggerR = pad.triggerR;
        entry.second.push_back(sample);
    }
}

void WriteHistoryFor(uint32_t address) {
    auto found = g_history.find(address);
    if (found == g_history.end()) {
        std::fprintf(stderr, "[kart-trace] history: 0x%08X was not in the grid set; the launch is not recorded\n",
                     address);
        g_history.clear();
        return;
    }
    if (g_out) {
        for (const HistorySample& h : found->second) {
            std::fprintf(g_out,
                         "%d,%.4f,%.4f,%.4f,"
                         "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                         "%d,%d,%u,%u,%u\n",
                         h.frame, h.x, h.y, h.z,
                         h.m[0], h.m[1], h.m[2], h.m[3], h.m[4], h.m[5], h.m[6], h.m[7], h.m[8],
                         static_cast<int>(h.stickX), static_cast<int>(h.stickY),
                         static_cast<unsigned>(h.button),
                         static_cast<unsigned>(h.triggerL),
                         static_cast<unsigned>(h.triggerR));
        }
    }
    std::fprintf(stderr, "[kart-trace] history: wrote %zu frames for 0x%08X ahead of the live trace\n",
                 found->second.size(), address);
    g_history.clear();
}

void OpenOutput() {
    const char* path = std::getenv("MKW_TRACE_OUT");
    g_out = std::fopen(path ? path : "/tmp/kart_trace.csv", "w");
    if (g_out) {
        // Nine rotation floats, not four. The candidate that survived verification was an
        // orthonormal matrix rather than a quaternion, and logging only the first four of
        // it recorded one row plus whatever followed - which was NaN.
        std::fprintf(g_out, "frame,x,y,z,"
                     "m0,m1,m2,m3,m4,m5,m6,m7,m8,"
                     "stickX,stickY,buttons,triggerL,triggerR\n");
    }
}

void Lock() {
    // A kart moves on essentially every frame of a race, and covers real ground doing it.
    // Ranking on distance alone picked things that jumped once and sat still, so require
    // sustained motion first and then take the furthest travelled.
    auto moves = [](const Candidate& c) {
        return c.seenFrames > 0 && c.movingFrames * 100 >= c.seenFrames * 80;
    };
    // A correctly aligned position has its vertical component in the middle and, on these
    // courses, much the smallest: they span tens of thousands of units horizontally and a
    // few thousand vertically. Scanning every four bytes also finds the same vector read
    // one field early, which looks equally kart-like but yields (z, x, y) - the first
    // successful lock did exactly that.
    // A kart turns, so both horizontal components change over a few seconds of racing.
    // A counter changes exactly one, which is how (0, 0, 300317) - a monotonically rising
    // value - beat the real karts on the first attempt: printed rounded it looked like a
    // position with a tiny vertical component.
    // The player's kart follows the scripted stick; a CPU is at chance. Requiring a clear
    // majority is what picks the player out of a field of twelve.
    auto followsStick = [](const Candidate& c) {
        return c.steerTotal >= 20 && c.steerAgree * 100 >= c.steerTotal * 75;
    };
    auto wellAligned = [](const Candidate& c) {
        const float ax = std::fabs(c.lastX), ay = std::fabs(c.lastY), az = std::fabs(c.lastZ);
        if (!(ay < ax && ay < az)) {
            return false;
        }
        if (!(c.movedX * 100 >= c.seenFrames * 40 && c.movedZ * 100 >= c.seenFrames * 40)) {
            return false;
        }
        // A position sits on the course, hundreds of units above the world floor. A
        // velocity or a facing direction is horizontal, so its vertical component hovers
        // around zero - and it correlates with the stick just as strongly as a position
        // does, so the steering test alone cannot tell them apart. The first run to
        // identify the player locked onto its velocity for exactly this reason.
        return c.framesWithHeight * 100 >= c.seenFrames * 70;
    };
    const Candidate* best = nullptr;
    for (const Candidate& c : g_candidates) {
        if (!moves(c) || !wellAligned(c) || !followsStick(c)) continue;
        if (!best || c.travelled > best->travelled) best = &c;
    }
    if (best) {
        std::fprintf(stderr, "[kart-trace] player kart: %d of %d steered frames agree (%.0f%%)\n",
                     best->steerAgree, best->steerTotal,
                     100.0 * best->steerAgree / std::max(1, best->steerTotal));
    }
    if (!best) {
        // Deliberately no fallback. Ranking on distance instead locks onto whatever moved
        // furthest, which is some other kart or object, and the resulting trace looks
        // entirely healthy - long, continuous, plausible speeds - while describing the
        // wrong thing. A run that produced 49% agreement with the stick had been recorded
        // exactly that way. Better to record nothing and retry than to record a lie.
        std::fprintf(stderr,
                     "[kart-trace] no candidate follows the stick (%zu survivors); retrying\n",
                     g_candidates.size());
        return;
    }
    if (!best) {
        // Fall back to alignment-agnostic ranking rather than finding nothing, and say so.
        for (const Candidate& c : g_candidates) {
            if (!moves(c)) continue;
            if (!best || c.travelled > best->travelled) best = &c;
        }
        if (best) {
            std::fprintf(stderr, "[kart-trace] no well-aligned candidate; using 0x%08X\n",
                         best->address);
        }
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
                     "[kart-trace]   0x%08X travelled %.0f over %d/%d moving, steer %d/%d, "
                     "at (%.0f, %.0f, %.0f)\n",
                     c->address, c->travelled, c->movingFrames, c->seenFrames,
                     c->steerAgree, c->steerTotal, c->lastX, c->lastY, c->lastZ);
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

    g_rotationAddress = g_address;   // rotation and translation are one transform
    OpenOutput();
    if (g_haveGrid) {
        WriteHistoryFor(g_address);
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
    // A known address skips the differential scan entirely. The scan needs the kart to be
    // moving, so it cannot lock before the race starts and a trace that begins with the
    // countdown - the only way to record a standing start - has to be told where to look.
    if (const char* pin = std::getenv("MKW_TRACE_ADDRESS")) {
        g_pinnedAddress = static_cast<uint32_t>(std::strtoul(pin, nullptr, 0));
    }
    if (const char* grid = std::getenv("MKW_TRACE_GRID")) {
        if (std::sscanf(grid, "%f,%f", &g_gridX, &g_gridZ) == 2) {
            g_haveGrid = true;
        }
    }
    if (const char* radius = std::getenv("MKW_TRACE_GRID_RADIUS")) {
        g_gridRadius = static_cast<float>(std::atof(radius));
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
        if (g_out && ReadFloat(g_address + 12, x) && ReadFloat(g_address + 28, y)
            && ReadFloat(g_address + 44, z)) {
            // Inputs alongside positions: without them a trace says what the kart did but
            // not what it was told to do, which is enough to measure top speed and turn
            // rate and not enough to calibrate anything that depends on the stick.
            float m[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            if (g_rotationAddress != 0) {
                const int rotationFloats[9] = {0, 1, 2, 4, 5, 6, 8, 9, 10};
                for (int i = 0; i < 9; ++i) {
                    ReadFloat(g_rotationAddress + rotationFloats[i] * 4, m[i]);
                }
            }
            const PADStatus& pad = PadScript::LastApplied();
            std::fprintf(g_out,
                         "%d,%.4f,%.4f,%.4f,"
                         "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                         "%d,%d,%u,%u,%u\n",
                         g_frame, x, y, z,
                         m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8],
                         static_cast<int>(pad.stickX), static_cast<int>(pad.stickY),
                         static_cast<unsigned>(pad.button),
                         static_cast<unsigned>(pad.triggerL),
                         static_cast<unsigned>(pad.triggerR));
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
    if (!g_locked && g_haveGrid) {
        const int sinceBegin = g_frame - g_beginAt;
        // Rebuild every half second until the lock: the karts are allocated when the race
        // scene loads, which is later than the first builds, and a launch missed by a
        // hundred frames is a launch missed. A MEM1 walk every thirty frames is affordable.
        if ((sinceBegin % 30) == 0) {
            BuildHistorySet();
            ++g_historyBuilds;
        }
        SampleHistory();
    }
    if (!g_locked && g_pinnedAddress != 0) {
        g_address = g_pinnedAddress;
        g_rotationAddress = g_pinnedAddress;
        g_locked = true;
        OpenOutput();
        std::fprintf(stderr, "[kart-trace] pinned to 0x%08X from frame %d\n", g_address, g_frame);
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
