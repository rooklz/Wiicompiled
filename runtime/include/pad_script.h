#pragma once

#include <dolphin/pad.h>

#include <string>

// Scripted GameCube controller input for unattended runs ([input] pad_script). The file is a
// timeline: each line `<seconds> [token ...]` means "from that many seconds after the first PAD
// read, port 1 reports exactly these inputs" - tokens A B X Y Z L R START UP DOWN LEFT RIGHT and
// SX+ SX- SY+ SY- (main stick) CX+ CX- CY+ CY- (C-stick); a line with no tokens releases
// everything. One second after the last line the script ends and the real controllers are
// reported again. Lines that do not start with a number are comments.
namespace PadScript {

void Load(const std::string& path);

// Overrides `status` (port 1) while the script is active; returns false when it is not.
bool Apply(PADStatus& status);

// Advances the script's clock by one guest frame. Called once per VI retrace; the script
// timeline is interpreted against this counter (at 60 Hz), never against wall time.
void Tick();

// The status most recently applied by the script, for anything that needs to record what
// the game was told. Zeroed when no script is running.
const PADStatus& LastApplied();

// Frame at which a SCRIPT_GO rebase anchored the timeline; 0 until it happens.
uint64_t RebasedAtFrame();

// Called by OSReport interception on the game's "Scene Restart" prints. When
// MKW_PAD_ANCHOR_SCENE=N is set, the Nth restart rebases the script timeline
// exactly like a SCRIPT_GO, frame-locked to the scene rather than to any
// external observer.
void NoteSceneRestart();

// Called by the tracer the moment any grid-height transform first moves during a
// race start: that is GO, the only anchor whose offset cannot drift with loading
// times. Rebases the script so its configured GO frame (MKW_PAD_ANCHOR_GO) lands
// exactly now, and hands control from the live channel to the script.
void NoteGo();

} // namespace PadScript
