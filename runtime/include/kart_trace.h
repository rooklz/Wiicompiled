#pragma once

// Kart state tracing. See src/debug/kart_trace.cpp for what this is for: it records what
// the real game does frame by frame so a reimplementation can be calibrated against it
// rather than tuned by eye. Inert unless MKW_TRACE_START is set.
namespace KartTrace {

void Initialize();
void Tick();      // once per frame, from the VI retrace path
void Shutdown();

}  // namespace KartTrace
