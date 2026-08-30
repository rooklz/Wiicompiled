#pragma once

// Keyboard as a GameCube controller ([input] keyboard_port = 0..3). aurora's PAD layer already
// resolves keyboard bindings per port (PADSetKeyButtonBindings / PADSetKeyAxisBindings /
// PADSetKeyboardActive) and applies them in PADRead; this only installs a fixed layout on the
// configured port: WASD = main stick, arrows = D-pad, IJKL = C-stick, X = A, Z = B, C = X,
// V = Y, Q = L, E = R, Shift = Z, Enter = Start.
namespace KeyboardPad {

// Installs the layout once, the first time it is called after PADRead has loaded any saved
// bindings (so the configured port always gets this layout). Does nothing when unconfigured.
void ApplyOnce();

} // namespace KeyboardPad
