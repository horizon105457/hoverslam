// Easing curves for camera moves and motion graphics: f(0) = 0, f(1) = 1.
export const clamp01 = u => (u < 0 ? 0 : u > 1 ? 1 : u);
export const linear = u => clamp01(u);
export const smooth = u => { u = clamp01(u); return u * u * (3 - 2 * u); };
export const smoother = u => { u = clamp01(u); return u * u * u * (u * (u * 6 - 15) + 10); };
export const inCubic = u => clamp01(u) ** 3;
export const outCubic = u => 1 - (1 - clamp01(u)) ** 3;
export const inOutCubic = u => { u = clamp01(u); return u < 0.5 ? 4 * u * u * u : 1 - (-2 * u + 2) ** 3 / 2; };
export const outExpo = u => { u = clamp01(u); return u === 1 ? 1 : 1 - 2 ** (-10 * u); };
export const inOutExpo = u => { u = clamp01(u); return u === 0 || u === 1 ? u : u < 0.5 ? 2 ** (20 * u - 10) / 2 : (2 - 2 ** (-20 * u + 10)) / 2; };
/** overshoot then settle (snap-in of parts) */
export const outBack = (u, s = 1.70158) => { u = clamp01(u) - 1; return 1 + (s + 1) * u * u * u + s * u * u; };
/** remap u from [a, b] to [0, 1] (clamped) */
export const range = (u, a, b) => clamp01((u - a) / (b - a));
