"""Position-bounded, C2-continuous joint trajectories without hardware access.

Only adjacent moving segments share velocity. Gaps, reversals, stationary
joints, landing frames and playback-cycle boundaries retain a stop. Times are
playback milliseconds (before the uniform global speed multiplier).
"""


class ContinuousTrajectory:
    def __init__(self, frames, initial_angles, time_map=None):
        self._time_map = time_map or (lambda value: float(value))
        self._segments = {}
        ordered = sorted(frames, key=lambda frame: frame.get("start_ms", 0))
        state = {int(joint): float(value) for joint, value in initial_angles.items()}
        segments = []
        for frame in ordered:
            targets = {int(j): float(v) for j, v in frame.get("angles", {}).items()}
            start = float(frame.get("start_ms", 0))
            end = start + max(1.0, float(frame.get("time_ms", 0)))
            duration = max(1e-9, self._time_map(end) - self._time_map(start))
            starts = {j: state.get(j, value) for j, value in targets.items()}
            # Keep the editor's established shortest-angle convention.
            deltas = {j: (value - starts[j] + 180.0) % 360.0 - 180.0
                      for j, value in targets.items()}
            segment = dict(
                frame=frame, start=start, end=end, duration=duration,
                starts=starts, targets=targets, deltas=deltas,
                slopes={j: delta / duration for j, delta in deltas.items()},
                incoming={j: 0.0 for j in targets},
                outgoing={j: 0.0 for j in targets},
            )
            segments.append(segment)
            self._segments[id(frame)] = segment
            state.update(targets)

        for left, right in zip(segments, segments[1:]):
            lf, rf = left['frame'], right['frame']
            if (left['end'] != right['start']
                    or lf.get('playback_cycle_end', False)
                    or lf.get('stop_at_end', False)
                    or '[착지]' in str(lf.get('name', ''))
                    or '[착지]' in str(rf.get('name', ''))):
                continue
            for joint in left['slopes'].keys() & right['slopes'].keys():
                before, after = left['slopes'][joint], right['slopes'][joint]
                if before * after <= 0.0:
                    continue
                # Do not blend across a wrapped encoder boundary, where the
                # editor must switch back to the saved absolute target.
                if any(abs(s['starts'][joint] + s['deltas'][joint]
                           - s['targets'][joint]) > 1e-8 for s in (left, right)):
                    continue
                velocity = min(abs(before), abs(after)) * (1 if before > 0 else -1)
                left['outgoing'][joint] = right['incoming'][joint] = velocity

        for segment in segments:
            coefficients = {}
            for joint, delta in segment['deltas'].items():
                m0 = segment['incoming'][joint] * segment['duration']
                m1 = segment['outgoing'][joint] * segment['duration']
                # Quintic Hermite curve, with zero acceleration at both ends.
                # Each endpoint speed <= the segment's average speed, so its
                # Bezier control points are ordered: no target overshoot.
                coefficients[joint] = (
                    segment['starts'][joint], m0,
                    10 * delta - 6 * m0 - 4 * m1,
                    -15 * delta + 8 * m0 + 7 * m1,
                    6 * delta - 3 * m0 - 3 * m1,
                )
            segment['coefficients'] = coefficients

    def sample_frame(self, frame, timeline_ms):
        segment = self._segments[id(frame)]
        if timeline_ms <= segment['start']:
            return dict(segment['starts'])
        if timeline_ms >= segment['end']:
            return dict(segment['targets'])
        u = (self._time_map(timeline_ms) - self._time_map(segment['start'])) / segment['duration']
        u = max(0.0, min(1.0, u))
        return {
            joint: q0 + m0 * u + u ** 3 * (c3 + u * (c4 + u * c5))
            for joint, (q0, m0, c3, c4, c5) in segment['coefficients'].items()
        }
