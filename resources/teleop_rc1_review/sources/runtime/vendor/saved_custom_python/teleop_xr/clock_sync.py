"""Per-WebSocket four-timestamp calibration for isolated mock input only.

The input epoch remains the browser's actual sampling epoch.  No fixed offset,
receive-time substitution, or relaxation of the existing freshness window is
performed.  Create a NEW ClockSession for each WebSocket / ownership session.
This clock check does not establish hardware or physical stopping safety.
"""

import math
import secrets
import time
from collections import OrderedDict
from dataclasses import dataclass


class _ClockError(ValueError):
    pass


@dataclass(frozen=True)
class _Pending:
    nonce: str
    server_nonce: str
    c1_wall_ms: float
    c1_mono_ms: float
    s2_wall_ms: float
    s2_mono_ms: float
    s3_wall_ms: float
    s3_mono_ms: float


@dataclass(frozen=True)
class _Sample:
    calibration_id: str
    offset_ms: float
    rtt_ms: float
    uncertainty_ms: float
    server_wall_ms: float
    server_mono_ms: float
    c4_wall_ms: float
    c4_mono_ms: float


class ClockSession:
    """Small, bounded calibration state; methods never propagate input errors.

    ``probe`` and ``ack`` accept the CONTENTS of the wire envelope's ``data``.
    Their replies are wire envelopes. ``normalize`` returns a new dictionary or
    None; ``reason`` holds its most recent failure reason. Clock callables use
    seconds, just like time.time / time.monotonic. All wire quantities use ms.
    """

    PENDING_TTL_MS = 2000.0
    SAMPLE_TTL_MS = 15000.0
    MAX_PENDING = 8
    MAX_SAMPLES = 64
    MAX_NONCE_HISTORY = 256
    CLIENT_CONTINUITY_MS = 10.0
    SERVER_CONTINUITY_MS = 50.0
    MAX_UNCERTAINTY_MS = 20.0
    MARGIN_MS = 2.0
    MIN_SAMPLES = 3
    OLDEST_AGE_MS = 250.0
    FUTURE_AGE_MS = -50.0

    def __init__(self, wall=time.time, mono=time.monotonic):
        self._wall = wall
        self._mono = mono
        self._host_anchor = None
        self._client_anchor = None
        self._last_frame_mono_ms = None
        self._pending = OrderedDict()
        self._nonces = OrderedDict()
        self._samples = []
        self.generation = secrets.token_hex(16)
        self.ready = False
        self.reason = "insufficient_samples"

    @staticmethod
    def _number(value):
        # bool and numeric-looking strings are not valid JSON timestamp numbers.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise _ClockError("invalid_timestamp")
        value = float(value)
        if not math.isfinite(value):
            raise _ClockError("nonfinite_timestamp")
        return value

    @staticmethod
    def _nonce(value):
        if not isinstance(value, str) or not value or len(value) > 128:
            raise _ClockError("invalid_nonce")
        return value

    def _invalidate(self, reason):
        self._pending.clear()
        self._samples.clear()
        self._client_anchor = None
        self._last_frame_mono_ms = None
        self.generation = secrets.token_hex(16)
        self.ready = False
        self.reason = reason

    def invalidate(self, reason="session_invalidated"):
        """Invalidate in-flight normalized states as well as pending probes."""
        self._invalidate(reason if isinstance(reason, str) else "session_invalidated")

    def _now(self):
        try:
            wall_ms = self._number(self._wall()) * 1000.0
            mono_ms = self._number(self._mono()) * 1000.0
        except Exception:
            self._invalidate("server_clock_invalid")
            raise _ClockError("server_clock_invalid") from None
        if not math.isfinite(wall_ms) or not math.isfinite(mono_ms):
            self._invalidate("server_clock_invalid")
            raise _ClockError("server_clock_invalid")
        if self._host_anchor is not None:
            old_wall, old_mono, last_mono = self._host_anchor
            elapsed = mono_ms - old_mono
            divergence = (wall_ms - old_wall) - elapsed
            if mono_ms < last_mono or abs(divergence) > self.SERVER_CONTINUITY_MS:
                # Establish only a fresh HOST anchor, not a fresh calibration.
                self._host_anchor = (wall_ms, mono_ms, mono_ms)
                self._invalidate("server_clock_discontinuity")
                raise _ClockError("server_clock_discontinuity")
            self._host_anchor = (old_wall, old_mono, mono_ms)
        else:
            self._host_anchor = (wall_ms, mono_ms, mono_ms)
        return wall_ms, mono_ms

    def _prune(self, now_mono_ms):
        expired = [key for key, item in self._pending.items()
                   if now_mono_ms - item.s2_mono_ms >= self.PENDING_TTL_MS]
        for key in expired:
            del self._pending[key]
        self._samples = [sample for sample in self._samples
                         if 0.0 <= now_mono_ms - sample.server_mono_ms
                         < self.SAMPLE_TTL_MS]
        # Nonces remain reserved during the sample window, with a hard bound.
        for nonce, seen in list(self._nonces.items()):
            if now_mono_ms - seen >= self.SAMPLE_TTL_MS:
                del self._nonces[nonce]

    def _check_client_anchor(self, wall_ms, mono_ms):
        if self._client_anchor is None:
            return
        old_wall, old_mono = self._client_anchor
        divergence = (wall_ms - old_wall) - (mono_ms - old_mono)
        if not math.isfinite(divergence) or abs(divergence) > self.CLIENT_CONTINUITY_MS:
            self._invalidate("client_clock_discontinuity")
            raise _ClockError("client_clock_discontinuity")

    def _select(self, now_mono_ms):
        had_samples = bool(self._samples)
        self._prune(now_mono_ms)
        if len(self._samples) < self.MIN_SAMPLES:
            self.ready = False
            self.reason = ("calibration_expired" if had_samples and not self._samples
                           else "insufficient_samples")
            return None
        # A real constant offset must lie in every exchange's error interval.
        lower = max(s.offset_ms - s.uncertainty_ms for s in self._samples)
        upper = min(s.offset_ms + s.uncertainty_ms for s in self._samples)
        if lower > upper:
            self._invalidate("inconsistent_offsets")
            return None
        selected = min(self._samples, key=lambda s: (s.rtt_ms, -s.server_mono_ms))
        if selected.uncertainty_ms > self.MAX_UNCERTAINTY_MS:
            self.ready = False
            self.reason = "excessive_uncertainty"
            return None
        self.ready = True
        self.reason = "ready"
        return selected

    def _diagnostic(self, now_mono_ms=None, selected=None):
        result = {"sample_count": len(self._samples),
                  "pending_count": len(self._pending),
                  "offset_ms": None, "rtt_ms": None,
                  "uncertainty_ms": None, "calibration_age_ms": None}
        if selected is not None and now_mono_ms is not None:
            result.update(offset_ms=selected.offset_ms, rtt_ms=selected.rtt_ms,
                          uncertainty_ms=selected.uncertainty_ms,
                          calibration_age_ms=now_mono_ms - selected.server_mono_ms)
        return result

    def _status(self, now_mono_ms=None, selected=None):
        return {"type": "openarm_clock_status", "data": {
            "ready": self.ready, "reason": self.reason,
            "diagnostic": self._diagnostic(now_mono_ms, selected)}}

    def probe(self, message):
        """Remember c1 and s2/s3; never trust client-supplied server times."""
        try:
            s2_wall, s2_mono = self._now()
            self._prune(s2_mono)
            if not isinstance(message, dict):
                raise _ClockError("invalid_probe")
            nonce = self._nonce(message.get("nonce"))
            c1_wall = self._number(message.get("c1_wall_ms"))
            c1_mono = self._number(message.get("c1_mono_ms"))
            if nonce in self._nonces:
                raise _ClockError("duplicate_nonce")
            if len(self._pending) >= self.MAX_PENDING:
                raise _ClockError("too_many_pending")
            self._check_client_anchor(c1_wall, c1_mono)
            if self._client_anchor is not None and c1_mono <= self._client_anchor[1]:
                raise _ClockError("nonindependent_probe")
            s3_wall, s3_mono = self._now()
            server_nonce = secrets.token_hex(16)
            item = _Pending(nonce, server_nonce, c1_wall, c1_mono,
                            s2_wall, s2_mono, s3_wall, s3_mono)
            self._pending[nonce] = item
            self._nonces[nonce] = s2_mono
            while len(self._nonces) > self.MAX_NONCE_HISTORY:
                self._nonces.popitem(last=False)
            return {"type": "openarm_clock_reply", "data": {
                "nonce": nonce, "server_nonce": server_nonce,
                "s2_wall_ms": s2_wall, "s3_wall_ms": s3_wall}}
        except Exception as exc:
            self.ready = False
            self.reason = str(exc) if isinstance(exc, _ClockError) else "invalid_probe"
            return {"type": "openarm_clock_reply", "data": {
                "ready": False, "reason": self.reason}}

    def ack(self, message):
        """Consume a matching pending exchange once and validate all clocks."""
        now_mono = None
        try:
            _, now_mono = self._now()
            if not isinstance(message, dict):
                raise _ClockError("invalid_ack")
            nonce = self._nonce(message.get("nonce"))
            server_nonce = self._nonce(message.get("server_nonce"))
            item = self._pending.get(nonce)
            if item is None:
                raise _ClockError("unknown_or_consumed_nonce")
            # Consume even a mismatched acknowledgement: never allow retry/replay.
            del self._pending[nonce]
            if not secrets.compare_digest(server_nonce, item.server_nonce):
                raise _ClockError("server_nonce_mismatch")
            if now_mono - item.s2_mono_ms >= self.PENDING_TTL_MS:
                raise _ClockError("pending_expired")
            c4_wall = self._number(message.get("c4_wall_ms"))
            c4_mono = self._number(message.get("c4_mono_ms"))
            client_elapsed = c4_wall - item.c1_wall_ms
            client_mono_elapsed = c4_mono - item.c1_mono_ms
            server_elapsed = item.s3_wall_ms - item.s2_wall_ms
            server_mono_elapsed = item.s3_mono_ms - item.s2_mono_ms
            values = (client_elapsed, client_mono_elapsed, server_elapsed,
                      server_mono_elapsed)
            if not all(math.isfinite(value) for value in values):
                raise _ClockError("nonfinite_delta")
            if (client_mono_elapsed < 0.0 or
                    abs(client_elapsed - client_mono_elapsed) > self.CLIENT_CONTINUITY_MS):
                self._invalidate("client_clock_discontinuity")
                raise _ClockError("client_clock_discontinuity")
            if (server_mono_elapsed < 0.0 or
                    abs(server_elapsed - server_mono_elapsed) > self.SERVER_CONTINUITY_MS):
                self._invalidate("server_clock_discontinuity")
                raise _ClockError("server_clock_discontinuity")
            self._check_client_anchor(item.c1_wall_ms, item.c1_mono_ms)
            self._check_client_anchor(c4_wall, c4_mono)
            if self._client_anchor is not None and item.c1_mono_ms <= self._client_anchor[1]:
                raise _ClockError("nonindependent_probe")
            rtt = client_elapsed - server_elapsed
            offset = ((item.s2_wall_ms - item.c1_wall_ms)
                      + (item.s3_wall_ms - c4_wall)) / 2.0
            uncertainty = rtt / 2.0 + self.MARGIN_MS
            if not all(math.isfinite(v) for v in (rtt, offset, uncertainty)):
                raise _ClockError("nonfinite_delta")
            if rtt < 0.0:
                raise _ClockError("negative_rtt")
            # Slow exchanges cannot count toward the three qualified samples.
            if uncertainty > self.MAX_UNCERTAINTY_MS:
                raise _ClockError("excessive_uncertainty")
            self._client_anchor = (c4_wall, c4_mono)
            self._samples.append(_Sample(secrets.token_hex(16), offset, rtt,
                                         uncertainty, item.s3_wall_ms, item.s3_mono_ms,
                                         c4_wall, c4_mono))
            self._samples = self._samples[-self.MAX_SAMPLES:]
            selected = self._select(now_mono)
            return self._status(now_mono, selected)
        except Exception as exc:
            self.ready = False
            self.reason = str(exc) if isinstance(exc, _ClockError) else "invalid_ack"
            return self._status(now_mono)

    def _check_freshness(self, raw_wall, raw_mono, mapped, sample, now_wall, now_mono):
        self._check_client_anchor(raw_wall, raw_mono)
        client_divergence = ((raw_wall - sample.c4_wall_ms)
                             - (raw_mono - sample.c4_mono_ms))
        host_divergence = ((now_wall - sample.server_wall_ms)
                           - (now_mono - sample.server_mono_ms))
        if not all(math.isfinite(v) for v in (client_divergence, host_divergence)):
            raise _ClockError("nonfinite_delta")
        if abs(client_divergence) > self.CLIENT_CONTINUITY_MS:
            self._invalidate("client_clock_discontinuity")
            raise _ClockError("client_clock_discontinuity")
        if abs(host_divergence) > self.SERVER_CONTINUITY_MS:
            self._invalidate("server_clock_discontinuity")
            raise _ClockError("server_clock_discontinuity")
        # Continuity tolerances are NOT free uncertainty. Even sub-threshold
        # wall steps must not make an actually old/future sample look fresh.
        effective_uncertainty = (sample.uncertainty_ms + abs(client_divergence)
                                 + abs(host_divergence))
        if effective_uncertainty > self.MAX_UNCERTAINTY_MS:
            self._invalidate("excessive_effective_uncertainty")
            raise _ClockError("excessive_effective_uncertainty")
        if mapped != raw_wall + sample.offset_ms:
            raise _ClockError("mapped_timestamp_mismatch")
        age = now_wall - mapped
        if not math.isfinite(mapped) or not math.isfinite(age):
            raise _ClockError("nonfinite_mapped_timestamp")
        # The whole uncertainty interval, not only its center, must fit.
        if age + effective_uncertainty > self.OLDEST_AGE_MS:
            raise _ClockError("stale_input")
        if age - effective_uncertainty < self.FUTURE_AGE_MS:
            raise _ClockError("future_input")
        return effective_uncertainty

    def still_valid(self, normalized_state):
        """Recheck before IK and immediately before publish; consume no sequence.

        Keep the ORIGINAL sample/offset/uncertainty. A new lower-RTT exchange
        must not silently remap an already-computing pose, or refresh its TTL.
        """
        try:
            now_wall, now_mono = self._now()
            if self._select(now_mono) is None:
                return False
            if not isinstance(normalized_state, dict):
                raise _ClockError("invalid_state")
            metadata = normalized_state.get("clock_sync")
            if not isinstance(metadata, dict) or metadata.get("generation") != self.generation:
                raise _ClockError("invalid_generation")
            sample = next((s for s in self._samples
                           if s.calibration_id == metadata.get("calibration_id")), None)
            if sample is None:
                raise _ClockError("calibration_expired")
            for key in ("offset_ms", "rtt_ms", "uncertainty_ms"):
                if self._number(metadata.get(key)) != getattr(sample, key):
                    raise _ClockError("calibration_metadata_mismatch")
            raw_wall = self._number(normalized_state.get("original_timestamp_unix_ms"))
            raw_mono = self._number(normalized_state.get("clock_monotonic_ms"))
            mapped = self._number(normalized_state.get("timestamp_unix_ms"))
            self._check_freshness(raw_wall, raw_mono, mapped, sample, now_wall, now_mono)
            self.reason = "ready"
            return True
        except Exception as exc:
            self.ready = False
            self.reason = str(exc) if isinstance(exc, _ClockError) else "invalid_state"
            return False

    def normalize(self, state):
        """Map only a proven original sample and conservatively check its age."""
        try:
            now_wall, now_mono = self._now()
            selected = self._select(now_mono)
            if selected is None:
                return None
            if not isinstance(state, dict):
                raise _ClockError("invalid_state")
            raw_wall = self._number(state.get("timestamp_unix_ms"))
            raw_mono = self._number(state.get("clock_monotonic_ms"))
            # Detect a reset performance clock before classifying a frame as
            # ordinary reordering; an actual clock reset clears calibration.
            self._check_client_anchor(raw_wall, raw_mono)
            if (self._last_frame_mono_ms is not None and
                    raw_mono <= self._last_frame_mono_ms):
                raise _ClockError("nonincreasing_input_monotonic")
            mapped = raw_wall + selected.offset_ms
            effective_uncertainty = self._check_freshness(
                raw_wall, raw_mono, mapped, selected, now_wall, now_mono)
            result = dict(state)
            result["original_timestamp_unix_ms"] = state["timestamp_unix_ms"]
            result["timestamp_unix_ms"] = mapped
            result["clock_sync"] = {
                "offset_ms": selected.offset_ms, "rtt_ms": selected.rtt_ms,
                "uncertainty_ms": selected.uncertainty_ms,
                "effective_uncertainty_ms": effective_uncertainty,
                "calibration_age_ms": now_mono - selected.server_mono_ms,
                "generation": self.generation,
                "calibration_id": selected.calibration_id}
            self._last_frame_mono_ms = raw_mono
            self.reason = "ready"
            return result
        except Exception as exc:
            self.ready = False
            self.reason = str(exc) if isinstance(exc, _ClockError) else "invalid_state"
            return None
