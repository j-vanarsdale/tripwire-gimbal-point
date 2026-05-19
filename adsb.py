import time, os
import asyncio
import telnetlib3
from threading import Thread, Lock
import numpy as np
import json
from datetime import datetime, timezone
# SBS-1 BaseStation CSV field indices
#  0: MSG type prefix ("MSG")
#  1: Transmission type (1-8)
#  2: Session ID
#  3: Aircraft ID
#  4: ICAO hex address
#  5: Flight ID
#  6: Date generated
#  7: Time generated
#  8: Date logged
#  9: Time logged
# 10: Callsign
# 11: Altitude (ft)
# 12: Ground speed (kts)
# 13: Track (degrees)
# 14: Latitude
# 15: Longitude
# 16: Vertical rate (ft/min)
# 17: Squawk
# 18: Alert (squawk change)
# 19: Emergency
# 20: SPI (ident)
# 21: Is on ground

FIELD_MAP = {
    "callsign":      10,
    "altitude":      11,
    "speed":         12,
    "track":         13,
    "lat":           14,
    "lon":           15,
    "vertical_rate": 16,
    "squawk":        17,
    "alert":         18,
    "emergency":     19,
    "spi":           20,
    "on_ground":     21,
}

MAX_EXTRAPOLATION_S = 20.0
LEAD_TIME_S = 1.0
INTERVAL = 0.5

def _parse_field(value: str):
    """Strip whitespace and return None for empty strings, else best-fit type."""
    value = value.strip()
    if value == "":
        return None
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        pass
    return value


def parse_msg(raw: str) -> dict | None:
    """
    Parse one SBS-1 CSV line into a dict of non-None fields plus the ICAO address.
    Returns None if the line is not a valid MSG sentence.
    """
    parts = raw.strip().split(",")
    if len(parts) < 10 or parts[0] != "MSG":
        return None

    icao = parts[4].strip()
    if not icao:
        return None

    result = {"icao": icao}
    for field, idx in FIELD_MAP.items():
        if idx < len(parts):
            parsed = _parse_field(parts[idx])
            if parsed is not None:
                result[field] = parsed

    return result


def closest_track(tracks: "TrackList", lat: float, lon: float) -> "Track | None":
    """
    Return the Track with a known position closest to (lat, lon).

    Uses the Haversine formula to calculate great-circle distance so the
    result is accurate regardless of where on the globe you are.
    Returns None if no tracks with a position are present.

    Parameters
    ----------
    tracks : TrackList
        The live track list to search.
    lat    : float
        Observer latitude in decimal degrees.
    lon    : float
        Observer longitude in decimal degrees.
    """
    import math

    def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Great-circle distance in kilometres between two points."""
        R = 6371.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi       = math.radians(lat2 - lat1)
        dlambda    = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    snapshot = tracks.get_all()
    best_track    = None
    best_distance = float("inf")

    for track in snapshot.values():
        if not track.has_position():
            continue
        distance = _haversine(lat, lon, track.lat, track.lon)
        if distance < best_distance:
            best_distance = distance
            best_track    = track

    return best_track

def _kf_init(lat, lon, alt_ft):

    x = np.array([lat, lon, alt_ft, 0.0, 0.0, 0.0], dtype=float)
    P = np.diag([1e-6, 1e-6, 100.0**2, 1e-4, 1e-4, 10.0**2])
    return x, P


def _kf_predict(x, P, dt):
    if dt <= 0.0:
        return x, P

    # State transition matrix F(dt)
    F = np.array([
        [1.0, 0.0, 0.0, dt,  0.0,  0.0],
        [0.0, 1.0, 0.0, 0.0, dt,   0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0,  dt ],
        [0.0, 0.0, 0.0, 1.0, 0.0,  0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0,  0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0,  1.0],
    ], dtype=float)

    q_pos = 1e-10   # deg^2 / s for lat/lon
    q_alt = 1.0     # ft^2 / s for altitude
    q_vel = 1e-6    # (deg/s)^2 / s
    q_v_alt = 1.0   # (ft/s)^2 / s

    Q = np.diag([
        q_pos * dt,
        q_pos * dt,
        q_alt * dt,
        q_vel * dt,
        q_vel * dt,
        q_v_alt * dt
    ])

    x = F @ x
    P = F @ P @ F.T + Q
    return x, P


def _kf_update(x, P, lat_meas, lon_meas, alt_ft_meas):
    """
    Measurement update with z = [lat, lon, alt_ft].
    """
    z = np.array([lat_meas, lon_meas, alt_ft_meas], dtype=float)

    H = np.array([
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    ], dtype=float)

    R = np.diag([1e-8, 1e-8, 50.0**2])  # 50 ft std dev in altitude

    y = z - H @ x                       # innovation
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)      # Kalman gain

    x = x + K @ y
    I = np.eye(P.shape[0])
    P = (I - K @ H) @ P
    return x, P

class Track:
    """Represents a single aircraft track."""

    __slots__ = (
        "icao", "callsign", "altitude", "speed", "track",
        "lat", "lon", "vertical_rate", "squawk",
        "alert", "emergency", "spi", "on_ground",
        "last_seen", "first_seen","last_pos_seen",
    )

    def __init__(self, icao: str):
        self.icao          = icao
        self.callsign      = None
        self.altitude      = None
        self.speed         = None
        self.track         = None
        self.lat           = None
        self.lon           = None
        self.vertical_rate = None
        self.squawk        = None
        self.alert         = None
        self.emergency     = None
        self.spi           = None
        self.on_ground     = None
        self.first_seen    = time.time()
        self.last_seen     = self.first_seen
        self.last_pos_seen = None

    def update(self, data: dict):
        """Merge a parsed message dict into this track."""
        for field in FIELD_MAP:
            if field in data:
                setattr(self, field, data[field])
        self.last_seen = time.time()
        if "lat" in data and "lon" in data and "altitude" in data:
            self.last_pos_seen = self.last_seen
    def age(self) -> float:
        """Seconds since last update."""
        return time.time() - self.last_seen

    def has_position(self) -> bool:
        return self.lat is not None and self.lon is not None

    def __repr__(self) -> str:
        lat = f"{self.lat:.5f}" if self.lat is not None else "None"
        lon = f"{self.lon:.5f}" if self.lon is not None else "None"
        return (
            f"Track({self.icao}"
            f" cs={self.callsign}"
            f" alt={self.altitude}"
            f" spd={self.speed}"
            f" trk={self.track}"
            f" lat={lat}"
            f" lon={lon}"
            f" age={self.age():.1f}s)"
        )


class TrackList(Thread):
    """
    Self-contained ADS-B track manager.

    Inherits Thread — call start() once and it connects to the ADS-B receiver,
    reads the SBS-1 stream, and keeps an internal dictionary of live aircraft
    tracks up to date. Stale tracks are pruned automatically.

    Usage
    -----
        tracks = TrackList("192.168.20.106", 30003, prune_after=60)
        tracks.start()

        # Retrieve a single track by ICAO address
        track = tracks.get("7C0FF9")

        # Snapshot of all live tracks  {icao: Track}
        all_tracks = tracks.get_all()

        # Stop the thread cleanly
        tracks.stop()
    """

    _PRUNE_INTERVAL = 100   # run prune() every N ingested messages

    def __init__(self, ip: str, port: int, prune_after: float = 60.0):
        """
        Parameters
        ----------
        ip          : ADS-B receiver host (e.g. dump1090 IP)
        port        : BaseStation TCP port (typically 30003)
        prune_after : Seconds of silence before a track is dropped
        """
        super().__init__(daemon=True)
        self._ip            = ip
        self._port          = port
        self.prune_after    = prune_after

        self._tracks: dict[str, Track] = {}
        self._lock          = Lock()
        self._stop_event    = False
        self._prune_counter = 0

    # ------------------------------------------------------------------
    # Thread entry point
    # ------------------------------------------------------------------

    def run(self):
        """Connect to the ADS-B receiver and process the stream."""
        asyncio.run(self._receive_loop())

    async def _receive_loop(self):
        try:
            reader, _ = await telnetlib3.open_connection(self._ip, self._port)
        except Exception as e:
            print(f"[TrackList] Connection failed: {e}")
            return

        print(f"[TrackList] Connected to {self._ip}:{self._port}")
        try:
            while not self._stop_event:
                line = await reader.readline()
                if line:
                    if isinstance(line, bytes):
                        line = line.decode("utf-8", errors="replace")
                    self._ingest(line)
        except Exception as e:
            print(f"[TrackList] Stream error: {e}")
        finally:
            print("[TrackList] Disconnected.")

    # ------------------------------------------------------------------
    # Internal ingest (called from within the thread)
    # ------------------------------------------------------------------

    def _ingest(self, raw: str):
        data = parse_msg(raw)
        if not data:
            return

        icao = data["icao"]
        with self._lock:
            if icao not in self._tracks:
                self._tracks[icao] = Track(icao)
            self._tracks[icao].update(data)

            self._prune_counter += 1
            if self._prune_counter >= self._PRUNE_INTERVAL:
                self._prune()
                self._prune_counter = 0

    def _prune(self):
        """Remove stale tracks. Must be called with self._lock held."""
        stale = [icao for icao, t in self._tracks.items() if t.age() > self.prune_after]
        for icao in stale:
            del self._tracks[icao]

    # ------------------------------------------------------------------
    # Public read interface (safe to call from any external thread)
    # ------------------------------------------------------------------

    def get(self, icao: str) -> Track | None:
        """Return a reference to the Track for *icao*, or None."""
        with self._lock:
            return self._tracks.get(icao)

    def get_all(self) -> dict[str, Track]:
        """Return a shallow-copy snapshot of all live tracks keyed by ICAO."""
        with self._lock:
            return dict(self._tracks)

    def pop(self, icao: str) -> Track | None:
        """Remove and return the Track for *icao*, or None."""
        with self._lock:
            return self._tracks.pop(icao, None)

    def prune(self) -> list[str]:
        """Manually prune stale tracks. Returns list of removed ICAO addresses."""
        with self._lock:
            stale = [icao for icao, t in self._tracks.items() if t.age() > self.prune_after]
            for icao in stale:
                del self._tracks[icao]
        return stale

    def stop(self):
        """Signal the receive loop to exit on its next iteration."""
        self._stop_event = True

    def __len__(self) -> int:
        with self._lock:
            return len(self._tracks)

    def __contains__(self, icao: str) -> bool:
        with self._lock:
            return icao in self._tracks

    def __repr__(self) -> str:
        return (
            f"TrackList(host={self._ip}:{self._port}"
            f" tracks={len(self)}"
            f" prune_after={self.prune_after}s)"
        )

class KalmanTracker:
    def __init__(self, track: Track, lead_s: float = LEAD_TIME_S,
                 max_extrap_s: float = MAX_EXTRAPOLATION_S):
        self._track      = track
        self._lead_s     = lead_s
        self._max_extrap = max_extrap_s

        self._x         = None
        self._P         = None
        self._kf_t      = None
        self._last_seen = None

        self.smooth_lat = None
        self.smooth_lon = None
        self.smooth_alt = None
        self.smooth_t   = None

    def get(self):
        if self.smooth_lat is None:
            return None
        return (self.smooth_lat, self.smooth_lon, self.smooth_alt, self.smooth_t)

    def step(self):
        t_now = time.time()
        track = self._track

        lat       = track.lat
        lon       = track.lon
        alt       = track.altitude
        last_seen = track.last_seen
        last_pos_seen = track.last_pos_seen

        has_pos = (lat is not None and lon is not None and alt is not None
           and last_pos_seen is not None)

        # ── Initialise on first position fix ────────────────────────────────
        if self._x is None:
            if not has_pos:
                return
            self._x, self._P = _kf_init(lat, lon, alt)
            self._kf_t      = t_now
            self._last_seen = last_seen
            self._publish(t_now)
            return

        # ── Predict ──────────────────────────────────────────────────────────
        dt = min(max(t_now - self._kf_t, 0.0), self._max_extrap)
        self._x, self._P = _kf_predict(self._x, self._P, dt)
        self._kf_t = t_now

        # ── Update only on new position fix ──────────────────────────────────
        if has_pos and last_pos_seen != self._last_seen:
            self._x, self._P = _kf_update(self._x, self._P, lat, lon, alt)
            self._last_seen = last_pos_seen

        self._publish(t_now)

    def _publish(self, t_now):
        lead = min(self._lead_s, self._max_extrap)
        x = self._x
        self.smooth_lat = x[0] + x[3] * lead
        self.smooth_lon = x[1] + x[4] * lead
        self.smooth_alt = x[2] + x[5] * lead
        self.smooth_t   = t_now + lead

# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

if __name__ == "__main__":

    ADSB_ADDR   = "192.168.137.140"
    ADSB_PORT   = 30003
    LAT = -33.7615257
    LON = 150.7431018

    tracks = TrackList(ADSB_ADDR, ADSB_PORT, prune_after=60)
    tracks.start()
    print("Populating tracks...")
    time.sleep(3)

    try:
        while True:
            track = closest_track(tracks, LAT, LON)
            if track:
                print(track)
            else:
                print("No tracks.")
            time.sleep(1)

    except KeyboardInterrupt:
        tracks.stop()