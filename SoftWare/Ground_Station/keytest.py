"""지상국의 NTRIP 수신, TTGO 시리얼 통신, 2초 반이중 주기를 관리한다.

이 모듈이 PC와 TTGO 사이의 시리얼 포트를 단독으로 관리한다.
GUI는 직접 시리얼로 전송하지 않고 다음 두 함수에 최신 데이터만 전달한다.

* ``publish_latest_rtcm_frame``: 허용 타입별 최신 RTCM 프레임 하나를 교체한다.
* ``queue_waypoints``: 사용자가 입력한 웨이포인트를 다음 송신 주기까지 보관한다.
* 백그라운드 스케줄러는 2초마다 RTCM과 웨이포인트를 한 번에 TTGO로 넘긴다.

PC→TTGO 패킷은 기존 GUI에서 사용하던 ``[지상국 ID][신호][길이/개수][데이터]``
방식을 유지한다. 기존 Waypoint ``FF F2 ...``는 바꾸지 않고 RTCM과 RX 전환용
신호만 같은 계열에 추가한다.
"""

from __future__ import annotations

import base64
import os
import socket
import struct
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

import serial


# ---------------------------------------------------------------------------
# TTGO 시리얼 포트와 반이중 통신 주기 설정
# 환경변수 GCS_TTGO_PORT를 지정하면 COM8 대신 해당 포트를 사용할 수 있다.
# ---------------------------------------------------------------------------
SERIAL_PORT = os.getenv("GCS_TTGO_PORT", "COM8")
SERIAL_BAUD = int(os.getenv("GCS_TTGO_BAUD", "115200"))
CYCLE_PERIOD_SECONDS = float(os.getenv("GCS_CYCLE_SECONDS", "2.0"))
RTCM_MAX_AGE_SECONDS = float(os.getenv("GCS_RTCM_MAX_AGE_SECONDS", "2.2"))

# ---------------------------------------------------------------------------
# NTRIP 서버 설정
# RTKBYTE.py에서 사용하던 접속 정보를 기본값으로 사용한다. 필요하면
# NTRIP_HOST, NTRIP_PORT 등의 환경변수로 소스 수정 없이 덮어쓸 수 있다.
# ---------------------------------------------------------------------------
NTRIP_HOST = os.getenv("NTRIP_HOST", "www.gnssdata.or.kr")
NTRIP_PORT = int(os.getenv("NTRIP_PORT", "2101"))
NTRIP_MOUNTPOINT = os.getenv("NTRIP_MOUNTPOINT", "SUWN-RTCM31")
NTRIP_USER = os.getenv("NTRIP_USER", "nanana400@naver.com")
NTRIP_PASSWORD = os.getenv("NTRIP_PASSWORD", "gnss")
NTRIP_CONNECT_TIMEOUT = float(os.getenv("NTRIP_CONNECT_TIMEOUT", "10.0"))
NTRIP_READ_TIMEOUT = float(os.getenv("NTRIP_READ_TIMEOUT", "0.2"))
NTRIP_RECONNECT_SECONDS = float(os.getenv("NTRIP_RECONNECT_SECONDS", "3.0"))
NTRIP_RECV_SIZE = int(os.getenv("NTRIP_RECV_SIZE", "4096"))

# LoRa 대역폭을 보정에 필요한 RTCM에만 사용한다. CRC가 정상이어도 이 목록에
# 없는 메시지는 latest RTCM 후보로 등록하지 않는다.
RTCM_PASS_TYPES = frozenset({1004, 1006, 1012, 1230})
RTCM_PASS_ORDER = (1004, 1006, 1012, 1230)

DRONE_ID = 0xFE
GPS_MSG = 0xF3
DATA_LEN = 14
EXTENDED_DATA_LEN = 22
QUALITY_DATA_LEN = 23
EXTENDED_VW_DATA_LEN = 26
QUALITY_VW_DATA_LEN = 27
WAYPOINT_VW_DATA_LEN = 28
TELEMETRY_RAD_SCALE = 1_000_000.0


# ---------------------------------------------------------------------------
# PC → TTGO 기존 패킷 계열
#   Waypoint : [FF][F2][개수 uint8][lat int32][lon int32]...
#   RTCM     : [FF][F4][길이 uint16 LE][원본 RTCM...]
#   RX 전환  : [FF][F5]
#
# FF/F2 Waypoint는 예전 KEYTEST와 완전히 같은 구조다. F4와 F5만 새 기능을
# 구분하기 위해 추가했다. 시리얼 구간에는 별도 Magic/버전/Cycle ID/CRC16을
# 붙이지 않는다. RTCM 프레임 자체의 CRC24Q는 원본 그대로 포함된다.
# ---------------------------------------------------------------------------
GROUND_STATION_ID = 0xFF
KEYBOARD_MSG = 0xF1
WAYPOINT_MSG = 0xF2
RTCM_MSG = 0xF4
ENTER_RX_MSG = 0xF5

# LoRa 웨이포인트에는 타입/순번/명령 ID 공간도 필요하다.
# 현재 TTGO의 128바이트 TX FIFO를 넘지 않도록 한 명령을 최대 15점으로 제한한다.
MAX_WAYPOINTS_PER_COMMAND = 15


@dataclass(frozen=True)
class LatestRtcmFrame:
    """다음 주기에 보낼 완전한 RTCM 프레임 하나를 보관한다."""

    payload: bytes
    message_type: int
    published_at: float
    generation: int


@dataclass(frozen=True)
class WaypointCommand:
    """사용자가 입력한 웨이포인트 묶음과 ACK 추적용 명령 번호다."""

    command_id: int
    points: tuple[tuple[float, float], ...]
    queued_at: float


# TTGO 시리얼, 스케줄러 및 공유 데이터는 서로 다른 스레드에서 접근하므로
# 각각의 lock/event로 보호한다.
_ser: Optional[serial.Serial] = None
_serial_lock = threading.Lock()
_state_lock = threading.Lock()
_stop_event = threading.Event()
_scheduler_thread: Optional[threading.Thread] = None

_ntrip_stop_event = threading.Event()
_ntrip_thread: Optional[threading.Thread] = None
_ntrip_socket: Optional[socket.socket] = None
_ntrip_socket_lock = threading.Lock()

_latest_rtcm_frames: dict[int, LatestRtcmFrame] = {}
_rtcm_generation = 0
_rtcm_replaced_frames = 0
_pending_waypoint: Optional[WaypointCommand] = None
_awaiting_waypoint_ack: dict[int, WaypointCommand] = {}
_next_waypoint_id = 1
_cycle_count = 0
_awaiting_downlink_end: Optional[tuple[int, str]] = None

_link_events: deque[str] = deque(maxlen=200)


def _emit_event(message: str) -> None:
    """터미널에 상태를 출력하고 GUI가 읽을 이벤트 큐에도 같은 내용을 넣는다."""

    print(message)
    with _state_lock:
        _link_events.append(message)


def get_link_events() -> list[str]:
    """통신 스레드가 쌓아 둔 상태 메시지를 GUI에 전달하고 큐에서 제거한다."""

    events: list[str] = []
    with _state_lock:
        while _link_events:
            events.append(_link_events.popleft())
    return events


def _build_rtcm_packet(payload: bytes) -> bytes:
    """기존 패킷 계열을 따라 ``FF F4 길이 RTCM`` 패킷을 만든다."""

    if not payload:
        raise ValueError("RTCM payload is empty")
    if len(payload) > 0xFFFF:
        raise ValueError("RTCM payload is too large for uint16 length")
    return bytes((GROUND_STATION_ID, RTCM_MSG)) + struct.pack("<H", len(payload)) + payload


def _build_enter_rx_packet() -> bytes:
    """TTGO에 LoRa 수신 모드 진입을 요청하는 ``FF F5`` 패킷을 만든다."""

    return bytes((GROUND_STATION_ID, ENTER_RX_MSG))


def _crc24q(data: bytes) -> int:
    """RTCM3 프레임 끝의 24비트 CRC24Q 값을 계산한다."""

    crc = 0
    for value in data:
        crc ^= value << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class _RtcmStreamDecoder:
    """임의 크기의 TCP 수신 데이터를 CRC가 정상인 RTCM3 프레임으로 복원한다."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.valid_frames = 0
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buffer.extend(data)
        decoded: list[tuple[int, bytes]] = []

        while True:
            try:
                start = self.buffer.index(0xD3)
            except ValueError:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break

            if start:
                self.discarded_bytes += start
                del self.buffer[:start]

            if len(self.buffer) < 3:
                break

            if self.buffer[1] & 0xFC:
                self.discarded_bytes += 1
                del self.buffer[0]
                continue

            payload_length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
            frame_length = 3 + payload_length + 3
            if len(self.buffer) < frame_length:
                break

            frame = bytes(self.buffer[:frame_length])
            received_crc = int.from_bytes(frame[-3:], "big")
            if _crc24q(frame[:-3]) != received_crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue

            del self.buffer[:frame_length]
            message_type = (
                (frame[3] << 4) | (frame[4] >> 4)
                if payload_length >= 2
                else -1
            )
            self.valid_frames += 1
            decoded.append((message_type, frame))

        return decoded


def _validate_rtcm_frame(frame: bytes) -> int:
    """CRC가 정상인 완전한 RTCM3 프레임 하나인지 검사하고 타입을 반환한다."""

    if len(frame) < 6:
        raise ValueError("RTCM frame is too short")
    if frame[0] != 0xD3:
        raise ValueError("RTCM preamble missing")
    if frame[1] & 0xFC:
        raise ValueError("invalid RTCM reserved bits")

    payload_length = ((frame[1] & 0x03) << 8) | frame[2]
    expected_length = 3 + payload_length + 3
    if len(frame) != expected_length:
        raise ValueError(
            f"RTCM frame length mismatch: received={len(frame)}, "
            f"expected={expected_length}"
        )

    received_crc = int.from_bytes(frame[-3:], "big")
    if _crc24q(frame[:-3]) != received_crc:
        raise ValueError("RTCM CRC24Q mismatch")
    return (frame[3] << 4) | (frame[4] >> 4) if payload_length >= 2 else -1


def publish_latest_rtcm_frame(
    message_type: int, frame: bytes, published_at: Optional[float] = None
) -> int:
    """허용 타입별 최신 정상 프레임 하나만 저장하고 같은 타입만 덮어쓴다."""

    global _rtcm_generation, _rtcm_replaced_frames

    if message_type not in RTCM_PASS_TYPES:
        raise ValueError(f"RTCM type {message_type} is not allowed")

    payload = bytes(frame)
    decoded_type = _validate_rtcm_frame(payload)
    if decoded_type != message_type:
        raise ValueError(
            f"RTCM type mismatch: decoder={message_type}, frame={decoded_type}"
        )

    with _state_lock:
        if message_type in _latest_rtcm_frames:
            _rtcm_replaced_frames += 1
        _rtcm_generation += 1
        generation = _rtcm_generation
        _latest_rtcm_frames[message_type] = LatestRtcmFrame(
            payload=payload,
            message_type=message_type,
            published_at=time.monotonic() if published_at is None else published_at,
            generation=generation,
        )
    return generation


def _build_ntrip_request() -> bytes:
    """기존 계정 정보를 HTTP Basic 인증 헤더로 만들어 NTRIP 요청을 구성한다."""

    mountpoint = NTRIP_MOUNTPOINT.lstrip("/")
    lines = [
        f"GET /{mountpoint} HTTP/1.0",
        f"Host: {NTRIP_HOST}",
        "User-Agent: NTRIP GCS/1.0",
        "Ntrip-Version: Ntrip/2.0",
        "Connection: close",
    ]
    if NTRIP_USER or NTRIP_PASSWORD:
        token = f"{NTRIP_USER}:{NTRIP_PASSWORD}".encode("utf-8")
        auth = base64.b64encode(token).decode("ascii")
        lines.append(f"Authorization: Basic {auth}")
    return ("\r\n".join(lines) + "\r\n\r\n").encode("ascii")


def _read_ntrip_header(sock: socket.socket) -> tuple[str, bytes]:
    """NTRIP 응답 헤더를 읽고 접속 성공 여부와 첫 RTCM 데이터를 반환한다."""

    response = bytearray()
    while b"\r\n\r\n" not in response and b"\n\n" not in response:
        chunk = sock.recv(1)
        if not chunk:
            raise ConnectionError("NTRIP server closed during handshake")
        response.extend(chunk)
        if len(response) > 16384:
            raise RuntimeError("NTRIP response header is too large")

    raw = bytes(response)
    separator = b"\r\n\r\n" if b"\r\n\r\n" in raw else b"\n\n"
    header, first_payload = raw.split(separator, 1)
    header_text = header.decode("iso-8859-1", errors="replace")
    first_line = header_text.splitlines()[0] if header_text.splitlines() else ""
    upper_line = first_line.upper()
    accepted = upper_line.startswith("ICY 200") or (
        upper_line.startswith("HTTP/") and " 200" in upper_line
    )
    if not accepted:
        raise RuntimeError(f"NTRIP connection rejected: {first_line or 'empty response'}")
    return first_line, first_payload


def _open_ntrip_socket() -> tuple[socket.socket, bytes]:
    """NTRIP 서버에 접속하고 인증이 끝난 수신 소켓을 준비한다."""

    sock = socket.create_connection(
        (NTRIP_HOST, NTRIP_PORT), timeout=NTRIP_CONNECT_TIMEOUT
    )
    try:
        sock.settimeout(NTRIP_CONNECT_TIMEOUT)
        sock.sendall(_build_ntrip_request())
        _first_line, first_payload = _read_ntrip_header(sock)
        sock.settimeout(NTRIP_READ_TIMEOUT)
        return sock, first_payload
    except Exception:
        sock.close()
        raise


def _ntrip_worker() -> None:
    """RTCM을 계속 수신하며 연결이 끊기면 설정된 간격으로 자동 재접속한다."""

    global _ntrip_socket

    while not _ntrip_stop_event.is_set():
        decoder = _RtcmStreamDecoder()
        received_bytes = 0
        published_frames = 0
        filtered_frames = 0
        filtered_types: Counter[int] = Counter()
        report_at = time.monotonic() + 5.0
        sock: Optional[socket.socket] = None

        try:
            _emit_event(
                f"[NTRIP] connecting {NTRIP_HOST}:{NTRIP_PORT}/"
                f"{NTRIP_MOUNTPOINT.lstrip('/')}"
            )
            sock, pending = _open_ntrip_socket()
            with _ntrip_socket_lock:
                _ntrip_socket = sock
            _emit_event("[NTRIP] connected; receiving RTCM")
            _emit_event(
                "[RTCM] filter pass types="
                + ",".join(str(value) for value in sorted(RTCM_PASS_TYPES))
            )

            while not _ntrip_stop_event.is_set():
                try:
                    if pending:
                        chunk, pending = pending, b""
                    else:
                        chunk = sock.recv(NTRIP_RECV_SIZE)
                    if not chunk:
                        raise ConnectionError("NTRIP server closed the stream")

                    now = time.monotonic()
                    received_bytes += len(chunk)
                    for message_type, frame in decoder.feed(chunk):
                        if message_type not in RTCM_PASS_TYPES:
                            filtered_frames += 1
                            filtered_types[message_type] += 1
                            continue
                        publish_latest_rtcm_frame(message_type, frame, now)
                        published_frames += 1
                except socket.timeout:
                    pass

                now = time.monotonic()
                if now >= report_at:
                    filtered_summary = ",".join(
                        f"{message_type}:{count}"
                        for message_type, count in sorted(filtered_types.items())
                    ) or "none"
                    _emit_event(
                        f"[NTRIP] RX={received_bytes} B, "
                        f"valid={decoder.valid_frames}, CRC errors={decoder.crc_errors}, "
                        f"allowed={published_frames}, filtered={filtered_frames} "
                        f"({filtered_summary}), latest-frame updates={published_frames}"
                    )
                    report_at = now + 5.0

        except Exception as exc:
            if not _ntrip_stop_event.is_set():
                _emit_event(
                    f"[NTRIP] {exc}; reconnecting in "
                    f"{NTRIP_RECONNECT_SECONDS:.1f}s"
                )
        finally:
            with _ntrip_socket_lock:
                if _ntrip_socket is sock:
                    _ntrip_socket = None
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

        if _ntrip_stop_event.wait(max(0.0, NTRIP_RECONNECT_SECONDS)):
            break


def start_ntrip_receiver() -> None:
    """자동 재접속 기능이 있는 NTRIP 수신 스레드를 시작한다."""

    global _ntrip_thread

    if _ntrip_thread is not None and _ntrip_thread.is_alive():
        return
    if not NTRIP_HOST or not NTRIP_MOUNTPOINT:
        raise ValueError("NTRIP host and mountpoint are required")

    _ntrip_stop_event.clear()
    _ntrip_thread = threading.Thread(
        target=_ntrip_worker,
        name="gcs-ntrip-receiver",
        daemon=True,
    )
    _ntrip_thread.start()


def stop_ntrip_receiver() -> None:
    """NTRIP 소켓을 닫아 대기 중인 recv를 깨우고 수신 스레드를 종료한다."""

    global _ntrip_thread, _ntrip_socket

    _ntrip_stop_event.set()
    with _ntrip_socket_lock:
        sock, _ntrip_socket = _ntrip_socket, None
    if sock is not None:
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    thread, _ntrip_thread = _ntrip_thread, None
    if thread is not None and thread is not threading.current_thread():
        thread.join(timeout=max(2.5, NTRIP_READ_TIMEOUT + 0.5))


def queue_waypoints(points: Iterable[Sequence[float]]) -> int:
    """최신 웨이포인트 명령을 다음 지상국 송신 구간까지 보관한다."""

    global _pending_waypoint, _next_waypoint_id

    normalized = tuple((float(point[0]), float(point[1])) for point in points)
    if not normalized:
        raise ValueError("at least one waypoint is required")
    if len(normalized) > MAX_WAYPOINTS_PER_COMMAND:
        raise ValueError(
            f"maximum {MAX_WAYPOINTS_PER_COMMAND} waypoints per command "
            "with the current 128-byte LoRa TX FIFO"
        )

    for lat, lon in normalized:
        if not -90.0 <= lat <= 90.0:
            raise ValueError(f"invalid latitude: {lat}")
        if not -180.0 <= lon <= 180.0:
            raise ValueError(f"invalid longitude: {lon}")

    with _state_lock:
        command_id = _next_waypoint_id
        _next_waypoint_id = 1 if command_id >= 0xFFFF else command_id + 1
        _pending_waypoint = WaypointCommand(command_id, normalized, time.monotonic())
    return command_id


def _build_waypoint_packet(command: WaypointCommand) -> bytes:
    """예전과 같은 ``FF F2 개수 좌표...`` Waypoint 패킷을 만든다."""

    # 기존 KEYTEST와 동일하게 command_id는 전송하지 않는다. 이 번호는 현재
    # GUI 로그와 내부 대기 목록에서만 사용한다.
    packet = bytearray(
        (GROUND_STATION_ID, WAYPOINT_MSG, len(command.points) & 0xFF)
    )
    for lat, lon in command.points:
        lat_i = int(round(lat * 1e7))
        lon_i = int(round(lon * 1e7))
        packet.extend(struct.pack("<ii", lat_i, lon_i))
    return bytes(packet)


def acknowledge_waypoint(command_id: int) -> bool:
    """드론 ACK가 확인된 웨이포인트 명령을 ACK 대기 목록에서 제거한다."""

    with _state_lock:
        return _awaiting_waypoint_ack.pop(command_id, None) is not None


def _take_cycle_inputs() -> tuple[
    list[LatestRtcmFrame],
    Optional[WaypointCommand],
    tuple[int, ...],
    tuple[int, ...],
]:
    """타입별 최신 RTCM을 한꺼번에 꺼내고 stale/missing 타입을 함께 반환한다."""

    global _pending_waypoint

    selected: list[LatestRtcmFrame] = []
    stale_types: list[int] = []
    missing_types: list[int] = []
    now = time.monotonic()

    with _state_lock:
        waypoint = _pending_waypoint
        _pending_waypoint = None

        # 각 타입 슬롯에는 최신 프레임 하나만 존재한다. 이번 cycle에서 꺼낸 슬롯은
        # 제거하므로 같은 프레임을 다음 cycle에 다시 보내지 않는다.
        for message_type in RTCM_PASS_ORDER:
            candidate = _latest_rtcm_frames.pop(message_type, None)
            if candidate is None:
                missing_types.append(message_type)
                continue

            age = now - candidate.published_at
            if age > RTCM_MAX_AGE_SECONDS:
                stale_types.append(message_type)
                continue

            selected.append(candidate)

    return selected, waypoint, tuple(stale_types), tuple(missing_types)


def _write_all(data: bytes) -> int:
    """레코드 전체가 기록될 때까지 TTGO 시리얼 포트에 반복해서 쓴다."""

    serial_port = _ser
    if serial_port is None or not serial_port.is_open:
        raise serial.SerialException(f"{SERIAL_PORT} is not open")

    total = 0
    view = memoryview(data)
    with _serial_lock:
        while total < len(data):
            written = serial_port.write(view[total:])
            if written <= 0:
                raise serial.SerialTimeoutException("serial write made no progress")
            total += written
    return total


def _run_downlink_cycle() -> None:
    """타입별 최신 RTCM과 선택적 웨이포인트 뒤에 RX 전환을 지시한다."""

    global _cycle_count, _pending_waypoint, _awaiting_downlink_end

    # cycle은 터미널 로그를 보기 위한 내부 카운터일 뿐 패킷에는 포함하지 않는다.
    _cycle_count += 1
    cycle = _cycle_count
    rtcm_frames, waypoint, stale_types, missing_types = _take_cycle_inputs()

    packets: list[bytes] = []
    rtcm_bytes = 0
    rtcm_types: list[int] = []
    rtcm_details: list[str] = []
    waypoint_id: Optional[int] = None

    _emit_event(f"[CYCLE {cycle}] BEGIN")

    # RTCM_PASS_ORDER 순서로 반환된 모든 신선한 프레임을 같은 cycle에 넣는다.
    # 패킷 형식은 각 프레임마다 기존 FF F4 + uint16 length + RTCM 그대로다.
    for rtcm_frame in rtcm_frames:
        age = time.monotonic() - rtcm_frame.published_at
        packets.append(_build_rtcm_packet(rtcm_frame.payload))
        rtcm_bytes += len(rtcm_frame.payload)
        rtcm_types.append(rtcm_frame.message_type)
        rtcm_details.append(
            f"{rtcm_frame.message_type}({len(rtcm_frame.payload)}B, age={age:.2f}s)"
        )

    selected_text = ",".join(str(value) for value in rtcm_types) or "none"
    summary_parts = [f"[CYCLE {cycle}] RTCM set: {selected_text}"]
    if stale_types:
        summary_parts.append("stale=" + ",".join(str(value) for value in stale_types))
    if missing_types:
        summary_parts.append("missing=" + ",".join(str(value) for value in missing_types))
    _emit_event(" / ".join(summary_parts))
    if rtcm_details:
        _emit_event(f"[CYCLE {cycle}] RTCM detail: " + "; ".join(rtcm_details))

    if waypoint is not None:
        # 기존 패킷 구조 FF F2 COUNT LAT/LON...를 그대로 사용한다.
        packets.append(_build_waypoint_packet(waypoint))
        waypoint_id = waypoint.command_id
        _emit_event(
            f"[CYCLE {cycle}] Waypoint: WP#{waypoint_id}, "
            f"points={len(waypoint.points)}"
        )
    else:
        _emit_event(f"[CYCLE {cycle}] Waypoint: none")

    # TTGO는 앞의 LoRa 송신을 모두 마친 뒤 FF F5를 보고 RX 모드로 들어가야 한다.
    # TTGO는 FF F5를 무선 A3(DOWNLINK_END)로 바꿔 송신하고 D 확인 line을 돌려준다.
    packets.append(_build_enter_rx_packet())

    with _state_lock:
        _awaiting_downlink_end = (cycle, selected_text)

    try:
        _write_all(b"".join(packets))
    except (OSError, serial.SerialException, serial.SerialTimeoutException) as exc:
        # 실패한 RTCM은 일부러 되돌려 놓지 않는다. 오래된 보정값을 다음 주기에
        # 재전송하면 안 되기 때문이다. 전달이 중요한 웨이포인트만 대기 상태로 복구한다.
        if waypoint is not None:
            with _state_lock:
                if _pending_waypoint is None:
                    _pending_waypoint = waypoint
        with _state_lock:
            if (_awaiting_downlink_end is not None and
                    _awaiting_downlink_end[0] == cycle):
                _awaiting_downlink_end = None
        _emit_event(f"[CYCLE {cycle}] serial TX failed: {exc}")
        return

    if waypoint is not None:
        with _state_lock:
            _awaiting_waypoint_ack[waypoint.command_id] = waypoint

    if rtcm_bytes > 0:
        _emit_event(
            f"[RTCM TX] cycle={cycle}, types={selected_text}, "
            f"frames={len(rtcm_frames)}, queued={rtcm_bytes} B"
        )
    else:
        _emit_event(f"[RTCM TX] cycle={cycle}, no fresh RTCM")

    if waypoint_id is not None:
        _emit_event(
            f"[WAYPOINT TX] cycle={cycle}, WP#{waypoint_id} queued to TTGO"
        )
    _emit_event(f"[CYCLE {cycle}] DOWNLINK_END queued to TTGO")


def _scheduler_worker() -> None:
    """기준 시각을 유지하면서 정확히 2초 간격으로 지상국 송신 주기를 실행한다."""

    if CYCLE_PERIOD_SECONDS <= 0:
        _emit_event("[GCS LINK] invalid cycle period; scheduler stopped")
        return

    deadline = time.monotonic() + CYCLE_PERIOD_SECONDS
    while not _stop_event.is_set():
        if _stop_event.wait(max(0.0, deadline - time.monotonic())):
            break

        _run_downlink_cycle()
        deadline += CYCLE_PERIOD_SECONDS

        now = time.monotonic()
        if now >= deadline:
            skipped = int((now - deadline) // CYCLE_PERIOD_SECONDS) + 1
            deadline += skipped * CYCLE_PERIOD_SECONDS
            _emit_event(f"[GCS LINK] skipped {skipped} expired cycle(s)")


def start_ground_link(port: Optional[str] = None, baudrate: Optional[int] = None) -> None:
    """TTGO 시리얼 포트를 열고 2초 송신 스케줄러를 시작한다."""

    global _ser, _scheduler_thread, SERIAL_PORT, SERIAL_BAUD

    if port:
        SERIAL_PORT = port
    if baudrate:
        SERIAL_BAUD = int(baudrate)

    if _ser is not None and _ser.is_open:
        return

    _ser = serial.Serial(
        SERIAL_PORT,
        SERIAL_BAUD,
        timeout=0.1,
        write_timeout=2.0,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    _stop_event.clear()
    _scheduler_thread = threading.Thread(
        target=_scheduler_worker,
        name="gcs-two-second-cycle",
        daemon=True,
    )
    _scheduler_thread.start()
    _emit_event(
        f"[GCS LINK] {SERIAL_PORT} open @ {SERIAL_BAUD}, "
        f"cycle={CYCLE_PERIOD_SECONDS:.2f}s"
    )


def stop_ground_link() -> None:
    """송신 스케줄러를 먼저 멈춘 다음 TTGO 시리얼 포트를 닫는다."""

    global _ser, _scheduler_thread

    _stop_event.set()
    thread, _scheduler_thread = _scheduler_thread, None
    if thread is not None and thread is not threading.current_thread():
        thread.join(timeout=2.5)

    serial_port, _ser = _ser, None
    if serial_port is not None:
        serial_port.close()


def send_ttgo(id: int, signal: int, msg):
    """이전 GUI 호출 형식을 유지하기 위한 호환 함수다.

    웨이포인트는 즉시 보내지 않고 큐에 넣는다. 새 RTK 구조에서는 RTCM과
    웨이포인트를 같은 송신 구간에 보내므로 키보드 명령의 즉시 송신은 막는다.
    """

    if signal == WAYPOINT_MSG:
        return queue_waypoints(msg)
    if signal == KEYBOARD_MSG:
        raise RuntimeError("immediate keyboard TX is disabled in scheduled RTK mode")
    raise ValueError(f"unsupported signal: 0x{signal:02X}")


# ---------------------------------------------------------------------------
# TTGO → PC 수신 라인 해석
# 기존 R/P ASCII 상태 보고를 유지한다. A 라인은 향후 웨이포인트 ACK용이고,
# D 라인은 TTGO가 A3를 실제 송신하고 RX로 전환했다는 확인이다.
# A,millis,waypoint_id,ok
# D,millis,ok
# ---------------------------------------------------------------------------
def read_ttgo():
    """TTGO가 올린 RSSI, 드론 상태 또는 웨이포인트 ACK 한 줄을 해석한다."""

    serial_port = _ser
    if serial_port is None or not serial_port.is_open:
        return None

    try:
        if serial_port.in_waiting <= 0:
            return None

        raw_line = serial_port.readline()
        line = raw_line.decode("ascii", errors="ignore").strip()
        if not line:
            return None

        if not line.startswith(("R,", "P,", "A,", "D,")):
            return None

        parts = line.split(",")

        if parts[0] == "R":
            if len(parts) < 3:
                return None
            return {
                "type": "rssi",
                "time_ms": int(parts[1]),
                "rssi": int(parts[2]),
            }

        if parts[0] == "A":
            if len(parts) < 4:
                return None
            command_id = int(parts[2])
            ok = int(parts[3])
            if ok:
                acknowledge_waypoint(command_id)
            return {
                "type": "waypoint_ack",
                "time_ms": int(parts[1]),
                "command_id": command_id,
                "ok": ok,
            }

        if parts[0] == "D":
            if len(parts) < 3:
                return None
            ok = int(parts[2])
            global _awaiting_downlink_end
            with _state_lock:
                awaiting = _awaiting_downlink_end
                _awaiting_downlink_end = None
            cycle = awaiting[0] if awaiting is not None else None
            selected_text = awaiting[1] if awaiting is not None else "unknown"
            cycle_text = str(cycle) if cycle is not None else "?"
            if ok:
                _emit_event(
                    f"[CYCLE {cycle_text}] RTCM set sent: {selected_text}"
                )
                _emit_event(f"[CYCLE {cycle_text}] DOWNLINK_END sent -> RX")
            else:
                _emit_event(f"[CYCLE {cycle_text}] DOWNLINK_END TX failed -> RX")
            return {
                "type": "downlink_end",
                "time_ms": int(parts[1]),
                "cycle": cycle,
                "ok": ok,
            }

        if len(parts) < 6:
            return None

        time_ms = int(parts[1])
        packet_rssi = int(parts[2])
        snr = int(parts[3]) / 100.0
        ok = int(parts[4])
        payload_hex = parts[5].replace(" ", "")
        raw_bytes = bytes.fromhex(payload_hex)

        if len(raw_bytes) < DATA_LEN:
            return None

        packet_id = raw_bytes[0]
        gps_seq = raw_bytes[1]
        comp = raw_bytes[2]
        gps_len = raw_bytes[3]

        if packet_id != DRONE_ID or comp != GPS_MSG:
            return None

        packet_size = 4 + gps_len * 8 + 2
        if gps_len < 1 or len(raw_bytes) < packet_size:
            return None

        lat_i, lon_i = struct.unpack_from("<ii", raw_bytes, 4)
        lat = lat_i / 1e7
        lon = lon_i / 1e7
        n6_offset = 4 + gps_len * 8
        detected_flag = raw_bytes[n6_offset]
        person_count = raw_bytes[n6_offset + 1]
        psi = None
        v = None
        w = None
        quality = None
        waypoint_target = None
        if len(raw_bytes) >= EXTENDED_VW_DATA_LEN:
            psi_i, v_i, w_i = struct.unpack_from("<iii", raw_bytes, 14)
            psi = psi_i / TELEMETRY_RAD_SCALE
            v = v_i / TELEMETRY_RAD_SCALE
            w = w_i / TELEMETRY_RAD_SCALE
        elif len(raw_bytes) >= EXTENDED_DATA_LEN:
            psi_i, w_i = struct.unpack_from("<ii", raw_bytes, 14)
            psi = psi_i / TELEMETRY_RAD_SCALE
            w = w_i / TELEMETRY_RAD_SCALE
        if len(raw_bytes) >= QUALITY_VW_DATA_LEN:
            quality = raw_bytes[26]
        elif len(raw_bytes) >= QUALITY_DATA_LEN:
            quality = raw_bytes[22]
        if len(raw_bytes) >= WAYPOINT_VW_DATA_LEN:
            waypoint_target = raw_bytes[27]

        telemetry_text = ""
        if psi is not None:
            telemetry_text = f"psi={psi:.4f} rad, "
            if v is not None:
                telemetry_text += f"v={v:.4f} m/s, "
            if w is not None:
                telemetry_text += f"w={w:.4f} rad/s, "
        quality_text = ""
        if quality is not None:
            quality_text = f"quality={quality}, "
        waypoint_text = ""
        if waypoint_target is not None:
            waypoint_text = f"wp={waypoint_target}, "

        _emit_event(
            f"[GPS RX] lat={lat:.7f}, lon={lon:.7f}, seq={gps_seq}, "
            f"det={detected_flag}, count={person_count}, "
            f"{telemetry_text}{quality_text}{waypoint_text}RSSI={packet_rssi} dBm, SNR={snr:.2f} dB"
        )

        return {
            "type": "gps",
            "time_ms": time_ms,
            "packet_rssi": packet_rssi,
            "snr": snr,
            "ok": ok,
            "lat": lat,
            "lon": lon,
            "seq": gps_seq,
            "len": gps_len,
            "detected": detected_flag,
            "person_count": person_count,
            "psi": psi,
            "v": v,
            "w": w,
            "quality": quality,
            "waypoint_target": waypoint_target,
        }

    except Exception as exc:
        _emit_event(f"[GCS LINK] read error: {exc}")
        return None
