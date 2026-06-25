# -*- coding: utf-8 -*-
"""
햄스터 로봇 스마트 충전 내비게이션 시스템 - 백엔드 (Flask)
=============================================================
웹에서 받은 격자 지도와 로봇에서 읽은 배터리 잔량을 바탕으로
다익스트라(Dijkstra)로 '도달 가능한 최단 충전소'를 찾고,
좌표 경로를 로봇 상대 명령(직진/좌회전/우회전)으로 변환한 뒤
roboid 라이브러리로 햄스터 로봇을 실제 구동한다.

지도 셀 값 규약:  0=벽, 1=길, 2=출발지, 3=충전소, 4=목적지
"""

from flask import Flask, render_template, request, jsonify
import heapq
import os
import time
import webbrowser
from threading import Timer, Thread, Lock

# ---------------------------------------------------------------------------
# roboid(하드웨어 라이브러리)는 로봇이 연결돼 있을 때만 필요하다.
# 미설치/미연결 상태에서도 웹과 알고리즘은 정상 동작하도록 안전하게 import 한다.
# ---------------------------------------------------------------------------
try:
    from roboid import HamsterS, wait
    ROBOID_AVAILABLE = True
except Exception:
    ROBOID_AVAILABLE = False

    def wait(ms):  # roboid 미설치 시 사용할 대체 함수
        import time
        time.sleep(ms / 1000.0)

# 태양광 추적기(아두이노) 시리얼 통신용 pyserial. 미설치/미연결이어도 앱은 동작한다.
try:
    import serial  # pyserial
    SERIAL_AVAILABLE = True
except Exception:
    SERIAL_AVAILABLE = False


app = Flask(__name__)

# ===========================================================================
# [1단계 산출물] 하드웨어 주행 보정 상수
#   - 실제 로봇으로 실측한 뒤 이 값들만 바꾸면 된다.
#   - ENABLE_HARDWARE=False 이면 로봇 없이 알고리즘/웹만 테스트(시뮬레이션)한다.
# ===========================================================================
ENABLE_HARDWARE = True    # 로봇을 실제로 굴릴 때 True 로 변경 (테스트만 할 땐 False)
HAMSTER_PORT = "COM7"     # 햄스터S 블루투스 동글 포트 (태양광 아두이노 COM6/COM8 과 구분)

# === 주행 방식 선택 ===
#   "board" : [권장] 바닥에 그린 라인을 센서로 인식하며 주행.
#             교차점을 로봇이 스스로 감지 → 시간 보정 불필요, 오차 누적 없음.
#   "time"  : 바퀴를 정해진 시간만큼 굴리는 방식. calibrate.py로 아래 값 보정 필요.
DRIVE_MODE = "board"

# ↓ DRIVE_MODE = "time" 일 때만 사용하는 보정 상수 (calibrate.py로 실측)
FORWARD_SPEED = 30        # 직진 바퀴 속도
FORWARD_TIME = 1200       # 격자 한 칸(예: 10cm) 이동 시간(ms)
TURN_SPEED = 30           # 회전 바퀴 속도
TURN_TIME = 600           # 90도 회전 시간(ms)
STOP_TIME = 500           # 동작 간 안정적 정지 시간(ms)

# === 배터리 / 에너지 모델 ===
PERCENT_PER_CELL = 2          # 한 칸 이동 시 소모되는 배터리 (%)
SIM_BATTERY_PERCENT = 100     # 로봇 미연결(시뮬레이션) 시 사용할 가상 배터리 (%)
# 햄스터 battery_state()는 3단계(2/1/0)만 제공 → 대략적 %로 환산 (실측 후 조정 권장)
BATTERY_STATE_TO_PERCENT = {2: 100, 1: 30, 0: 5}   # NORMAL / LOW / EMPTY

# === 태양광 추적기(아두이노) 연동 ===
SOLAR_BAUD = 9600             # 아두이노 Serial.begin(9600) 과 일치
# [전압 우선용] 충전소별 태양광 추적기 포트.
#   지도 스캔 순서(위→아래, 왼→오른쪽)대로 충전소에 1:1 매칭된다.
#   예: 첫 번째 충전소 → SOLAR_STATION_PORTS[0], 두 번째 → SOLAR_STATION_PORTS[1]
SOLAR_STATION_PORTS = ["COM6", "COM8"]   # ★ 태양광 추적기 2개를 연결한 포트 (충전소 순서대로)

# === 집게 설정 ===
GRIPPER_HOLD_MS = 1000    # 집기 유지 시간 (ms)


# ===========================================================================
# 1. 지도(2차원 배열) → 그래프(인접 리스트) 변환
# ===========================================================================
def build_graph(grid):
    """격자 지도를 그래프로 변환하고 출발지/충전소/목적지 좌표를 함께 반환한다.

    Returns:
        graph       : {(r, c): {이웃(r,c): 가중치}} 형태의 인접 리스트
        start       : 출발지 좌표 (없으면 None)
        stations    : 충전소 좌표 리스트
        destination : 목적지 좌표 (없으면 None)
    """
    rows = len(grid)
    cols = len(grid[0]) if rows else 0

    graph = {}
    start = None
    stations = []
    destination = None

    # (1) 벽(0)이 아닌 칸만 노드로 등록
    for r in range(rows):
        for c in range(cols):
            cell = grid[r][c]
            if cell == 0:
                continue  # 벽은 길이 아니므로 노드에서 제외
            graph[(r, c)] = {}
            if cell == 2:
                start = (r, c)
            elif cell == 3:
                stations.append((r, c))
            elif cell == 4:
                destination = (r, c)

    # (2) 상하좌우로 인접한 노드끼리 간선(가중치 1) 연결
    for (r, c) in graph:
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            neighbor = (r + dr, c + dc)
            if neighbor in graph:
                graph[(r, c)][neighbor] = 1

    return graph, start, stations, destination


# ===========================================================================
# 2. 다익스트라 알고리즘 (우선순위 큐 heapq 사용)
# ===========================================================================
def dijkstra(graph, start):
    """start 노드로부터 모든 노드까지의 최단거리와 경로 추적 정보를 계산한다."""
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    prev = {node: None for node in graph}  # 경로 복원을 위한 직전 노드 기록

    pq = [(0, start)]  # (누적거리, 노드)
    while pq:
        current_distance, current_node = heapq.heappop(pq)

        # 이미 더 짧은 경로로 처리된 노드면 건너뜀
        if current_distance > distances[current_node]:
            continue

        for next_node, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[next_node]:
                distances[next_node] = distance
                prev[next_node] = current_node
                heapq.heappush(pq, (distance, next_node))

    return distances, prev


def reconstruct_path(prev, target):
    """prev 딕셔너리에서 target까지의 경로를 역추적해 반환한다."""
    path = []
    node = target
    while node is not None:
        path.append(node)
        node = prev[node]
    path.reverse()
    return path


def get_final_direction(path, start_dir):
    """경로를 주행한 후 로봇이 바라보는 방향을 반환한다."""
    if len(path) <= 1:
        return start_dir
    delta_to_dir = {(-1, 0): "UP", (0, 1): "RIGHT", (1, 0): "DOWN", (0, -1): "LEFT"}
    r1, c1 = path[-2]
    r2, c2 = path[-1]
    return delta_to_dir.get((r2 - r1, c2 - c1), start_dir)


# ===========================================================================
# 3. 좌표 경로 → 로봇 상대 경로 명령(시퀀스) 변환
#    좌표계: (row, col), row 증가 = 아래쪽(DOWN)
#    로봇은 처음에 위(UP)를 바라본다고 가정.
# ===========================================================================
def convert_to_relative_commands(path, start_dir="UP"):
    """[(r,c), ...] 좌표 경로를 ['forward','turn_right',...] 명령 배열로 변환."""
    dirs = ["UP", "RIGHT", "DOWN", "LEFT"]           # 시계방향 정렬
    delta = {"UP": (-1, 0), "RIGHT": (0, 1),
             "DOWN": (1, 0), "LEFT": (0, -1)}

    current_dir = start_dir
    commands = []

    for i in range(len(path) - 1):
        (r1, c1), (r2, c2) = path[i], path[i + 1]
        move = (r2 - r1, c2 - c1)

        # 이동 벡터로 목표 방향 결정
        target_dir = None
        for name, vec in delta.items():
            if vec == move:
                target_dir = name
                break

        # 현재 방향 → 목표 방향 회전량 계산 (시계방향 인덱스 차이)
        diff = (dirs.index(target_dir) - dirs.index(current_dir)) % 4
        if diff == 1:
            commands.append("turn_right")
        elif diff == 2:
            commands.append("turn_back")   # 180도
        elif diff == 3:
            commands.append("turn_left")

        commands.append("forward")
        current_dir = target_dir

    return commands


# ===========================================================================
# 햄스터 연결(싱글턴) & 배터리 읽기
# ===========================================================================
_hamster = None
_drive_lock = Lock()   # 동시에 두 개의 주행이 겹치지 않도록 보호


def get_hamster():
    """햄스터 인스턴스를 한 번만 생성해 재사용한다 (중복 연결 방지)."""
    global _hamster
    if _hamster is None:
        _hamster = HamsterS(port_name=HAMSTER_PORT)   # COM7 동글로 연결 (연결될 때까지 대기)
        wait(1000)                                    # 연결 안정화
    return _hamster


def read_battery_from_robot():
    """로봇에서 배터리 상태를 읽어 %로 환산해 반환. 로봇이 없으면 None.
    ※ 햄스터는 정밀 %가 아니라 3단계 상태만 제공하므로 BATTERY_STATE_TO_PERCENT로 환산."""
    if ENABLE_HARDWARE and ROBOID_AVAILABLE:
        state = get_hamster().battery_state()
        return BATTERY_STATE_TO_PERCENT.get(state, 100)
    return None


# --- 태양광 추적기(아두이노) 시리얼: 포트별로 백그라운드에서 최신 전압을 읽어둔다 ---
_solar_voltages = {}    # {포트이름: 최신 전압(V)}
_solar_threads = set()  # 이미 리더를 시작한 포트
_solar_last_print = {}  # 콘솔 출력 throttle용 {포트: 마지막 출력시각}


def _solar_reader(port):
    """해당 포트의 아두이노 시리얼을 계속 읽어 최신 전압을 저장한다."""
    while True:
        try:
            ser = serial.Serial(port, SOLAR_BAUD, timeout=2)
            print(f"태양광 추적기 연결됨: {port}")
            while True:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    try:
                        val = float(line)
                        _solar_voltages[port] = val   # 아두이노가 보내는 전압값
                        now = time.time()             # 2초에 한 번씩 콘솔에 값 출력
                        if now - _solar_last_print.get(port, 0) >= 2:
                            print(f"[태양광] {port} = {val:.2f} V")
                            _solar_last_print[port] = now
                    except ValueError:
                        print(f"[시리얼 알림] 숫자가 아닌 데이터 무시됨: {line}")
                        pass  # 숫자가 아닌 줄(예: 'OLED OK')은 무시
        except Exception as e:
            print(f"태양광 시리얼 오류({port}): {e} — 3초 후 재연결 시도")
            _solar_voltages[port] = None
            time.sleep(3)


def start_solar_reader(port):
    """해당 포트의 리더 스레드를 한 번만 시작한다."""
    if SERIAL_AVAILABLE and port and port not in _solar_threads:
        _solar_threads.add(port)
        Thread(target=_solar_reader, args=(port,), daemon=True).start()


def read_solar_voltage(port, wait_first=True):
    """해당 포트의 최신 태양광 전압(V)을 반환. 못 읽으면 None."""
    if not SERIAL_AVAILABLE or not port:
        return None
    start_solar_reader(port)
    if wait_first:
        for _ in range(20):                # 첫 값이 들어올 때까지 잠깐 대기 (최대 ~2초)
            if _solar_voltages.get(port) is not None:
                break
            time.sleep(0.1)
    return _solar_voltages.get(port)


# ===========================================================================
# 4. 햄스터 하드웨어 실제 구동
#    ENABLE_HARDWARE=False 또는 roboid 미설치 시 시뮬레이션으로 동작.
# ===========================================================================
def execute_hamster_drive(commands):
    """로봇 구동 진입점. ENABLE_HARDWARE=False면 시뮬레이션(명령 출력만)."""
    if not (ENABLE_HARDWARE and ROBOID_AVAILABLE):
        print(f"[시뮬레이션/{DRIVE_MODE}] 전송 예정 명령:", commands)
        return

    try:
        hamster = get_hamster()
        print("햄스터 로봇 연결 성공. 주행을 시작합니다.")

        if DRIVE_MODE == "board":
            _drive_board(hamster, commands)
        else:
            _drive_time(hamster, commands)

        hamster.wheels(0, 0)
        print("구간 주행 완료.")
    except Exception as e:
        print(f"하드웨어 구동 중 에러 발생: {e}")


def _drive_async(commands):
    """실제 주행을 백그라운드에서 실행한다 (경로는 응답으로 화면에 먼저 표시됨).
    이미 주행 중이면 중복 실행을 막는다."""
    if not _drive_lock.acquire(blocking=False):
        print("이미 주행 중 — 이번 주행 명령은 건너뜁니다.")
        return
    try:
        execute_hamster_drive(commands)
    finally:
        _drive_lock.release()


def _drive_via_station_async(commands1, commands2):
    """충전소 경유 주행: 1구간(충전소까지) → 충전 대기(5초) → 2구간(목적지까지)."""
    if not _drive_lock.acquire(blocking=False):
        print("이미 주행 중 — 이번 주행 명령은 건너뜁니다.")
        return
    try:
        print("[주행 1구간] 출발지 → 충전소")
        execute_hamster_drive(commands1)
        do_grip_action()   # 충전소 도착 후 집게 모션
        print("[충전 대기] 충전소 도착. 충전 중... (5초)")
        time.sleep(5)
        print("[주행 2구간] 충전소 → 목적지")
        execute_hamster_drive(commands2)
        print("[완료] 목적지 도착.")
    finally:
        _drive_lock.release()


def do_grip_action():
    """목적지 도착 후 집게 모션: open_gripper → close_gripper."""
    if not (ENABLE_HARDWARE and ROBOID_AVAILABLE):
        print("[시뮬레이션] 집게 동작: 열기 → 닫기(집기)")
        return
    try:
        hamster = get_hamster()
        print("[집게] 열기...")
        hamster.open_gripper()
        wait(500)
        print("[집게] 닫기(집기)...")
        hamster.close_gripper()
        wait(GRIPPER_HOLD_MS)
        print("[집게] 집기 완료.")
    except Exception as e:
        print(f"집게 동작 오류: {e}")


def _drive_board(hamster, commands):
    """[권장] 바닥의 검은 라인을 센서로 인식하며 격자 주행 (시간 보정 불필요).
    board_forward/left/right 가 교차점을 스스로 감지하므로 오차가 누적되지 않는다.

    ※ 햄스터의 board_left/right 는 교차점에서 '제자리 회전'만 한다(이동 X).
      따라서 회전 뒤에는 반드시 board_forward 로 다음 칸까지 이동해야 한다.
      명령(turn/forward)과 동작을 1:1로 매핑한다.
    """
    for cmd in commands:
        if cmd == "forward":
            hamster.board_forward()       # 다음 교차점까지 라인 따라 직진
        elif cmd == "turn_left":
            hamster.board_left()          # 제자리 좌회전 (이동 X)
        elif cmd == "turn_right":
            hamster.board_right()         # 제자리 우회전 (이동 X)
        elif cmd == "turn_back":          # 180도 = 제자리 좌회전 2번
            hamster.board_left()
            hamster.board_left()


def _drive_time(hamster, commands):
    """바퀴를 정해진 시간만큼 굴리는 방식 (calibrate.py로 시간값 보정 필요)."""
    for cmd in commands:
        if cmd == "forward":
            hamster.wheels(FORWARD_SPEED, FORWARD_SPEED)
            wait(FORWARD_TIME)
        elif cmd == "turn_right":
            hamster.wheels(TURN_SPEED, -TURN_SPEED)
            wait(TURN_TIME)
        elif cmd == "turn_left":
            hamster.wheels(-TURN_SPEED, TURN_SPEED)
            wait(TURN_TIME)
        elif cmd == "turn_back":
            hamster.wheels(TURN_SPEED, -TURN_SPEED)
            wait(TURN_TIME * 2)  # 180도 = 90도 두 번

        hamster.wheels(0, 0)  # 각 동작 후 브레이크
        wait(STOP_TIME)


# ===========================================================================
# 라우팅
# ===========================================================================
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/solar')
def solar_status():
    """충전소별 태양광 추적기의 현재 전압. (웹 격자 표시 + 모니터용)
    voltages[i] = i번째 충전소(지도 스캔 순서)의 전압."""
    voltages = [read_solar_voltage(p, wait_first=False) for p in SOLAR_STATION_PORTS]
    return jsonify({"voltages": voltages, "ports": SOLAR_STATION_PORTS})


@app.route('/api/navigate', methods=['POST'])
def navigate():
    data = request.get_json(silent=True) or {}
    grid = data.get('map')
    battery_mode = data.get('battery_mode', 'manual')   # 'manual'=직접 입력, 'auto'=로봇에서 읽기
    battery_value = data.get('battery_value')
    priority = data.get('priority', 'distance')          # 'distance'=거리 우선 / 'voltage'=전압 우선
    start_dir = data.get('start_dir', 'UP')
    if start_dir not in ("UP", "RIGHT", "DOWN", "LEFT"):
        start_dir = "UP"

    if not grid:
        return jsonify({"status": "fail", "message": "지도 데이터가 비어 있습니다."})

    # 배터리 잔량 결정
    if battery_mode == 'auto':
        battery_percent = read_battery_from_robot()
        if battery_percent is None:
            return jsonify({"status": "fail",
                            "message": "로봇에서 배터리를 읽을 수 없습니다(로봇 미연결). '직접 입력'을 사용하세요."})
    else:
        try:
            battery_percent = int(battery_value)
        except (TypeError, ValueError):
            return jsonify({"status": "fail", "message": "배터리 값이 올바르지 않습니다."})

    # 클릭 순서 → 충전소별 포트 매핑 (프론트엔드 stationOrder 와 일치)
    station_order_raw = data.get('station_order', [])
    station_click_order = [tuple(pos) for pos in station_order_raw]

    graph, start, stations, destination = build_graph(grid)

    if start is None:
        return jsonify({"status": "fail", "message": "출발지(파랑)가 지정되지 않았습니다."})
    if destination is None:
        return jsonify({"status": "fail", "message": "목적지(빨강)가 지정되지 않았습니다."})

    # 출발지에서 다익스트라
    distances, prev = dijkstra(graph, start)
    dist_to_dest = distances.get(destination, float('inf'))

    # ─────────────────────────────────────────────────────────────
    # 케이스 1: 배터리가 충분하면 목적지 직행
    # ─────────────────────────────────────────────────────────────
    if dist_to_dest != float('inf') and dist_to_dest * PERCENT_PER_CELL <= battery_percent:
        path = reconstruct_path(prev, destination)
        commands = convert_to_relative_commands(path, start_dir)
        Thread(target=_drive_async, args=(commands,), daemon=True).start()
        return jsonify({
            "status": "success",
            "mode": "direct",
            "target": list(destination),
            "charging_stop": None,
            "path": [list(p) for p in path],
            "commands": commands,
            "shortest_distance": dist_to_dest,
            "battery_percent": battery_percent,
            "battery_source": "로봇 측정" if battery_mode == "auto" else "직접 입력",
            "station_voltage": None,
            "priority_label": "전압 우선" if priority == "voltage" else "거리 우선",
            "energy_needed": dist_to_dest * PERCENT_PER_CELL,
            "percent_per_cell": PERCENT_PER_CELL,
            "hardware": ENABLE_HARDWARE and ROBOID_AVAILABLE,
        })

    # ─────────────────────────────────────────────────────────────
    # 케이스 2: 배터리 부족 → 충전소 경유 탐색
    # ─────────────────────────────────────────────────────────────
    candidates = []   # 배터리로 갈 수 있는 충전소 중 완충 후 목적지까지 이어지는 후보
    for st in stations:
        dist_to_st = distances.get(st, float('inf'))
        if dist_to_st == float('inf') or dist_to_st * PERCENT_PER_CELL > battery_percent:
            continue  # 현재 배터리로 도달 불가

        # 충전 후 100%로 이 충전소에서 목적지까지 경로 탐색
        dist_from_st, prev_from_st = dijkstra(graph, st)
        dist_st_to_dest = dist_from_st.get(destination, float('inf'))
        if dist_st_to_dest != float('inf') and dist_st_to_dest * PERCENT_PER_CELL <= 100:
            candidates.append({
                'station': st,
                'dist_to_st': dist_to_st,
                'dist_st_to_dest': dist_st_to_dest,
                'prev_from_st': prev_from_st,
            })

    if not candidates:
        if dist_to_dest == float('inf'):
            return jsonify({"status": "fail",
                            "message": "목적지까지 연결된 경로가 없습니다. (사방이 막힌 고립 상태)"})
        reachable_sts = [st for st in stations
                         if distances.get(st, float('inf')) * PERCENT_PER_CELL <= battery_percent]
        if not reachable_sts:
            nearest_cost = min(
                (distances.get(st, float('inf')) * PERCENT_PER_CELL for st in stations),
                default=float('inf'))
            return jsonify({
                "status": "fail",
                "message": (f"배터리 부족: 목적지까지 {dist_to_dest * PERCENT_PER_CELL:.0f}% 소모 필요, "
                            f"가장 가까운 충전소까지도 {nearest_cost:.0f}% 필요합니다. (현재 {battery_percent}%)")
            })
        return jsonify({
            "status": "fail",
            "message": "도달 가능한 충전소를 거쳐도 목적지에 도달할 수 없습니다. 경로를 재설정하세요."
        })

    # 충전소 → 포트 매핑 (클릭 순서 우선, 없으면 스캔 순서 fallback)
    def station_port(st):
        if station_click_order and st in station_click_order:
            idx = station_click_order.index(st)
        else:
            idx = stations.index(st)
        return SOLAR_STATION_PORTS[idx] if idx < len(SOLAR_STATION_PORTS) else None

    # 우선순위에 따라 경유 충전소 선택
    station_voltage = None
    if priority == 'voltage':
        for cand in candidates:
            cand['voltage'] = read_solar_voltage(station_port(cand['station']))
        best = max(candidates,
                   key=lambda x: (x.get('voltage') or -1.0,
                                  -(x['dist_to_st'] + x['dist_st_to_dest'])))
        station_voltage = best.get('voltage')
    else:  # 거리 우선: 총 이동 거리 최소화
        best = min(candidates, key=lambda x: x['dist_to_st'] + x['dist_st_to_dest'])

    # 경로 복원
    path1 = reconstruct_path(prev, best['station'])              # 출발지 → 충전소
    path2 = reconstruct_path(best['prev_from_st'], destination)  # 충전소 → 목적지
    full_path = path1 + path2[1:]   # 충전소 좌표 중복 제거

    # 구간별 명령 생성 (2구간 시작 방향 = 1구간 마지막 방향)
    commands1 = convert_to_relative_commands(path1, start_dir)
    dir_at_station = get_final_direction(path1, start_dir)
    commands2 = convert_to_relative_commands(path2, dir_at_station)

    Thread(target=_drive_via_station_async, args=(commands1, commands2), daemon=True).start()

    total_dist = best['dist_to_st'] + best['dist_st_to_dest']
    return jsonify({
        "status": "success",
        "mode": "via_station",
        "target": list(destination),
        "charging_stop": list(best['station']),
        "path": [list(p) for p in full_path],
        "commands": commands1 + commands2,
        "commands_to_station": commands1,
        "commands_to_dest": commands2,
        "shortest_distance": total_dist,
        "dist_to_station": best['dist_to_st'],
        "dist_to_dest": best['dist_st_to_dest'],
        "battery_percent": battery_percent,
        "battery_source": "로봇 측정" if battery_mode == "auto" else "직접 입력",
        "station_voltage": station_voltage,
        "priority_label": "전압 우선" if priority == "voltage" else "거리 우선",
        "energy_needed": best['dist_to_st'] * PERCENT_PER_CELL,
        "percent_per_cell": PERCENT_PER_CELL,
        "hardware": ENABLE_HARDWARE and ROBOID_AVAILABLE,
    })


# ===========================================================================
# 서버 실행 시 기본 브라우저로 웹 화면을 자동으로 연다.
# ===========================================================================
HOST = "127.0.0.1"
PORT = 5000


def open_browser():
    webbrowser.open_new(f"http://{HOST}:{PORT}/")


if __name__ == '__main__':
    # debug=True의 자동 재시작(리로더)은 프로세스를 부모(감시)+자식(실행) 2개로 띄운다.
    # 부모에서만 브라우저를 열어, 코드 저장으로 재시작될 때 탭이 중복으로 열리지 않게 한다.
    if not os.environ.get("WERKZEUG_RUN_MAIN"):
        Timer(1.0, open_browser).start()
    app.run(host=HOST, port=PORT, debug=True)
