/* ============================================================
   햄스터 로봇 스마트 충전 내비게이션 - 프론트엔드 로직
   - 격자 지도 편집(클릭/드래그)
   - 백엔드 /api/navigate 로 비동기 요청 후 결과 시각화
   셀 값 규약: 0=벽, 1=길, 2=출발지, 3=충전소
   ============================================================ */

let rows = 5;
let cols = 5;
let grid = [];            // 2차원 지도 데이터
let currentTool = 2;      // 현재 선택 도구 (기본: 출발지)
let isPainting = false;   // 드래그 칠하기 상태
let stationVoltages = []; // 충전소 배치 순서대로의 태양광 전압 (/api/solar 폴링)
let stationOrder = [];    // 충전소 배치 순서: [[r,c], [r,c], …] (클릭/드래그 순)

const gridEl = document.getElementById('grid');
const resultEl = document.getElementById('result');

// ---------------------------------------------------------------------------
// 지도 초기화 & 렌더링
// ---------------------------------------------------------------------------
function initGrid(keepData = false) {
    if (!keepData) {
        grid = Array.from({ length: rows }, () => Array(cols).fill(1)); // 전부 '길'
        stationOrder = [];
    }
    renderGrid();
}

function renderGrid() {
    gridEl.style.gridTemplateColumns = `repeat(${cols}, 46px)`;
    gridEl.innerHTML = '';

    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            const cell = document.createElement('div');
            cell.className = 'cell';
            cell.dataset.type = grid[r][c];
            cell.dataset.r = r;
            cell.dataset.c = c;
            applyCellContent(cell);

            cell.addEventListener('mousedown', (e) => {
                e.preventDefault();
                isPainting = true;
                paint(r, c);
            });
            cell.addEventListener('mouseenter', () => {
                if (isPainting) paint(r, c);
            });

            gridEl.appendChild(cell);
        }
    }
}

document.addEventListener('mouseup', () => { isPainting = false; });

function iconFor(type) {
    return { 0: '', 1: '', 2: '🚗', 3: '🔋', 4: '🏁' }[type] || '';
}

// 충전소가 배치된 순서(클릭 순서)에서 몇 번째인지 (SOLAR_STATION_PORTS 인덱스 매칭)
function chargingIndex(r, c) {
    return stationOrder.findIndex(([sr, sc]) => sr === r && sc === c);
}

// 셀 내용 표시: 충전소는 🔋 + 태양광 전압, 그 외는 아이콘만
function applyCellContent(cell) {
    const r = +cell.dataset.r, c = +cell.dataset.c;
    if (grid[r][c] === 3) {
        const v = stationVoltages[chargingIndex(r, c)];
        const txt = (v != null) ? v.toFixed(1) + 'V' : '–';
        cell.innerHTML = `🔋<span class="volt">${txt}</span>`;
    } else {
        cell.textContent = iconFor(grid[r][c]);
    }
}

// ---------------------------------------------------------------------------
// 셀 칠하기
// ---------------------------------------------------------------------------
function paint(r, c) {
    const prevType = grid[r][c];

    // 출발지/목적지는 지도에 단 하나만 존재하도록 보장
    if (currentTool === 2) clearType(2);
    if (currentTool === 4) clearType(4);

    // 충전소 배치 순서 유지
    if (prevType === 3 && currentTool !== 3) {
        // 충전소 → 다른 타입: stationOrder에서 제거
        stationOrder = stationOrder.filter(([sr, sc]) => !(sr === r && sc === c));
    } else if (currentTool === 3 && prevType !== 3) {
        // 새 충전소 배치: stationOrder 끝에 추가
        stationOrder.push([r, c]);
    }

    grid[r][c] = currentTool;
    clearPathHighlight();   // 지도를 수정하면 이전 경로 표시 제거
    updateCell(r, c);
}

function clearType(type) {
    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            if (grid[r][c] === type) {
                grid[r][c] = 1;      // 길로 되돌림
                updateCell(r, c);
            }
        }
    }
}

function updateCell(r, c) {
    const cell = gridEl.querySelector(`.cell[data-r="${r}"][data-c="${c}"]`);
    if (!cell) return;
    cell.dataset.type = grid[r][c];
    applyCellContent(cell);
}

// ---------------------------------------------------------------------------
// 도구 팔레트
// ---------------------------------------------------------------------------
document.querySelectorAll('.tool').forEach(btn => {
    btn.addEventListener('click', () => {
        document.querySelectorAll('.tool').forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        currentTool = parseInt(btn.dataset.type, 10);
    });
});

// ---------------------------------------------------------------------------
// 크기 변경 / 초기화
// ---------------------------------------------------------------------------
document.getElementById('resizeBtn').addEventListener('click', () => {
    rows = clamp(parseInt(document.getElementById('rowsInput').value, 10), 2, 15);
    cols = clamp(parseInt(document.getElementById('colsInput').value, 10), 2, 15);
    initGrid(false);
    hideResult();
});

document.getElementById('clearBtn').addEventListener('click', () => {
    initGrid(false);
    hideResult();
});

function clamp(v, min, max) { return Math.max(min, Math.min(max, v || min)); }

// ---------------------------------------------------------------------------
// 주행 시작 → 백엔드 비동기 통신
// ---------------------------------------------------------------------------
document.getElementById('startBtn').addEventListener('click', startNavigation);

// 배터리 모드 전환: '로봇에서 불러오기' 선택 시 직접 입력 칸 비활성화
document.querySelectorAll('input[name="batteryMode"]').forEach(radio => {
    radio.addEventListener('change', () => {
        const isManual = document.querySelector('input[name="batteryMode"]:checked').value === 'manual';
        document.getElementById('batteryInput').disabled = !isManual;
    });
});

function startNavigation() {
    const batteryMode = document.querySelector('input[name="batteryMode"]:checked').value;
    const batteryValue = parseInt(document.getElementById('batteryInput').value, 10);
    const startDir = document.getElementById('startDirInput').value;
    const priority = document.getElementById('priorityInput').value;
    clearPathHighlight();

    const requestData = {
        battery_mode: batteryMode,
        battery_value: batteryValue,
        priority: priority,
        map: grid,
        start_dir: startDir,
        station_order: stationOrder   // 클릭 순서 → 포트 매핑 일치
    };

    fetch('/api/navigate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(requestData)
    })
        .then(res => res.json())
        .then(data => {
            if (data.status === 'success') {
                highlightPath(data.path, data.charging_stop);
                showResult(true, data);
                const sim = data.hardware ? '' : '\n(시뮬레이션 모드 - 로봇 미연결)';
                if (data.mode === 'direct') {
                    showModal(true, '직행 경로 탐색 완료 · 주행 시작!',
                        `배터리 ${data.battery_percent}%로 목적지까지 ${data.shortest_distance}칸(소모 ${data.energy_needed}%) 직행합니다.${sim}`);
                } else {
                    showModal(true, '충전소 경유 경로 탐색 완료 · 주행 시작!',
                        `배터리 부족으로 직행 불가. 충전소([${data.charging_stop.join(', ')}])를 경유 후 목적지로 이동합니다.${sim}`);
                }
            } else {
                showResult(false, data);
                showModal(false, '주행 실패', data.message);
            }
        })
        .catch(err => {
            showModal(false, '통신 오류', '서버와 통신하지 못했습니다.');
            console.error('Error:', err);
        });
}

// ---------------------------------------------------------------------------
// 결과 시각화
// ---------------------------------------------------------------------------
function highlightPath(path, chargingStop) {
    path.forEach(([r, c], idx) => {
        const cell = gridEl.querySelector(`.cell[data-r="${r}"][data-c="${c}"]`);
        if (!cell) return;
        cell.classList.add('on-path');
        // 중간 경로 칸에 순서 번호 표시 (출발지/충전소/목적지 제외)
        if (grid[r][c] === 1) {
            cell.innerHTML = `<span class="step">${idx}</span>`;
        }
        // 경유 충전소에 ⚡ 마커
        if (chargingStop && r === chargingStop[0] && c === chargingStop[1]) {
            cell.classList.add('waypoint');
        }
    });
}

function clearPathHighlight() {
    gridEl.querySelectorAll('.cell.on-path').forEach(cell => {
        cell.classList.remove('on-path', 'waypoint');
        applyCellContent(cell);
    });
}

const CMD_LABEL = {
    forward: '직진', turn_left: '↺좌회전',
    turn_right: '↻우회전', turn_back: '⤺후진(180°)'
};

function showResult(success, data) {
    resultEl.classList.remove('hidden', 'fail');
    if (success) {
        const seq = data.commands.map(c => CMD_LABEL[c] || c).join(' → ');
        let detail = '';
        if (data.mode === 'direct') {
            detail =
                `<b>✅ 목적지 직행</b> ( ${data.target.join(', ')} )<br>` +
                `배터리: <b>${data.battery_percent}%</b> (${data.battery_source}) · ` +
                `이동: <b>${data.shortest_distance}칸</b> ` +
                `(소모 <b>${data.energy_needed}%</b> @ ${data.percent_per_cell}%/칸) · ` +
                `우선순위: <b>${data.priority_label}</b>`;
        } else {
            detail =
                `<b>⚡ 충전소 경유 → 목적지</b> ( ${data.target.join(', ')} )<br>` +
                `배터리: <b>${data.battery_percent}%</b> (${data.battery_source}) · ` +
                `출발→충전소: <b>${data.dist_to_station}칸</b>(${data.dist_to_station * data.percent_per_cell}%) · ` +
                `충전 후→목적지: <b>${data.dist_to_dest}칸</b>(${data.dist_to_dest * data.percent_per_cell}%) · ` +
                `우선순위: <b>${data.priority_label}</b>` +
                (data.station_voltage != null
                    ? ` · 충전소 전압: <b>${data.station_voltage.toFixed(2)}V</b>` : '');
        }
        resultEl.innerHTML = detail + `<div class="cmd-seq">${seq}</div>`;
    } else {
        resultEl.classList.add('fail');
        resultEl.innerHTML = `<b>❌ 실패</b><br>${data.message}`;
    }
}

function hideResult() { resultEl.classList.add('hidden'); }

// ---------------------------------------------------------------------------
// 모달
// ---------------------------------------------------------------------------
function showModal(success, title, msg) {
    document.getElementById('modalIcon').textContent = success ? '✅' : '⚠️';
    document.getElementById('modalTitle').textContent = title;
    document.getElementById('modalMsg').textContent = msg;
    document.getElementById('modal').classList.remove('hidden');
}
document.getElementById('modalClose').addEventListener('click', () => {
    document.getElementById('modal').classList.add('hidden');
});

// ---------------------------------------------------------------------------
// 충전소 태양광 전압 폴링 → 격자 충전소 셀에 표시
// ---------------------------------------------------------------------------
function refreshStationVoltages() {
    fetch('/api/solar')
        .then(res => res.json())
        .then(data => {
            stationVoltages = data.voltages || [];
            gridEl.querySelectorAll('.cell[data-type="3"]').forEach(applyCellContent);
        })
        .catch(() => {});   // 서버 통신 실패 시 조용히 무시
}
setInterval(refreshStationVoltages, 1500);   // 1.5초마다 갱신

// ---------------------------------------------------------------------------
// 시작
// ---------------------------------------------------------------------------
initGrid(false);
refreshStationVoltages();
