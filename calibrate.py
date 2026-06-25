# -*- coding: utf-8 -*-
"""
햄스터S 로봇 주행 보정(실측) 도구       [DRIVE_MODE = "time" 전용]
====================================================================
app.py의 두 상수를 실제 로봇으로 측정해 찾는다.
  - FORWARD_TIME : 격자 한 칸을 이동하는 데 걸리는 시간(ms)
  - TURN_TIME    : 정확히 90도 회전하는 데 걸리는 시간(ms)

[사용법]
  1) 블루투스 동글 + 햄스터S 로봇 연결
  2) pip install roboid
  3) 바닥에 '격자 한 칸 크기'(예: 10cm)를 자/테이프로 표시
  4) python calibrate.py  실행 후 메뉴대로 반복 측정
  5) 완료 후 출력된 값을 app.py 의 FORWARD_TIME / TURN_TIME 에 입력
"""
from roboid import HamsterS, wait

# app.py 의 FORWARD_SPEED / TURN_SPEED 와 반드시 같은 값으로 유지할 것!
SPEED = 30

hamster = HamsterS()


def forward(ms):
    print(f"  → 직진 {ms}ms 실행")
    hamster.wheels(SPEED, SPEED)
    wait(ms)
    hamster.wheels(0, 0)
    wait(400)


def turn_right(ms):
    print(f"  → 우회전 {ms}ms 실행")
    hamster.wheels(SPEED, -SPEED)
    wait(ms)
    hamster.wheels(0, 0)
    wait(400)


def ask_int(prompt, default):
    try:
        s = input(prompt).strip()
        return int(s) if s else default
    except ValueError:
        print("  ! 숫자를 입력하세요. 변경 취소.")
        return default


def main():
    forward_time = 1200
    turn_time = 600

    print("=" * 50)
    print(" 햄스터S 주행 보정 도구  (DRIVE_MODE='time' 전용)")
    print(" 바닥에 격자 한 칸 크기(예: 10cm)를 표시해 두세요.")
    print("=" * 50)

    while True:
        print(f"\n[현재값] FORWARD_TIME = {forward_time}ms | TURN_TIME = {turn_time}ms")
        print(" 1) 직진 1칸 테스트        (한 칸 선에 멈추는지 확인)")
        print(" 2) 직진 시간 변경")
        print(" 3) 우회전 90도 테스트     (직각으로 도는지 확인)")
        print(" 4) 회전 4번(360도) 테스트  (제자리로 정확히 오면 OK)")
        print(" 5) 회전 시간 변경")
        print(" 0) 종료 → 최종값 출력")
        sel = input("선택> ").strip()

        if sel == '1':
            forward(forward_time)
        elif sel == '2':
            forward_time = ask_int("새 FORWARD_TIME(ms)> ", forward_time)
        elif sel == '3':
            turn_right(turn_time)
        elif sel == '4':
            print("  4회 회전 시작 (제자리 = 보정 완료, 부족/초과 = 시간 조정)")
            for _ in range(4):
                turn_right(turn_time)
                wait(500)
        elif sel == '5':
            turn_time = ask_int("새 TURN_TIME(ms)> ", turn_time)
        elif sel == '0':
            print("\n" + "=" * 50)
            print(" 측정 완료! app.py 에 아래 두 줄을 그대로 넣으세요:")
            print(f"   FORWARD_TIME = {forward_time}")
            print(f"   TURN_TIME    = {turn_time}")
            print("=" * 50)
            hamster.wheels(0, 0)
            break
        else:
            print("  ! 0~5 중에서 선택하세요.")


if __name__ == '__main__':
    main()
