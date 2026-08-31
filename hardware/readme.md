# hardware

[SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100) 로봇팔을 기반으로, 상완/하완 길이를 40mm 늘려 가동 범위를 넓힌 **SO101_40mmUP** 버전의 하드웨어(3D 프린트 CAD) 파일입니다.

## 폴더 구성

| 폴더 | 설명 |
|---|---|
| [`SO101_40mmUP/`](./SO101_40mmUP/README.md) | 40mm 연장형 로봇팔의 CAD 원본, 조립체, 3D 프린트용 파일 |

`SO101_40mmUP/` 내부는 다시 다음과 같이 나뉩니다.

- `asm/` — SolidWorks 조립체(SLDASM) 파일 (`leader/`, `follower/`)
- `step/` — 개별 부품 CAD 원본 (STEP/SLDPRT) (`leader/`, `follower/`)
- `stl/` — 3D 프린트용 STL/SLDPRT 파일 (`leader/`, `follower/`)
- `p2s_print/` — 슬라이서로 바로 출력 가능한 프린트 세트 파일(3MF)

로봇팔은 **리더(Leader, 조작용)**와 **팔로워(Follower, 실제 동작용)** 한 쌍으로 동작하는 텔레오퍼레이션 구조로, 각 폴더는 `leader/`, `follower/` 하위 폴더로 구분되어 있습니다. 세부 부품 목록과 용도는 각 폴더의 README.md를 참고하세요.
