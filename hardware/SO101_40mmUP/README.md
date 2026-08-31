# SO101_40mmUP — 40mm 연장형 사양

순정 SO-ARM101의 상완/하완(Upper/Under arm) 길이를 40mm 늘려 가동 범위를 넓힌 변형 버전입니다. CAD 원본(조립체/파트)과 3D 프린트 슬라이서용 파일까지 용도별로 폴더가 나뉘어 있습니다.

## 폴더 구성

| 폴더 | 설명 |
|---|---|
| [`asm/`](./asm/README.md) | SolidWorks 조립체(SLDASM) 파일. 부품이 결합된 상태의 원본 |
| [`step/`](./step/README.md) | 개별 부품 CAD 원본 (STEP/SLDPRT). 순정 부품 + 40mm 연장 부품 포함 |
| [`stl/`](./stl/README.md) | 3D 프린트용 STL(+원본 SLDPRT) 파일 |
| [`p2s_print/`](./p2s_print/README.md) | 슬라이서(3MF)로 바로 출력 가능한 프린트 세트 파일 |

각 폴더 내부는 `leader/`(리더 암), `follower/`(팔로워 암)로 다시 나뉩니다. `arms`/`Base`/`Holder`/`BaseHolder_WristRoll_Mount_Rotation` 등은 40mm 연장에 맞춰 새로 설계된 상완·하완 및 베이스-손목 연결 홀더 부품입니다.
