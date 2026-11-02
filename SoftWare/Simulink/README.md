# 밸런싱로봇 Simulink 모델

[untitled.slx](untitled.slx)는 밸런싱로봇의 물리 모델과 제어·통신 블록을 담은 원본 Simulink 파일이다. MATLAB 함수로 구성한 [MPPI 사전검증 코드](../Mppi_matlab_simulation/)와 함께, 로봇 제어 구조를 확인하는 자료로 사용한다.

모델에는 Simscape Multibody의 관절·강체·접촉 모델, BLDC/PMSM 모터 모델, MATLAB Function 블록, 시리얼 송수신 및 Byte Pack/Unpack 블록이 포함되어 있다. HILS에서는 [ESP32 테스트 펌웨어](../hils_test_code/)와 연결해 사용한다.

## 모델 열기

저장소를 내려받고 MATLAB의 현재 폴더를 이 폴더로 옮긴 뒤 실행한다.

```matlab
open_system('untitled.slx')
```

파일을 더블클릭해 열어도 된다. GitHub에서 파일을 열면 Simulink 편집 화면 대신 다운로드 화면이 나오므로, 내려받은 파일을 MATLAB에서 연다.

## 개발 환경과 저장된 설정

- 저장 버전: MATLAB/Simulink **R2025b**
- 모델의 블록에 필요한 제품: Simulink, Simscape, Simscape Multibody, Simscape Electrical, Instrument Control Toolbox, Embedded Coder
- Solver: `ode1be`
- Fixed step: `0.001 s`
- Stop time: `15 s`
- 시리얼 포트: `COM13`
- 시리얼 통신 속도: `921600 bps`

R2025b에서 모델을 불러오는 검사를 통과했다. 의존 파일 검사에서는 이 모델 외의 별도 사용자 파일이 확인되지 않았다. 위 제품 목록은 모델의 라이브러리 참조를 기준으로 정리했다.

## HILS 연결

`COM13`은 모델을 저장한 PC의 설정이다. HILS를 실행하기 전에 모델 안의 Serial Configuration/Send/Receive 블록에서 실제 USB-to-UART 장치의 포트로 맞춘다. ESP32에도 같은 통신 속도를 사용하는 HILS 펌웨어를 올려야 한다.

모델을 여는 작업과 HILS 시뮬레이션 실행은 별개이다. 이번 업로드에서는 모델 로드와 파일 의존성만 확인했으며, ESP32 연결 및 시뮬레이션 재실행은 하지 않았다.

## 관련 자료

- [전체 제어 구조와 SILS/HILS 설명](../../README.md)
- [HILS 블록 구성 이미지](../../docs/images/hils_matlab_block.png)
- [SILS 테스트 이미지](../../docs/images/sils_test.png)
- [HILS 통신 구조 PDF](../../docs/references/hils_flow.pdf)
- MathWorks 블록 문서: [Serial Configuration](https://www.mathworks.com/help/instrument/serialconfiguration.html), [Byte Pack](https://www.mathworks.com/help/ecoder/ref/bytepack.html)

생성 코드 폴더(`*_ert_rtw`), Simulink 캐시(`slprj`), 백업 파일(`*.slx.original`)은 포함하지 않는다.
