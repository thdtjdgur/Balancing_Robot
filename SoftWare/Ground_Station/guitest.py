import customtkinter as ctk      # UI 디자인 라이브러리 (버튼, 창 예쁘게 만듦)
import tkintermapview            # 지도 위젯
import cv2                       # OpenCV (카메라 영상 처리)
import time                      # 시간 표시용
from PIL import Image, ImageTk   # 이미지 파일 처리
import tkinter as tk             # 기본 UI 도구
import os                        # 파일 경로 잡는 도구
import random                    # [테스트용] 가짜 좌표 만들 때 사용

# 그래프를 위한 추가 라이브러리
from collections import deque
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from keytest import (
    get_link_events,
    queue_waypoints,
    read_ttgo,
    start_ground_link,
    start_ntrip_receiver,
    stop_ground_link,
    stop_ntrip_receiver,
)

# =================================================================
# [1] 경로 자동 보정 (이미지 로드 에러 방지)
# -----------------------------------------------------------------
# 설명: VS Code에서 실행할 때 이미지를 못 찾는 문제를 막기 위해,
#       작업 위치를 현재 파일이 있는 폴더로 강제 고정합니다.
# =================================================================
script_directory = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_directory)
print(f"✅ 작업 폴더 설정 완료: {os.getcwd()}")


# =================================================================
# [2] 설정 (CONFIG) : 여기서 웬만한 숫자는 다 바꾸세요!                                # 딕셔너리 문법(define 모아논 느낌)
# =================================================================
CONFIG = {
    # 1. 지도 설정``
    "API_KEY": "FB928738-CF29-39F6-98F4-85D5D7A93B75",  # V-WORLD API 키
    "HOME_LAT": 37.32066,       # [수정가능] 본부(HQ) 위도
    "HOME_LON": 127.12631,      # [수정가능] 본부(HQ) 경도
    
    # 2. 시스템 설정
    "WIN_SIZE": "1400x900",    # 프로그램 창 크기
    "CAM_INDEX": 0,            # [수정가능] 카메라 번호 (0:노트북캠, 1:USB수신기)
    
    # 3. 색상 테마 (사이버펑크 느낌)
    "COLOR_BG": "#020202",     # 전체 배경 (검정)
    "COLOR_PANEL": "#0A0A0A",  # 패널 배경 (진한 회색)
    "COLOR_CYAN": "#00F0FF",   # 강조색 (형광 하늘색)
    "COLOR_GREEN": "#00FF00",  # 정상 상태 (초록)
    "COLOR_YELLOW": "#FFFF00", # 경로선/GPS로그 (노랑)
    "COLOR_RED": "#FF2A2A",    # 경고/리셋 (빨강)
    "COLOR_ORANGE": "#FFA500", # GPS 로그용 (주황)
    
    # 4. 폰트 설정
    "FONT_MAIN": ("Consolas", 11),          # 일반 글씨
    "FONT_BOLD": ("Consolas", 11, "bold"),  # 굵은 글씨
    "FONT_HEADER": ("Impact", 20),          # 제목 글씨

    #5. 키 구분
    "command" : 0xFF,
    "k_msg" : 0xF1,   #키입력
    "w_msg" : 0xF2    #좌표
}

# 라이브러리 테마 설정 (다크 모드)
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("dark-blue")


# =================================================================
# [3] 메인 프로그램 클래스
# =================================================================
class FinalGCS(ctk.CTk):
    def __init__(self):
        super().__init__()
        
        # 1. 윈도우 창 만들기
        self.title("🛰️ PROJECT: DAESANG_LET'S GO // CONTROL TOWER")
        self.geometry(CONFIG["WIN_SIZE"])
        self.configure(fg_color=CONFIG["COLOR_BG"])
        
        # 2. 화면 나누기 (그리드 설정)
        # column 0: 카메라 화면 (비중 45)
        # column 1: 지도 화면 (비중 45)
        # column 2: 버튼 패널 (비중 10)
        self.grid_columnconfigure(0, weight=45, uniform="main") 
        self.grid_columnconfigure(1, weight=45, uniform="main")
        self.grid_columnconfigure(2, weight=10) 
        
        # row 0: 헤더, row 1: 메인화면, row 2: 로그창
        self.grid_rowconfigure(0, weight=0)
        self.grid_rowconfigure(1, weight=1)
        self.grid_rowconfigure(2, weight=0) 

        # 3. 변수 초기화
        self.waypoint_list = []  # 찍은 점들을 저장할 리스트
        self.path_object = None  # 지도에 그려진 노란 선
        self.drone_marker = None # 드론 아이콘 객체

        # ---------------------------------------------------------
        # 사람 검출 마커 저장용
        # N6에서 detected=1이 들어오면 해당 GPS 위치에 초록색 마커를 찍기 위해 사용
        # detect_events는 지도 다시 그릴 때도 발견 마커를 복구하기 위해 좌표를 저장함
        # ---------------------------------------------------------
        self.detect_events = []          # [(lat, lon, person_count), ...]
        self.detect_event_count = 0

        # ---------------------------------------------------------
        # LoRa RSSI 그래프용 데이터 버퍼
        # ---------------------------------------------------------
        # rssi_times/rssi_values:
        # TTGO에서 50ms마다 보내는 R 라인용
        #
        # packet_times/packet_rssi_values:
        # 드론 GPS 패킷이 실제로 수신된 순간 P 라인용
        # ---------------------------------------------------------
        self.rssi_times = deque(maxlen=1500)
        self.rssi_values = deque(maxlen=1500)

        self.packet_times = deque(maxlen=300)
        self.packet_rssi_values = deque(maxlen=300)
        self.packet_snr_values = deque(maxlen=300)

        self.graph_t0 = None

        # ---------------------------------------------------------
        # 그래프 최적화용 변수
        # matplotlib 그래프는 무거워서, 시리얼을 읽을 때마다 매번 다시 그리면 GUI가 버벅일 수 있음
        # 따라서 0.2초마다 한 번만 그래프를 실제로 다시 그림
        # ---------------------------------------------------------
        self.last_graph_draw_time = 0
        self.graph_draw_period = 0.08

        # 그래프 객체는 _init_map_panel() 안에서 생성됨
        self.signal_fig = None
        self.ax_rssi = None
        self.rssi_line = None
        self.packet_line = None
        self.signal_canvas = None
        self.signal_info_label = None

        # 4. 화면 구성 요소 불러오기
        self._init_header()       # 상단 제목줄
        self._init_camera_panel() # 왼쪽 카메라
        self._init_map_panel()    # 가운데 지도 + 아래쪽 RSSI 그래프
        self._init_data_panel()   # 오른쪽 버튼
        self._init_log_panel()    # 하단 로그창 (2개 분할)

        # 5. 시작 작업
        self.set_home_marker()    # 본부 아이콘 찍기
        self.update_clock()       # 시계 돌리기
        self.log_system("SYSTEM INITIALIZED...")
        self.log_system(f"HQ ESTABLISHED @ {CONFIG['HOME_LAT']}, {CONFIG['HOME_LON']}")   

        # COM 포트를 여는 시점을 import 단계에서 GUI 초기화 이후로 옮긴다.
        # NTRIP와 TTGO 링크는 독립적으로 시작한다. 한쪽 연결에 실패해도
        # 지도/카메라 GUI와 다른 쪽 통신은 계속 사용할 수 있다.
        try:
            start_ntrip_receiver()
        except Exception as e:
            self.log_system(f"⚠️ NTRIP START FAILED: {e}")

        try:
            start_ground_link()
        except Exception as e:
            self.log_system(f"⚠️ TTGO LINK OPEN FAILED: {e}")

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # [중요] GPS 수신 시작
        # 원래는 시뮬레이션 이름이지만, 지금은 TTGO에서 올라오는 실제 GPS/RSSI 라인을 계속 읽는 함수임
        self.simulate_gps_reception()

        # 키보드 입력 받기
        self.key_input()


    # -------------------------------------------------------------
    # [UI 1] 상단 헤더 (제목, 시계)
    # -------------------------------------------------------------
    def _init_header(self):
        header = ctk.CTkFrame(self, height=40, fg_color=CONFIG["COLOR_PANEL"], corner_radius=0)
        header.grid(row=0, column=0, columnspan=3, sticky="ew", padx=1, pady=(0,1))

        ctk.CTkLabel(header, text=" EMERGENCY OPS ", font=CONFIG["FONT_HEADER"], text_color=CONFIG["COLOR_CYAN"]).pack(side="left", padx=15)
        self.clock_label = ctk.CTkLabel(header, text="00:00:00", font=CONFIG["FONT_BOLD"], text_color="white")
        self.clock_label.pack(side="right", padx=15)
        ctk.CTkLabel(header, text=" ● ONLINE ", font=CONFIG["FONT_BOLD"], text_color=CONFIG["COLOR_GREEN"]).pack(side="right")

    # -------------------------------------------------------------
    # [UI 2] 왼쪽 카메라 패널
    # -------------------------------------------------------------
    def _init_camera_panel(self):
        container = ctk.CTkFrame(self, fg_color=CONFIG["COLOR_BG"], corner_radius=0)
        container.grid(row=1, column=0, sticky="nsew", padx=(0,1), pady=0)
        
        ctk.CTkLabel(container, text="[ LIVE VIDEO FEED ]", font=CONFIG["FONT_BOLD"], text_color=CONFIG["COLOR_CYAN"], anchor="w").pack(fill="x", padx=5, pady=(5,2))
        self.video_label = tk.Label(container, bg="black", bd=0)
        self.video_label.pack(expand=True, fill="both")
        
        # 카메라 연결
        self.cap = cv2.VideoCapture(CONFIG["CAM_INDEX"])

        # ---------------------------------------------------------
        # 카메라 최적화
        # 기본 노트북캠이 1080p로 잡히면 RGB 변환/리사이즈 비용이 커져서 GUI가 버벅일 수 있음
        # 전시용 GUI 화면에서는 640x480 정도면 충분해서 해상도와 FPS를 제한함
        # ---------------------------------------------------------
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.update_video() # 영상 재생 시작

    # -------------------------------------------------------------
    # [UI 3] 중앙 지도 패널 (V-WORLD)
    # -------------------------------------------------------------
    def _init_map_panel(self):
        container = ctk.CTkFrame(self, fg_color=CONFIG["COLOR_PANEL"], corner_radius=0)
        container.grid(row=1, column=1, sticky="nsew", padx=(1,1), pady=0)

        # 지도 영역 안에서
        # row 0: 제목
        # row 1: 지도
        # row 2: 그래프 제목/상태
        # row 3: RSSI 그래프
        container.grid_columnconfigure(0, weight=1)
        container.grid_rowconfigure(0, weight=0)
        container.grid_rowconfigure(1, weight=7)
        container.grid_rowconfigure(2, weight=0)
        container.grid_rowconfigure(3, weight=3)
        
        ctk.CTkLabel(container, text="[ RESPONSE MAP : V-WORLD ]", font=CONFIG["FONT_BOLD"], text_color=CONFIG["COLOR_CYAN"], anchor="w").grid(row=0, column=0, sticky="ew", padx=5, pady=(5,2))

        # 지도 위젯 생성
        self.map_widget = tkintermapview.TkinterMapView(container, corner_radius=0)
        self.map_widget.grid(row=1, column=0, sticky="nsew", padx=0, pady=0)

        # 줌 버튼(+,-) 숨기기 (깔끔하게 보이기 위해)
        try:
            self.map_widget.canvas.itemconfigure(self.map_widget.button_zoom_in.canvas_rect, state='hidden')
            self.map_widget.canvas.itemconfigure(self.map_widget.button_zoom_in.canvas_text, state='hidden')
            self.map_widget.canvas.itemconfigure(self.map_widget.button_zoom_out.canvas_rect, state='hidden')
            self.map_widget.canvas.itemconfigure(self.map_widget.button_zoom_out.canvas_text, state='hidden')
        except: 
            pass

        # [V-WORLD 서버 설정]
        key = CONFIG["API_KEY"]
        vworld_url = f"http://api.vworld.kr/req/wmts/1.0.0/{key}/Satellite/{{z}}/{{y}}/{{x}}.jpeg"
        self.map_widget.set_tile_server(vworld_url, max_zoom=19)
        
        # 초기 위치로 이동
        self.map_widget.set_position(CONFIG["HOME_LAT"], CONFIG["HOME_LON"])
        self.map_widget.set_zoom(17)
        self.map_widget.add_left_click_map_command(self.add_waypoint) # 클릭 시 웨이포인트 추가 기능 연결

        # ---------------------------------------------------------
        # 지도 영역 아래쪽에 LoRa RSSI 그래프 추가
        # ---------------------------------------------------------
        graph_header = ctk.CTkFrame(container, fg_color="#111111", corner_radius=0, height=25)
        graph_header.grid(row=2, column=0, sticky="ew", padx=0, pady=(1, 0))
        graph_header.grid_columnconfigure(0, weight=1)

        ctk.CTkLabel(
            graph_header,
            text="  [ LORA SIGNAL GRAPH : DRONE GPS RSSI ]",
            height=25,
            font=CONFIG["FONT_BOLD"],
            text_color=CONFIG["COLOR_CYAN"],
            anchor="w"
        ).grid(row=0, column=0, sticky="ew")

        self.signal_info_label = ctk.CTkLabel(
            graph_header,
            text="RSSI: -- dBm | PKT: -- dBm | SNR: -- dB",
            height=25,
            font=CONFIG["FONT_MAIN"],
            text_color="white",
            anchor="e"
        )
        self.signal_info_label.grid(row=0, column=1, sticky="e", padx=8)

        graph_frame = ctk.CTkFrame(container, fg_color="black", corner_radius=0)
        graph_frame.grid(row=3, column=0, sticky="nsew", padx=0, pady=0)
        graph_frame.grid_columnconfigure(0, weight=1)
        graph_frame.grid_rowconfigure(0, weight=1)

        self._init_signal_graph(graph_frame)

    # -------------------------------------------------------------
    # [UI 3-1] LoRa RSSI 그래프 초기화
    # -------------------------------------------------------------
    def _init_signal_graph(self, parent):
        self.signal_fig = Figure(figsize=(5, 2.2), dpi=100)
        self.signal_fig.patch.set_facecolor("#020202")

        self.ax_rssi = self.signal_fig.add_subplot(111)
        self.ax_rssi.set_facecolor("#050505")
        self.ax_rssi.set_title("LoRa RSSI Monitor", color="white", fontsize=9)
        self.ax_rssi.set_xlabel("Time (s)", color="white", fontsize=8)
        self.ax_rssi.set_ylabel("RSSI (dBm)", color="white", fontsize=8)
        self.ax_rssi.tick_params(axis="x", colors="white", labelsize=7)
        self.ax_rssi.tick_params(axis="y", colors="white", labelsize=7)
        self.ax_rssi.grid(True, alpha=0.25)

        # R 라인 기반 배경 RSSI
        self.rssi_line, = self.ax_rssi.plot(
            [],
            [],
            linewidth=1.2,
            label="Current RSSI"
        )

        # P 라인 기반 GPS 패킷 수신 순간 RSSI
        self.packet_line, = self.ax_rssi.plot(
            [],
            [],
            linestyle="none",
            marker="o",
            markersize=5,
            label="GPS Packet RSSI"
        )

        self.ax_rssi.legend(loc="lower right", fontsize=7)

        self.signal_canvas = FigureCanvasTkAgg(self.signal_fig, master=parent)
        self.signal_canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    # -------------------------------------------------------------
    # [UI 4] 오른쪽 버튼 패널
    # -------------------------------------------------------------
    def _init_data_panel(self):
        container = ctk.CTkFrame(self, fg_color=CONFIG["COLOR_PANEL"], corner_radius=0, width=200)
        container.grid(row=1, column=2, sticky="nsew", padx=(1,0), pady=0)

        # 상태 게이지
        ctk.CTkLabel(container, text="[ STATUS ]", font=CONFIG["FONT_BOLD"], text_color="white").pack(pady=(15,10))
        self._make_gauge(container, "BATTERY", CONFIG["COLOR_CYAN"], 0.88)
        self._make_gauge(container, "SIGNAL", "#FFA500", 0.75)
        self._make_gauge(container, "THRUST", CONFIG["COLOR_RED"], 0.0)

        ctk.CTkLabel(container, text="[ MISSION CTRL ]", font=CONFIG["FONT_BOLD"], text_color="white").pack(pady=(30,10))
        
        # 편집 모드 스위치
        self.wp_mode_switch = ctk.CTkSwitch(container, text="EDIT MODE", font=CONFIG["FONT_BOLD"], 
                                            progress_color=CONFIG["COLOR_GREEN"], text_color="white")
        self.wp_mode_switch.pack(pady=10)
        self.wp_mode_switch.select()

        # [버튼] 전송 (아래쪽 여백 30px 줘서 띄움)
        ctk.CTkButton(container, text="📡 SEND POINT", command=self.send_mission,
                      fg_color="#0055FF", hover_color="#0033CC", font=CONFIG["FONT_BOLD"]).pack(fill="x", padx=10, pady=(5, 30))

        # [버튼] 되돌리기
        ctk.CTkButton(container, text="↩ UNDO WP", command=self.undo_waypoint,
                      fg_color="#444", hover_color="#666", border_color="#666", border_width=1, font=CONFIG["FONT_BOLD"]).pack(fill="x", padx=10, pady=5)

        # [버튼] 초기화
        ctk.CTkButton(container, text="🔄 RESET", command=self.reset_mission, 
                      fg_color="#330000", hover_color="red", border_color="red", border_width=1,font=CONFIG["FONT_BOLD"]).pack(fill="x", padx=10, pady=5) 

    def _make_gauge(self, parent, title, color, val):
        f = ctk.CTkFrame(parent, fg_color="transparent")
        f.pack(fill="x", padx=10, pady=8)
        ctk.CTkLabel(f, text=title, font=CONFIG["FONT_MAIN"], text_color="gray").pack(side="left")
        prog = ctk.CTkProgressBar(f, height=6, progress_color=color)
        prog.set(val)
        prog.pack(fill="x", pady=2)

    # -------------------------------------------------------------
    # [UI 5] 하단 로그 패널 (SYSTEM / GPS / RTCM 3개로 쪼개기)
    # -------------------------------------------------------------
    def _init_log_panel(self):
        # 전체 로그 컨테이너
        log_container = ctk.CTkFrame(self, fg_color=CONFIG["COLOR_PANEL"], corner_radius=0, height=120)
        log_container.grid(row=2, column=0, columnspan=3, sticky="nsew", padx=0, pady=(1,0))
        
        # 비율 설정 (시스템 4 : GPS 3 : RTCM 3)
        log_container.grid_columnconfigure(0, weight=4)
        log_container.grid_columnconfigure(1, weight=3)
        log_container.grid_columnconfigure(2, weight=3)
        log_container.grid_rowconfigure(0, weight=0) # 제목줄 (높이 고정됨)
        log_container.grid_rowconfigure(1, weight=1) # 내용줄

        # -----------------------------------------------------------
        # [수정] 제목을 '라벨'이 아니라 '제목 바' 형태로 변경 (높이 고정)
        # height=25, fg_color="#222" (배경색 추가) -> 이렇게 하면 높이가 무조건 맞음
        # -----------------------------------------------------------
        
        # 1. 왼쪽: 시스템 로그 제목
        ctk.CTkLabel(log_container, text="  [ SYSTEM LOG ]", height=25, 
                     fg_color="#222222", # 제목 배경색 (진한 회색)
                     font=CONFIG["FONT_BOLD"], text_color=CONFIG["COLOR_GREEN"], anchor="w").grid(row=0, column=0, sticky="ew", padx=(0,1), pady=0)
        
        # 1-2. 왼쪽 내용창
        self.sys_log_box = ctk.CTkTextbox(log_container, fg_color="black", text_color=CONFIG["COLOR_GREEN"], font=CONFIG["FONT_MAIN"], corner_radius=0)
        self.sys_log_box.grid(row=1, column=0, sticky="nsew", padx=(0,1), pady=0)
        self.sys_log_box.configure(state="disabled")

        # 2. 가운데: GPS 로그 제목
        ctk.CTkLabel(log_container, text="  [ REAL-TIME GPS ]", height=25, 
                     fg_color="#222222", # 제목 배경색 (진한 회색)
                     font=CONFIG["FONT_BOLD"], text_color=CONFIG["COLOR_ORANGE"], anchor="w").grid(row=0, column=1, sticky="ew", padx=0, pady=0)
        
        # 2-2. 오른쪽 내용창
        self.gps_log_box = ctk.CTkTextbox(log_container, fg_color="black", text_color=CONFIG["COLOR_ORANGE"], font=CONFIG["FONT_MAIN"], corner_radius=0)
        self.gps_log_box.grid(row=1, column=1, sticky="nsew", padx=(0,1), pady=0)
        self.gps_log_box.configure(state="disabled")

        # 3. 오른쪽: NTRIP/RTCM 전용 로그 제목
        ctk.CTkLabel(log_container, text="  [ RTCM / NTRIP ]", height=25,
                     fg_color="#222222",
                     font=CONFIG["FONT_BOLD"], text_color=CONFIG["COLOR_CYAN"],
                     anchor="w").grid(row=0, column=2, sticky="ew", padx=0, pady=0)

        # NTRIP 연결, 최신 RTCM 1개 선택, 2초 주기 송신 로그만 이 창에 출력한다.
        self.rtcm_log_box = ctk.CTkTextbox(
            log_container,
            fg_color="black",
            text_color=CONFIG["COLOR_CYAN"],
            font=CONFIG["FONT_MAIN"],
            corner_radius=0,
        )
        self.rtcm_log_box.grid(row=1, column=2, sticky="nsew", padx=0, pady=0)
        self.rtcm_log_box.configure(state="disabled")

    # -------------------------------------------------------------
    # [기능] 로그 출력 함수들
    # -------------------------------------------------------------
    def log_system(self, msg): # 시스템 로그 / 왼쪽 창에 출력
        self.sys_log_box.configure(state="normal")
        ts = time.strftime("[%H:%M:%S]")
        self.sys_log_box.insert("end", f"{ts} > {msg}\n")
        self.sys_log_box.see("end")
        self.sys_log_box.configure(state="disabled")

    def log_rtcm(self, msg):
        """NTRIP 접속과 RTCM 생성/송신 로그를 오른쪽 전용 창에 출력한다."""
        self.rtcm_log_box.configure(state="normal")
        ts = time.strftime("[%H:%M:%S]")
        self.rtcm_log_box.insert("end", f"{ts} > {msg}\n")
        self.rtcm_log_box.see("end")
        self.rtcm_log_box.configure(state="disabled")


################################### 드론 + GPS 관련

    def log_gps(self, lat, lon, psi=None, v=None, w=None, quality=None, waypoint_target=None): # 오른쪽 창에 드론 GPS 위치 로그 출력
        self.gps_log_box.configure(state="normal")
        ts = time.strftime("[%H:%M:%S]")
        telemetry_text = ""
        if quality is not None:
            telemetry_text += f" Q:{quality}"
        if waypoint_target is not None:
            telemetry_text += f" WP:{waypoint_target}"
        if psi is not None:
            telemetry_text += f" PSI:{psi:.3f}rad"
        gps_text = f"{ts} | LAT:{lat:.2f} LON:{lon:.2f}{telemetry_text}"
        command_text = ""
        if v is not None:
            command_text += f" V:{v:.3f}m/s"
        if w is not None:
            command_text += f" W:{w:.3f}rad/s"
        if command_text:
            gps_text += f"\n           {command_text.strip()}"

        self.gps_log_box.insert("end", f"{gps_text}\n\n")
        self.gps_log_box.see("end")
        self.gps_log_box.configure(state="disabled")
    

    # -------------------------------------------------------------
    # [기능] 드론 마커(아이콘) 업데이트
    # -------------------------------------------------------------
    def update_drone_marker(self, lat, lon):
        # 마커가 없으면 새로 만듭니다.
        if self.drone_marker is None:
            try:
                # DRONE.png 이미지를 불러옵니다.
                img = Image.open("DRONE.png").resize((40, 40), Image.Resampling.LANCZOS)
                self.drone_icon_img = ImageTk.PhotoImage(img) # 가비지 컬렉션 방지용 변수 저장
                
                # 지도에 아이콘 생성 (색상 변경 부분)
                self.drone_marker = self.map_widget.set_marker(
                    lat, lon, 
                    text="UAV", 
                    icon=self.drone_icon_img,
                    text_color="#FFA500",         # <--- [수정] 글자 색상 (yellow, white, #00F0FF 등)
                    font=("Bahnschrift", 20)          # <--- [추천] 글자 폰트랑 크기도 키우면 더 잘 보임
                )
                self.log_system("LOG: Drone Marker Created.")
            except:
                # 이미지가 없으면 빨간 핀으로 대체
                self.drone_marker = self.map_widget.set_marker(
                    lat, lon, 
                    text="UAV", 
                    marker_color_outside="red",
                    text_color="red",         # <--- [수정] 여기도 색상 추가
                    font=("Impact", 12)
                )
        
        # 이미 마커가 있으면 위치만 쇽! 옮깁니다.
        self.drone_marker.set_position(lat, lon)


    # -------------------------------------------------------------
    # [기능] 사람 검출 마커 추가
    # -------------------------------------------------------------
    def add_detect_marker(self, lat, lon, person_count):
        self.detect_event_count += 1

        # 나중에 지도 다시 그릴 때 검출 마커도 복구하기 위해 저장
        self.detect_events.append((lat, lon, person_count))

        self.map_widget.set_marker(
            lat,
            lon,
            text=f"FOUND {person_count}",
            marker_color_outside=CONFIG["COLOR_GREEN"],
            marker_color_circle=CONFIG["COLOR_GREEN"],
            text_color=CONFIG["COLOR_GREEN"],
            font=("Bahnschrift", 14, "bold")
        )

        self.log_system(
            f"🟢 PERSON DETECTED: count={person_count} @ LAT:{lat:.5f}, LON:{lon:.5f}"
        )


    # -------------------------------------------------------------
    # [기능] 사람 검출 마커 다시 그리기
    # -------------------------------------------------------------
    def redraw_detect_markers(self):
        for lat, lon, person_count in self.detect_events:
            self.map_widget.set_marker(
                lat,
                lon,
                text=f"FOUND {person_count}",
                marker_color_outside=CONFIG["COLOR_GREEN"],
                marker_color_circle=CONFIG["COLOR_GREEN"],
                text_color=CONFIG["COLOR_GREEN"],
                font=("Bahnschrift", 14, "bold")
            )


    # -------------------------------------------------------------
    # [기능] LoRa RSSI 그래프 업데이트
    # -------------------------------------------------------------
    def update_signal_graph(self):
        # 아직 그래프가 만들어지기 전이면 아무것도 안 함
        if self.ax_rssi is None or self.signal_canvas is None:
            return

        # R 라인 기반 배경 RSSI 선
        self.rssi_line.set_data(list(self.rssi_times), list(self.rssi_values))

        # P 라인 기반 GPS 패킷 수신 순간 점
        self.packet_line.set_data(list(self.packet_times), list(self.packet_rssi_values))

        all_times = list(self.rssi_times) + list(self.packet_times)
        all_rssi = list(self.rssi_values) + list(self.packet_rssi_values)

        if all_times:
            t_max = max(all_times)
            t_min = max(0, t_max - 10)  # 최근 10초만 표시
            self.ax_rssi.set_xlim(t_min, t_max + 0.1)

        if all_rssi:
            y_min = min(all_rssi) - 5
            y_max = max(all_rssi) + 5

            if y_min == y_max:
                y_min -= 1
                y_max += 1

            self.ax_rssi.set_ylim(y_min, y_max)

        self.signal_canvas.draw_idle()


    # -------------------------------------------------------------
    # [중요] GPS 수신 시뮬레이션 (1초마다 실행됨)
    # -------------------------------------------------------------
    def simulate_gps_reception(self):
    
    #TTGO에서 수신한 드론 GPS 좌표를 읽어서 지도에 표시한다.
    #read_ttgo()는 새 GPS 패킷이 있으면 (lat, lon)을 반환하고,
    #아직 데이터가 없으면 None을 반환한다.

        # 백그라운드 통신 메시지를 종류에 따라 SYSTEM과 RTCM 전용 창으로 분리한다.
        for event_message in get_link_events():
            if event_message.startswith(("[NTRIP]", "[RTCM]", "[RTCM TX]")):
                self.log_rtcm(event_message)
            else:
                self.log_system(event_message)

        # [수정]
        # 이제 read_ttgo()는 단순히 (lat, lon)을 반환하지 않고,
        # TTGO에서 올라오는 R/P 라인을 구분해서 딕셔너리로 반환한다.
        #
        # R 라인:
        # {"type": "rssi", "time_ms": ..., "rssi": ...}
        #
        # P 라인:
        # {"type": "gps", "time_ms": ..., "packet_rssi": ..., "snr": ..., "lat": ..., "lon": ...}
        #
        # 한 번 호출될 때 시리얼에 여러 줄이 쌓여 있을 수 있으므로,
        # 최대 15줄까지 연속으로 처리한다.
        # 기존 30줄은 한 번에 너무 많이 처리하면 Tkinter 메인루프가 잠깐 잡힐 수 있어서 줄임
        for _ in range(15):
            msg = read_ttgo() # ttgo에서 보낸값 읽음

            if msg is None:
                break

            msg_type = msg.get("type")

            # -----------------------------------------------------
            # R 라인 처리
            # TTGO가 50ms마다 보내는 현재 RSSI 값
            # 지도 아래쪽 RSSI 그래프의 선으로 표시됨
            # -----------------------------------------------------
            if msg_type == "rssi":
                t_ms = msg["time_ms"]

                if self.graph_t0 is None:
                    self.graph_t0 = t_ms

                t_sec = (t_ms - self.graph_t0) / 1000.0

                self.rssi_times.append(t_sec)
                self.rssi_values.append(msg["rssi"])

                if self.signal_info_label is not None:
                    self.signal_info_label.configure(
                        text=f"RSSI: {msg['rssi']} dBm | PKT: -- dBm | SNR: -- dB"
                    )

            # -----------------------------------------------------
            # P 라인 처리
            # 드론 GPS 패킷이 실제로 수신된 순간
            # 지도 마커 업데이트 + 패킷 RSSI 점 표시
            # -----------------------------------------------------
            elif msg_type == "gps":
                t_ms = msg["time_ms"]

                if self.graph_t0 is None:
                    self.graph_t0 = t_ms

                t_sec = (t_ms - self.graph_t0) / 1000.0

                lat = msg["lat"]
                lon = msg["lon"]
                psi = msg.get("psi")
                v = msg.get("v")
                w = msg.get("w")
                quality = msg.get("quality")
                waypoint_target = msg.get("waypoint_target")

                # N6 사람 검출 정보
                # keytest.py의 read_ttgo()에서 detected/person_count를 같이 넘겨줌
                detected = msg.get("detected", 0)
                person_count = msg.get("person_count", 0)

                self.log_gps(lat, lon, psi, v, w, quality, waypoint_target)
                self.update_drone_marker(lat, lon)
                if psi is not None or v is not None or w is not None:
                    quality_text = f", quality={quality}" if quality is not None else ""
                    waypoint_text = f", wp={waypoint_target}" if waypoint_target is not None else ""
                    psi_text = f", psi={psi:.4f} rad" if psi is not None else ""
                    v_text = f", v={v:.4f} m/s" if v is not None else ""
                    w_text = f", w={w:.4f} rad/s" if w is not None else ""
                    self.log_system(
                        f"[GUI] position updated: lat={lat:.7f}, lon={lon:.7f}{psi_text}{v_text}{w_text}{quality_text}{waypoint_text}"
                    )
                else:
                    self.log_system(
                        f"[GUI] position updated: lat={lat:.7f}, lon={lon:.7f}"
                    )

                # 사람이 새로 검출된 이벤트면 현재 드론 GPS 위치에 초록색 마커 표시
                # 드론 쪽에서 중복 방지 처리해서 detected=1은 이벤트성으로만 들어오는 구조
                if detected == 1 and person_count > 0:
                    self.add_detect_marker(lat, lon, person_count)

                self.packet_times.append(t_sec)
                self.packet_rssi_values.append(msg["packet_rssi"])
                self.packet_snr_values.append(msg["snr"])

                if self.signal_info_label is not None:
                    self.signal_info_label.configure(
                        text=f"RSSI: {msg['packet_rssi']} dBm | PKT: {msg['packet_rssi']} dBm | SNR: {msg['snr']:.2f} dB"
                    )

                # 필요하면 시스템 로그에도 패킷 수신 상태 출력 가능
                # self.log_system(f"GPS RX RSSI={msg['packet_rssi']} dBm, SNR={msg['snr']:.2f} dB")

            # 향후 드론이 Waypoint ACK를 보내면 TTGO가
            # A,millis,waypoint_id,ok 라인으로 올려준다.
            elif msg_type == "waypoint_ack":
                if msg.get("ok"):
                    self.log_system(f"WAYPOINT ACK: WP#{msg['command_id']} RECEIVED")
                else:
                    self.log_system(f"⚠️ WAYPOINT NACK: WP#{msg['command_id']}")

        # ---------------------------------------------------------
        # 그래프 최적화
        # 시리얼 수신은 30ms마다 계속 처리하되,
        # matplotlib 그래프는 무거우므로 0.2초마다만 다시 그림
        # ---------------------------------------------------------
        now = time.time()
        if now - self.last_graph_draw_time >= self.graph_draw_period:
            self.last_graph_draw_time = now
            self.update_signal_graph()

        self.after(30, self.simulate_gps_reception)



    def send_mission(self): 
        #######################################################################################  
        # [수정 구역] 웨이포인트 좌표 전송 코드 넣는 곳
        ######################################################################################
        
        if not self.waypoint_list:                        # 웨이포인트 안찍었을때
            self.log_system("⚠️ ERROR: NO WAYPOINTS")
            return
        
        # 버튼을 누른 시점에 바로 LoRa로 보내지 않는다.
        # KEYTEST의 pending Waypoint로 등록하고 다음 2초 RTCM TX 구간에서 함께 전송한다.
        try:
            command_id = queue_waypoints(self.waypoint_list)
        except Exception as e:
            self.log_system(f"⚠️ WAYPOINT QUEUE ERROR: {e}")
            return

        self.log_system(
            f"UPLINK: WP#{command_id} QUEUED ({len(self.waypoint_list)} points)"
        )
        self.log_system("UPLINK: WILL SEND IN NEXT RTCM TX PHASE")




    ################################################################################## 키보드 입력

    def key_input(self):
        self.bind("<Key>", self.key_control) 
         #tkinter 에 이벤트를 감지하면 뭘 실행해라 라는 bind함수가있음, 그리고 tkinter가 미리 준비해 둔 키 입력 이벤트 이름인 Key가 있음
         #그리고 key로 받은 값을 뒤에 인자 함수에 매개변수로 자동으로 넘겨줌, 그래서 아래서 콜백함수에 그 키 입력값을 받을 event라는 구조체
       
    def key_control(self,event):
        #이제 내가 키를 누르면 뭐 키코드,그 키가 뭔지 등등이 event 클래스 안에 변수에 저장되있고 
        #그 event.keysym 이라는 keysym안에 내가 누른 키의 문자가 뭔지 문자열로 저장됨(tkinter가 정의한 각 키의 명칭) --> space바 키 누르면 keysym에 space 문자열이 저장됨 --> 그래서 읽을때 한개씩 읽으면 s로  
        # 이제 그거를 lower이라는 파이썬 내장 함수를 이용해서 소문자로 바꾼다
        key = event.keysym.lower()

        if key not in ['w','a','s','d']:
            self.log_system("others")
            print("others")
            return
        
        else:
            # 새 반이중 통신의 downlink는 RTCM + Waypoint로 정의되어 있다.
            # W/A/S/D를 즉시 송신하면 2초 사이클을 깨뜨리므로 현재는 전송하지 않는다.
            self.log_system(f"MANUAL KEY '{key}' BLOCKED: SCHEDULED RTK MODE")

        
            


        






    # -------------------------------------------------------------
    # [기능] 본부 아이콘 설정
    # -------------------------------------------------------------
    def set_home_marker(self):
        try:
            icon_img = Image.open("TOWER.png").resize((50, 50), Image.Resampling.LANCZOS) ########## 컨트롤 타워 이미지
            self.hq_icon_img = ImageTk.PhotoImage(icon_img)
            self.map_widget.set_marker(CONFIG["HOME_LAT"], CONFIG["HOME_LON"], text="◈ COMMAND",
                                       text_color="#FF2A2A", font=("Impact",20), icon=self.hq_icon_img)
            self.log_system("Custom HQ Icon Loaded (TOWER.png).")
        except Exception as e:
            self.map_widget.set_marker(CONFIG["HOME_LAT"], CONFIG["HOME_LON"], text="◈ COMMAND",
                                       text_color=CONFIG["COLOR_CYAN"], font="FONT_BOLD", marker_color_outside=CONFIG["COLOR_CYAN"])

    # -------------------------------------------------------------
    # [기능] 기타 유틸리티 (시계, 카메라)
    # -------------------------------------------------------------
    def update_clock(self):
        self.clock_label.configure(text=time.strftime("%H:%M:%S"))
        self.after(1000, self.update_clock)

    def update_video(self):
        if self.cap is None or not self.cap.isOpened():
            self._reconnect_camera(); self.after(500, self.update_video); return
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            w = self.video_label.winfo_width()
            h = self.video_label.winfo_height()
            if w > 10:
                ratio = min(w / img.width, h / img.height)
                img = img.resize((int(img.width * ratio), int(img.height * ratio)), Image.Resampling.LANCZOS)
                imgtk = ImageTk.PhotoImage(image=img)
                self.video_label.config(image=imgtk, bg="black"); self.video_label.imgtk = imgtk 

            # -----------------------------------------------------
            # 카메라 최적화
            # 기존 10ms는 거의 100FPS로 화면을 갱신하려는 수준이라 GUI가 버벅일 수 있음
            # 33ms는 약 30FPS라 체감은 부드럽고 부하는 훨씬 줄어듦
            # -----------------------------------------------------
            self.after(20, self.update_video)
        else:
            self.log_system("⚠️ SIGNAL LOST"); self.cap.release(); self._reconnect_camera(); self.after(500, self.update_video)

    def _reconnect_camera(self):
        try:
            self.cap = cv2.VideoCapture(CONFIG["CAM_INDEX"])
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except: 
            pass

    # -------------------------------------------------------------
    # [미션] 웨이포인트 추가/삭제/전송/리셋
    # -------------------------------------------------------------
    def add_waypoint(self, coords):
        if self.wp_mode_switch.get() == 0: return
        self.waypoint_list.append(coords)
        idx = len(self.waypoint_list)
        self.map_widget.set_marker(coords[0], coords[1], text=f"WP{idx}")
        self.log_system(f"CMD: ADDED WP{idx} @ [{coords[0]:.5f}, {coords[1]:.5f}]")
        if len(self.waypoint_list) > 1:
            if self.path_object: self.map_widget.delete_all_path()
            self.path_object = self.map_widget.set_path(self.waypoint_list, color=CONFIG["COLOR_YELLOW"], width=5)

    def undo_waypoint(self):
        if self.waypoint_list:
            self.waypoint_list.pop(); self._redraw_map(); self.log_system(f"CMD: UNDO LAST WP")
        else: self.log_system(" NO WAYPOINTS TO UNDO")

    def reset_mission(self):
        self.waypoint_list.clear(); self.map_widget.delete_all_marker(); self.map_widget.delete_all_path()
        self.path_object = None; self.set_home_marker()
        self.drone_marker = None # 리셋 시 드론 마커도 지울지 선택 (여기선 지움)

        # 사람 검출 마커 정보도 초기화
        self.detect_events.clear()
        self.detect_event_count = 0

        self.log_system("CMD: MISSION RESET (HQ RESTORED)")

    def _redraw_map(self):
        self.map_widget.delete_all_marker(); self.map_widget.delete_all_path(); self.set_home_marker()
        for i, coords in enumerate(self.waypoint_list): self.map_widget.set_marker(coords[0], coords[1], text=f"WP{i+1}")
        if len(self.waypoint_list) > 1: self.path_object = self.map_widget.set_path(self.waypoint_list, color=CONFIG["COLOR_YELLOW"], width=3)

        # 지도 다시 그릴 때 사람 검출 마커도 같이 복구
        self.redraw_detect_markers()

    def on_close(self):
        # 창을 닫을 때 백그라운드 NTRIP 수신과 2초 TTGO 스케줄러를 먼저 종료한다.
        # 그 다음 카메라 자원을 해제해야 종료 후에도 통신 스레드가 남지 않는다.
        stop_ntrip_receiver()
        stop_ground_link()
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()
        self.destroy()

    def __del__(self):
        if hasattr(self, 'cap') and self.cap is not None and self.cap.isOpened():
            self.cap.release()

if __name__ == "__main__":
    app = FinalGCS()
    app.mainloop()
