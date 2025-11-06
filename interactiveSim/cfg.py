from dataclasses import dataclass

# =========================
# Config & Constants
# =========================
@dataclass
class Cfg:
    # Display
    W: int = 1000
    H: int = 700
    FPS: int = 60

    # Colors
    BG_BLUE: tuple = (11, 53, 83)
    ROAD_DARK: tuple = (9, 43, 69)
    PALE: tuple = (230, 233, 238)
    PALE2: tuple = (210, 255, 220)
    GRID: tuple = (180, 190, 198)
    EGO_COLOR: tuple = (80, 200, 255)
    EGO_ACCENT: tuple = (60, 160, 220)
    SHADOW: tuple = (0, 0, 0, 60)
    BTN_BG: tuple = (25, 90, 130)
    BTN_BORDER: tuple = (160, 190, 210)
    BTN_TEXT: tuple = (230, 233, 238)

    # Units / scaling
    PX_PER_M: float = 2
    VISUAL_SPEED_GAIN: float = 10

    # Keyboard force input
    F_STEP_RATE: float = 1000.0  # N/s (how fast command ramps when key held)

    # Plots (5 on the left)
    PLOT_MARGIN: int = 10
    PLOT_W: int = 360
    PLOT_H: int = 100
    PLOT_VGAP: int = 10

    # Vehicle/road geometry
    EGO_SIZE: tuple = (30, 55)
    NPC_SIZE: tuple = (28, 50)
    LANE_COUNT: int = 2
    LANE_WIDTH: int = 90
    EDGE_WIDTH: int = 8
    CENTER_DASH_LEN_M: float = 50
    CENTER_DASH_GAP_M: float = 85

    # Traffic
    NPC_L1_MIN_KMH: float = 102.0
    NPC_L1_MAX_KMH: float = 120.0
    NPC_L2_MIN_KMH: float = 85.0
    NPC_L2_MAX_KMH: float = 98.0
    TRAFFIC_MIN_GAP_Y: int = 7 #in meters!
    SPAWN_INTERVAL: tuple = (0.7, 1.5)
    MAX_NPCS: int = 1

    # Initial speed (km/h) for ego
    V_INIT_KMH: float = 100.0

    ROAD_GUTTER: int = 50
    BTN_W: int = 140
    BTN_H: int = 36
    BTN_MARGIN: int = 10

