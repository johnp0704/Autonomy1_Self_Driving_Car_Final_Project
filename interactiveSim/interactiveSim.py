# copyright 2025 Dr. Hamid Ossareh, Univ of. Vermont
# do not share or distribute
# EE5550: Autonomy; final project

import pygame, random, sys, math, csv, time
from collections import deque
from car import Car
from cfg import Cfg
from gradefield import GradeField
from controller import Controller

cfg = Cfg()
SEED = 10
random.seed(SEED)

# =========================
# Unit helpers
# =========================
def kmh_to_mps(kmh): return kmh / 3.6
def mps_to_kmh(mps): return mps * 3.6
def mps_to_pxps(mps): return mps * cfg.PX_PER_M

# =========================
# Drawing helpers
# =========================
def rounded_rect(surface, color, rect, radius=10, width=0):
    pygame.draw.rect(surface, color, rect, width=width, border_radius=radius)

def draw_button(screen, rect, text, font):
    mouse_pos = pygame.mouse.get_pos()
    hovered = rect.collidepoint(mouse_pos)
    bg = (35, 120, 175) if hovered else cfg.BTN_BG
    border = (200, 230, 255) if hovered else cfg.BTN_BORDER
    rounded_rect(screen, bg, rect, radius=8)
    rounded_rect(screen, border, rect, radius=8, width=1)
    label = font.render(text, True, cfg.BTN_TEXT)
    screen.blit(label, (rect.centerx - label.get_width()//2,
                        rect.centery - label.get_height()//2))

# =========================
# Layout
# =========================
CENTER_DASH_LEN = int(cfg.CENTER_DASH_LEN_M * cfg.PX_PER_M)
CENTER_DASH_GAP = int(cfg.CENTER_DASH_GAP_M * cfg.PX_PER_M)

PLOT_RECTS = [
    pygame.Rect(cfg.PLOT_MARGIN, cfg.PLOT_MARGIN + i*(cfg.PLOT_H + cfg.PLOT_VGAP), cfg.PLOT_W, cfg.PLOT_H)
    for i in range(5)
]
GRADE_PLOT_RECT, PLOT_2, PLOT_3, PLOT_4, PLOT_5 = PLOT_RECTS
LEFT_COLUMN_RIGHT = cfg.PLOT_MARGIN + cfg.PLOT_W
ROAD_X = LEFT_COLUMN_RIGHT + cfg.ROAD_GUTTER
ROAD_WIDTH = cfg.LANE_COUNT * cfg.LANE_WIDTH

def lane_x_center(lane_idx):
    return ROAD_X + cfg.LANE_WIDTH * (0.5 + lane_idx)

# =========================
# Plots
# =========================
class LinePlot:
    def __init__(self, rect, win=4.0, vmin=-110, vmax=110, title=""):
        self.rect, self.win, self.vmin, self.vmax, self.title = rect, win, vmin, vmax, title
        self.data, self.font = deque(), None
    def add(self, t, v):
        self.data.append((t, v))
        tmin = t - self.win
        while self.data and self.data[0][0] < tmin:
            self.data.popleft()
    def draw(self, screen):
        if self.font is None: self.font = pygame.font.SysFont(None, 18)
        r = self.rect
        rounded_rect(screen, (6, 33, 53), r, radius=10)
        rounded_rect(screen, cfg.GRID, r, radius=10, width=1)
        if len(self.data) < 2: return
        t_now = self.data[-1][0]; t_min = t_now - self.win
        x0, y0, w, h = r.x+8, r.y+8, r.w-16, r.h-16
        def map_x(t): return x0 + (t - t_min)/self.win * w
        def map_y(v):
            cl = max(self.vmin, min(self.vmax, v))
            return y0 + (1 - (cl - self.vmin)/(self.vmax - self.vmin)) * h
        lbl = self.font.render(self.title, True, cfg.PALE); screen.blit(lbl, (r.x+10, r.y+6))
        pygame.draw.line(screen, cfg.GRID, (x0, map_y(0.0)), (x0+w, map_y(0.0)), 1)
        pts = [(map_x(t), map_y(v)) for (t, v) in self.data]
        pygame.draw.lines(screen, cfg.PALE, False, pts, 2)
        v_now = self.data[-1][1]
        val_text = f"{v_now:6.2f}"
        val_lbl = self.font.render(val_text, True, cfg.PALE)
        screen.blit(val_lbl, (r.right - val_lbl.get_width() - 10, r.y + 6))

class GradeWindowPlot:
    def __init__(self, rect, halfwin=2.0, gmin=-8, gmax=8):
        self.rect, self.halfwin, self.gmin, self.gmax = rect, halfwin, gmin, gmax
        self.font = None
    def draw(self, screen, field: GradeField, x_now_m):
        if self.font is None: self.font = pygame.font.SysFont(None, 18)
        r = self.rect
        rounded_rect(screen, (6, 33, 53), r, radius=10)
        rounded_rect(screen, cfg.GRID, r, radius=10, width=1)
        screen.blit(self.font.render("Road grade (%)", True, cfg.PALE), (r.x+10, r.y+6))
        x0, y0, w, h = r.x+8, r.y+8, r.w-16, r.h-16
        x_min, x_max = x_now_m - self.halfwin*50, x_now_m + self.halfwin*50  # window in meters

        def map_x(x): return x0 + (x - x_min)/(x_max - x_min) * w
        def map_y(g):
            cl = max(self.gmin, min(self.gmax, g))
            return y0 + (1 - (cl - self.gmin)/(self.gmax - self.gmin)) * h

        for g in (0.0, 5.0, -5.0):
            pygame.draw.line(screen, cfg.GRID, (x0, map_y(g)), (x0+w, map_y(g)), 1)

        pts = [(map_x(x_min + (i/120)*(x_max - x_min)),
                map_y(field.grade(x_min + (i/120)*(x_max - x_min))))
            for i in range(121)]
        pygame.draw.lines(screen, cfg.PALE2, False, pts, 2)
        x_now = map_x(x_now_m)
        pygame.draw.line(screen, cfg.PALE2, (x_now, y0), (x_now, y0+h), 1)
        g_now = field.grade(x_now_m)
        pygame.draw.circle(screen, cfg.PALE, (int(x_now), int(map_y(g_now))), 4)

# =========================
# World objects (Lane markers, Traffic, NPCs) S
# =========================
class LaneMarkers:
    def __init__(self):
        self.offset = 0.0
        self.total = CENTER_DASH_LEN + CENTER_DASH_GAP
    def update(self, dt, ego_forward_pxps):
        self.offset = (self.offset + ego_forward_pxps * dt * cfg.VISUAL_SPEED_GAIN) % self.total
    def draw(self, screen):
        # Solid edges
        pygame.draw.rect(screen, cfg.PALE, (ROAD_X - cfg.EDGE_WIDTH, 0, cfg.EDGE_WIDTH, cfg.H))
        pygame.draw.rect(screen, cfg.PALE, (ROAD_X + ROAD_WIDTH, 0, cfg.EDGE_WIDTH, cfg.H))
        # Dashed centerline
        cx = ROAD_X + ROAD_WIDTH // 2 - 4
        y = -CENTER_DASH_LEN + int(self.offset)
        while y < cfg.H + CENTER_DASH_LEN:
            pygame.draw.rect(screen, cfg.PALE, (cx, y, 8, CENTER_DASH_LEN), border_radius=3)
            y += self.total

class NPCCar:
    def __init__(self, lane, spawn_side, speed_mps, ego):
        self.w, self.h = cfg.NPC_SIZE
        self.lane = lane
        self.speed_mps = speed_mps 
        self.ego = ego              # to compute rendering from ego position

        # initialize relative longitudinal distance in meters from a screen spawn edge
        spawn_y_px = (-self.h) if spawn_side == 0 else (cfg.H + self.h)
        # include VISUAL_SPEED_GAIN to match render mapping
        self.s_rel_m = (ego.cy - spawn_y_px) / (cfg.PX_PER_M * cfg.VISUAL_SPEED_GAIN)

        # pre-rendered body
        self.surf = pygame.Surface(cfg.NPC_SIZE, pygame.SRCALPHA)
        sh = pygame.Surface((self.w, self.h), pygame.SRCALPHA); sh.fill(cfg.SHADOW)
        self.surf.blit(sh, (0, 0))
        body = cfg.PALE2 if random.random() < 0.5 else cfg.PALE
        rounded_rect(self.surf, body, self.surf.get_rect(), 9)
        pygame.draw.rect(self.surf, (200,205,210), (5, 6, self.w-10, 10), border_radius=6)
        pygame.draw.rect(self.surf, (200,205,210), (5, self.h-14, self.w-10, 10), border_radius=5)

    def render_x_px(self):
        return int(lane_x_center(self.lane) - self.w/2)

    def render_y_px(self):
        # map meters to pixels, apply visual speed gain for scrolling look
        return self.ego.cy - self.s_rel_m * cfg.PX_PER_M * cfg.VISUAL_SPEED_GAIN

    def draw(self, screen):
        screen.blit(self.surf, (self.render_x_px(), int(self.render_y_px())))

    def offscreen(self):
        y_px = self.render_y_px()
        return y_px > cfg.H + 140 or y_px < -self.h - 240

class Traffic:
    def __init__(self, ego):
        self.ego = ego
        self.cars, self.t_until_spawn = [], random.uniform(*cfg.SPAWN_INTERVAL)
        self.data = []  # rows: [longitudinal_m, lateral_m, rel_speed_mps]
        self.min_distance = 100

    def _lane_spawn_clear(self, lane, spawn_side):
        # crude visual spacing check in pixel space (uses current render y)
        for c in self.cars:
            if c.lane != lane:
                continue
            y_px = c.render_y_px()
            tmp = cfg.TRAFFIC_MIN_GAP_Y * cfg.PX_PER_M * cfg.VISUAL_SPEED_GAIN
            if spawn_side == 0 and y_px < tmp:  # near top
                return False
            if spawn_side == 1 and y_px > (cfg.H - tmp):  # near bottom
                return False
        return True

    def update(self, dt):
        # spawn
        self.t_until_spawn -= dt
        if self.t_until_spawn <= 0 and len(self.cars) < cfg.MAX_NPCS:
            lane = random.randrange(cfg.LANE_COUNT)
            spawn_side = random.randint(0, 1)  # 0: top, 1: bottom
            if (lane == 0):
                speed_mps = kmh_to_mps(random.uniform(cfg.NPC_L1_MIN_KMH, cfg.NPC_L1_MAX_KMH))
            else:
                speed_mps = kmh_to_mps(random.uniform(cfg.NPC_L2_MIN_KMH, cfg.NPC_L2_MAX_KMH))
            if self._lane_spawn_clear(lane, spawn_side):
                self.cars.append(NPCCar(lane, spawn_side, speed_mps, self.ego))
            self.t_until_spawn = random.uniform(*cfg.SPAWN_INTERVAL)

        self.data = []
        ego_vx_mps = self.ego.car.vx  # ego forward speed in m/s

        # car-following in meters per lane
        min_gap_m = cfg.TRAFFIC_MIN_GAP_Y
        lane_groups = {}
        for c in self.cars:
            lane_groups.setdefault(c.lane, []).append(c)
        for lane, group in lane_groups.items():
            group.sort(key=lambda nc: nc.s_rel_m, reverse=True)  # farthest ahead first
            for i in range(len(group) - 1):
                leader = group[i]
                follower = group[i+1]
                gap = leader.s_rel_m - follower.s_rel_m
                if 0 < gap < min_gap_m and follower.speed_mps > leader.speed_mps:
                    follower.speed_mps = leader.speed_mps

        # integrate once in meters, collect data
        self.min_distance = 100
        for c in self.cars:
            rel_lat_m = (lane_x_center(c.lane) - self.ego.cx) / cfg.PX_PER_M
            rel_speed_mps = c.speed_mps - ego_vx_mps
            if abs(rel_lat_m)<cfg.LANE_WIDTH/cfg.PX_PER_M/2:
                if (0<-c.s_rel_m<min_gap_m and rel_speed_mps>0):
                    rel_speed_mps = 0
                if(0 < c.s_rel_m < self.min_distance):
                    self.min_distance = c.s_rel_m
                
            c.s_rel_m += rel_speed_mps * dt
            self.data.append([c.s_rel_m, rel_lat_m, rel_speed_mps])
            if(math.sqrt(c.s_rel_m**2+rel_lat_m**2) < 6.9):
                print(print("\033[31m TOO CLOSE! \033[0m"))

        # cull offscreen
        self.cars = [c for c in self.cars if not c.offscreen()]

    def draw(self, screen):
        # draw back-to-front by render y
        for c in sorted(self.cars, key=lambda k: k.render_y_px()):
            c.draw(screen)

# =========================
# Ego sprite that *uses* Car dynamics
# =========================
class EgoSprite:
    def __init__(self, car: Car):
        self.car = car
        self.w, self.h = cfg.EGO_SIZE
        road_center_x = ROAD_X + ROAD_WIDTH // 2
        self.cx0 = float(road_center_x)            # starting pixel center (lateral)
        self.cy  = float(int(cfg.H * 0.74) + self.h//2)

        # pre-rendered body
        self.base = pygame.Surface(cfg.EGO_SIZE, pygame.SRCALPHA)
        sh = pygame.Surface((self.w, self.h), pygame.SRCALPHA); sh.fill(cfg.SHADOW)
        self.base.blit(sh, (0, 0))
        rounded_rect(self.base, cfg.EGO_COLOR, self.base.get_rect(), 10)
        pygame.draw.rect(self.base, cfg.EGO_ACCENT, (6, 8, self.w-12, 12), border_radius=8)
        pygame.draw.rect(self.base, cfg.EGO_ACCENT, (6, self.h-18, self.w-12, 12), border_radius=6)
        for wx in (6, self.w-12):
            pygame.draw.rect(self.base, (100,150,190), (wx, 18, 6, 16), border_radius=3)
            pygame.draw.rect(self.base, (100,150,190), (wx, self.h-34, 6, 16), border_radius=3)

    @property
    def yaw(self):
        return self.car.phi

    def update_screen_pose(self):
        # Map car.y (m lateral) -> lateral pixels
        self.cx = self.cx0 + self.car.y * cfg.PX_PER_M * 2 #*2 multiplier to speed up

    def draw(self, screen):
        self.update_screen_pose()
        angle_deg = -math.degrees(self.car.phi)  # screen y down
        rotated = pygame.transform.rotate(self.base, angle_deg)
        rect = rotated.get_rect(center=(int(self.cx), int(self.cy)))
        screen.blit(rotated, rect.topleft)

# =========================
# Main
# =========================
pygame.init()
screen = pygame.display.set_mode((cfg.W, cfg.H))
pygame.display.set_caption("Final project")
clock = pygame.time.Clock()

Ts = 1.0 / cfg.FPS   # update rate of the screen and the controller
ratio = 5
T_phys = Ts/ratio    # update rate of the physics

car = Car(T_phys, initial_speed=kmh_to_mps(cfg.V_INIT_KMH))
controller = Controller(Ts, [809.88, car.speed])
ego = EgoSprite(car)
traffic = Traffic(ego)
marks = LaneMarkers()
grade = GradeField(spacing=50.0, amp=5.0, seed=SEED)

# Plots
plot_2  = LinePlot(PLOT_2, win=4.0, vmin=0,   vmax=140, title="Speed V (km/h)")
plot_3 = LinePlot(PLOT_3,  win=4.0, vmin=200, vmax=6000, title="Fuel rate")  
plot_4 = LinePlot(PLOT_4,  win=4.0, vmin=-5000, vmax=1700, title="Force (N)")  
plot_5 = LinePlot(PLOT_5,  win=4.0, vmin=-8,   vmax=8,   title="Steering (deg)")
grade_pl = GradeWindowPlot(GRADE_PLOT_RECT)

font = pygame.font.SysFont(None, 20)
big_font = pygame.font.SysFont(None, 36)

sim_t, paused = 0.0, True
user_speed = 100.0/3.6           # desired m/s (passed into Controller)
des_lane = 1                     # desired lane (0 or 1)

telemetry = []

btn_rect = pygame.Rect(cfg.W - cfg.BTN_MARGIN - cfg.BTN_W, cfg.BTN_MARGIN, cfg.BTN_W, cfg.BTN_H)

running = True
while running:
    dt = clock.tick(cfg.FPS) / 1000.0

    # Events
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        elif e.type == pygame.KEYDOWN:
            if e.key in (pygame.K_ESCAPE, pygame.K_q): running = False
            elif e.key == pygame.K_SPACE: paused = not paused
        elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
            if btn_rect.collidepoint(e.pos) and telemetry:
                ts = time.strftime("%Y%m%d_%H%M%S")
                fname = f"driving_log_{ts}.csv"
                with open(fname, "w", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow(["time_s","speed_mps","lateral_pos_m",
                                        "driving_force_N","steering_rad","yaw_rad","total_fuel_mg","min_dist_m"])
                    writer.writerows(telemetry)
                print(f"Saved {len(telemetry)} rows to {fname}")

    # Update
    if not paused:
        sim_t += dt
        keys = pygame.key.get_pressed()

        # Desired speed adjustments
        if keys[pygame.K_UP]:   user_speed += cfg.F_STEP_RATE * dt * 0.01
        if keys[pygame.K_DOWN]: user_speed -= cfg.F_STEP_RATE * dt * 0.01
        if (user_speed < 20.83):
            user_speed = 20.83
        if (user_speed > 27.78):
            user_speed = 27.78

        # Steering command via lane selection
        if keys[pygame.K_RIGHT]:  des_lane = min(des_lane + 1, 1)
        if keys[pygame.K_LEFT]:   des_lane = max(des_lane - 1,-1)

        Fd, delta_cmd = controller.update(car.speed, car.y, car.phi, user_speed, des_lane, traffic.data, grade)

        # Grade and dynamics step
        beta = math.atan(grade.grade_at(car.x) / 100.0)  # radians
        for _ in range(ratio):
            car.update(Fd, delta_cmd, beta)

        # World scroll uses forward component car.vx (allow negative for reverse)
        ego_forward_pxps = mps_to_pxps(car.vx)
        marks.update(dt, ego_forward_pxps)
        traffic.update(dt)  

        # Plots
        plot_2.add(sim_t, mps_to_kmh(car.speed))
        plot_3.add(sim_t, car.fuel_rate)
        plot_4.add(sim_t, Fd)
        plot_5.add(sim_t, delta_cmd*180/math.pi)

        # Telemetry (≤ 60 s)
        if sim_t <= 60.0:
            telemetry.append([sim_t, car.speed, car.y, Fd, delta_cmd, car.phi, car.total_fuel_used, traffic.min_distance])

    # Draw
    screen.fill(cfg.BG_BLUE)
    pygame.draw.rect(screen, cfg.ROAD_DARK, (ROAD_X, 0, ROAD_WIDTH, cfg.H))
    marks.draw(screen)
    traffic.draw(screen)
    ego.draw(screen)

    # Plots
    grade_pl.draw(screen, grade, car.x)
    plot_2.draw(screen); plot_3.draw(screen); plot_4.draw(screen); plot_5.draw(screen)

    # Pause overlay
    if paused:
        overlay = pygame.Surface((cfg.W, cfg.H), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 120))
        screen.blit(overlay, (0, 0))
        msg = big_font.render("PAUSED  (press SPACE to resume)", True, cfg.PALE)
        screen.blit(msg, (cfg.W//2 - msg.get_width()//2, cfg.H//2 - msg.get_height()//2))

    # Save button on top
    draw_button(screen, btn_rect, "Save Data", font)

    # HUD
    fps = clock.get_fps()
    hud = font.render(
        f"FPS: {fps:4.1f}, Vdes: {user_speed:5.1f} m/s, Fuel: {car.total_fuel_used/1000:3.0f} g, t: {sim_t:3.0f} s, min dist: {traffic.min_distance:5.1f} m",
        True, cfg.PALE
    )
    screen.blit(hud, (12, cfg.H - 28))
    pygame.display.flip()

pygame.quit(); sys.exit()
