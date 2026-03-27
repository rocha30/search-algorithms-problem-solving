import pygame
import sys
import os
import time
from collections import deque
import heapq
from maze import load_maze, get_neighbors, Node, reconstruct_path, manhattan, euclidean

# ── Window & layout ───────────────────────────────────────────────
WIN_W, WIN_H    = 1100, 720
MAZE_W, MAZE_H  = 700, 700   # usable area for the maze
MAZE_OX         = 5          # left margin
PANEL_X         = MAZE_OX + MAZE_W + 15
PANEL_W         = WIN_W - PANEL_X - 10

# ── Palette ───────────────────────────────────────────────────────
C = {
    'bg':         (18,  18,  28),
    'panel':      (28,  28,  42),
    'border':     (55,  55,  80),
    'wall':       (30,  30,  40),
    'free':       (245, 245, 250),
    'start':      (40,  200, 80),
    'goal':       (220, 50,  50),
    'explored':   (100, 160, 220),
    'current':    (255, 255, 80),
    'path':       (255, 165, 0),
    'text':       (210, 210, 225),
    'text_dim':   (120, 120, 145),
    'btn':        (50,  80,  140),
    'btn_hover':  (70,  110, 180),
    'btn_active': (35,  145, 65),
    'btn_run':    (160, 90,  20),
    'btn_reset':  (130, 35,  35),
    'btn_text':   (255, 255, 255),
}

# ── Algorithms ────────────────────────────────────────────────────
ALGORITHMS = [
    ("BFS",                "bfs"),
    ("DFS",                "dfs"),
    ("Greedy (Manhattan)", "greedy_manhattan"),
    ("Greedy (Euclidean)", "greedy_euclidean"),
    ("A* (Manhattan)",     "astar_manhattan"),
    ("A* (Euclidean)",     "astar_euclidean"),
]

SPEEDS       = [1, 2, 5, 10, 30, 80, 250]
SPEED_LABELS = ["x1", "x2", "x5", "x10", "x30", "x80", "x250"]


# ── Generator algorithms ──────────────────────────────────────────
# Each generator yields (position, is_done, path_or_none).
# is_done=False → exploration step; is_done=True → finished (path=[] means no path).

def bfs_gen(grid, start, goal):
    queue = deque([Node(start)])
    visited = {start}
    while queue:
        cur = queue.popleft()
        yield cur.position, False, None
        if cur.position == goal:
            yield cur.position, True, reconstruct_path(cur)
            return
        for nb in get_neighbors(grid, cur.position):
            if nb not in visited:
                visited.add(nb)
                queue.append(Node(nb, parent=cur, g=cur.g + 1))
    yield None, True, []


def dfs_gen(grid, start, goal):
    stack = [Node(start)]
    visited = set()
    while stack:
        cur = stack.pop()
        if cur.position in visited:
            continue
        visited.add(cur.position)
        yield cur.position, False, None
        if cur.position == goal:
            yield cur.position, True, reconstruct_path(cur)
            return
        for nb in get_neighbors(grid, cur.position):
            if nb not in visited:
                stack.append(Node(nb, parent=cur, g=cur.g + 1))
    yield None, True, []


def greedy_gen(grid, start, goal, heuristic='manhattan'):
    h_func = euclidean if heuristic == 'euclidean' else manhattan
    counter = 0
    heap = [(h_func(start, goal), counter, Node(start, h=h_func(start, goal)))]
    visited = set()
    while heap:
        _, _, cur = heapq.heappop(heap)
        if cur.position in visited:
            continue
        visited.add(cur.position)
        yield cur.position, False, None
        if cur.position == goal:
            yield cur.position, True, reconstruct_path(cur)
            return
        for nb in get_neighbors(grid, cur.position):
            if nb not in visited:
                h = h_func(nb, goal)
                counter += 1
                heapq.heappush(heap, (h, counter, Node(nb, parent=cur, g=cur.g + 1, h=h)))
    yield None, True, []


def astar_gen(grid, start, goal, heuristic='manhattan'):
    h_func = euclidean if heuristic == 'euclidean' else manhattan
    h0 = h_func(start, goal)
    start_node = Node(start, g=0, h=h0)
    counter = 0
    heap = [(start_node.f, counter, start_node)]
    best_g = {start: 0}
    while heap:
        _, _, cur = heapq.heappop(heap)
        if cur.g > best_g.get(cur.position, float('inf')):
            continue
        yield cur.position, False, None
        if cur.position == goal:
            yield cur.position, True, reconstruct_path(cur)
            return
        for nb in get_neighbors(grid, cur.position):
            new_g = cur.g + 1
            if new_g < best_g.get(nb, float('inf')):
                best_g[nb] = new_g
                h = h_func(nb, goal)
                node = Node(nb, parent=cur, g=new_g, h=h)
                counter += 1
                heapq.heappush(heap, (node.f, counter, node))
    yield None, True, []


def make_generator(algo_key, grid, start, goal):
    if algo_key == 'bfs':
        return bfs_gen(grid, start, goal)
    if algo_key == 'dfs':
        return dfs_gen(grid, start, goal)
    if algo_key == 'greedy_manhattan':
        return greedy_gen(grid, start, goal, 'manhattan')
    if algo_key == 'greedy_euclidean':
        return greedy_gen(grid, start, goal, 'euclidean')
    if algo_key == 'astar_manhattan':
        return astar_gen(grid, start, goal, 'manhattan')
    if algo_key == 'astar_euclidean':
        return astar_gen(grid, start, goal, 'euclidean')


# ── GUI Application ───────────────────────────────────────────────

class MazeSolverGUI:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        pygame.display.set_caption("Maze Solver")
        self.clock = pygame.time.Clock()

        self.font_title = pygame.font.SysFont("monospace", 17, bold=True)
        self.font_md    = pygame.font.SysFont("monospace", 13)
        self.font_sm    = pygame.font.SysFont("monospace", 11)

        # Maze files
        data_dir = os.path.join(os.path.dirname(__file__), "data")
        self.maze_files = sorted(f for f in os.listdir(data_dir) if f.endswith(".txt"))
        self.data_dir   = data_dir
        self.file_idx   = 0
        self.algo_idx   = 0
        self.speed_idx  = 3   # default x10

        # Maze data
        self.grid        = None
        self.start       = None
        self.goal        = None
        self.cell_size   = 1
        self.maze_offset = (0, 0)
        self.base_surf   = None   # cached maze background
        self.overlay     = None   # explored / path overlay (RGBA)

        # Animation state
        self.gen            = None
        self.explored       = set()
        self.current_pos    = None
        self.path           = None
        self.running        = False
        self.done           = False

        # Metrics
        self.nodes_explored = 0
        self.path_length    = None
        self.elapsed        = 0.0
        self._t_start       = None

        # Click registry (rebuilt each frame)
        self._buttons = []

        self._load_maze()

    # ── Maze loading ──────────────────────────────────────────────

    def _load_maze(self):
        filepath = os.path.join(self.data_dir, self.maze_files[self.file_idx])
        self.grid, self.start, self.goal = load_maze(filepath)
        rows = len(self.grid)
        cols = len(self.grid[0])
        self.cell_size = max(1, min(MAZE_W // cols, MAZE_H // rows))
        mw = self.cell_size * cols
        mh = self.cell_size * rows
        ox = MAZE_OX + (MAZE_W - mw) // 2
        oy = (WIN_H - mh) // 2
        self.maze_offset = (ox, oy)
        self._build_base_surface()
        self._reset()

    def _build_base_surface(self):
        cs   = self.cell_size
        rows = len(self.grid)
        cols = len(self.grid[0])
        surf = pygame.Surface((cols * cs, rows * cs))
        for r, row in enumerate(self.grid):
            for c, val in enumerate(row):
                if val == 1:
                    color = C['wall']
                elif (r, c) == self.start:
                    color = C['start']
                elif (r, c) == self.goal:
                    color = C['goal']
                else:
                    color = C['free']
                pygame.draw.rect(surf, color, (c * cs, r * cs, cs, cs))
        self.base_surf = surf
        # transparent overlay same size
        self.overlay = pygame.Surface((cols * cs, rows * cs), pygame.SRCALPHA)
        self.overlay.fill((0, 0, 0, 0))

    def _reset(self):
        self.gen            = None
        self.explored       = set()
        self.current_pos    = None
        self.path           = None
        self.running        = False
        self.done           = False
        self.nodes_explored = 0
        self.path_length    = None
        self.elapsed        = 0.0
        self._t_start       = None
        if self.overlay:
            self.overlay.fill((0, 0, 0, 0))

    # ── Overlay helpers ───────────────────────────────────────────

    def _paint(self, pos, color, alpha=200):
        r, c = pos
        cs = self.cell_size
        pygame.draw.rect(self.overlay, (*color, alpha), (c * cs, r * cs, cs, cs))

    def _clear_cell(self, pos):
        r, c = pos
        cs = self.cell_size
        pygame.draw.rect(self.overlay, (0, 0, 0, 0), (c * cs, r * cs, cs, cs))

    # ── Animation step ────────────────────────────────────────────

    def _start_run(self):
        self._reset()
        self.gen     = make_generator(ALGORITHMS[self.algo_idx][1], self.grid, self.start, self.goal)
        self.running = True
        self._t_start = time.perf_counter()

    def _step(self):
        steps = SPEEDS[self.speed_idx]
        for _ in range(steps):
            try:
                pos, done, path = next(self.gen)
            except StopIteration:
                self._finish(None, [])
                return

            if not done:
                # paint explored (skip start/goal — they stay their color)
                if pos and pos != self.start and pos != self.goal:
                    # erase old current marker
                    if self.current_pos and self.current_pos != self.start and self.current_pos != self.goal:
                        self._paint(self.current_pos, C['explored'])
                    self._paint(pos, C['current'], 255)
                self.current_pos = pos
                self.explored.add(pos)
                self.nodes_explored += 1
            else:
                self._finish(pos, path)
                return

    def _finish(self, pos, path):
        self.elapsed = time.perf_counter() - self._t_start
        self.path    = path or []
        self.path_length = len(path) - 1 if path else None
        self.running = False
        self.done    = True
        # erase current marker and draw final path
        if self.current_pos and self.current_pos != self.start and self.current_pos != self.goal:
            self._paint(self.current_pos, C['explored'])
        for p in self.path:
            if p != self.start and p != self.goal:
                self._paint(p, C['path'], 230)

    # ── Drawing ───────────────────────────────────────────────────

    def draw_maze(self):
        ox, oy = self.maze_offset
        self.screen.blit(self.base_surf, (ox, oy))
        self.screen.blit(self.overlay,   (ox, oy))

    def draw_panel(self):
        # background
        pygame.draw.rect(self.screen, C['panel'], (PANEL_X - 8, 0, WIN_W - PANEL_X + 8, WIN_H))
        pygame.draw.line(self.screen, C['border'], (PANEL_X - 8, 0), (PANEL_X - 8, WIN_H), 1)

        x  = PANEL_X
        y  = 16
        bw = PANEL_W  # button width

        # ── Title ────────────────────────────────────────────────
        self._text(self.font_title, "MAZE SOLVER", x, y, C['text'])
        y += 28
        self._hline(x, y, bw)
        y += 12

        # ── File selector (prev / filename / next) ───────────────
        self._text(self.font_md, "Laberinto:", x, y, C['text_dim'])
        y += 18

        arrow_w = 28
        name_w  = bw - 2 * arrow_w - 4
        fname   = self.maze_files[self.file_idx].replace(".txt", "")

        r_prev = pygame.Rect(x, y, arrow_w, 24)
        r_name = pygame.Rect(x + arrow_w + 2, y, name_w, 24)
        r_next = pygame.Rect(x + arrow_w + 2 + name_w + 2, y, arrow_w, 24)

        self._btn(r_prev, "<", C['btn'], 'prev_file', None)
        pygame.draw.rect(self.screen, C['btn'], r_name, border_radius=3)
        self._center_text(self.font_sm, fname, r_name, C['btn_text'])
        self._btn(r_next, ">", C['btn'], 'next_file', None)
        y += 32

        self._hline(x, y, bw)
        y += 12

        # ── Algorithm selector ───────────────────────────────────
        self._text(self.font_md, "Algoritmo:", x, y, C['text_dim'])
        y += 18

        for i, (name, _) in enumerate(ALGORITHMS):
            rect  = pygame.Rect(x, y, bw, 22)
            color = C['btn_active'] if i == self.algo_idx else C['btn']
            self._btn(rect, name, color, 'algo', i)
            y += 25

        y += 4
        self._hline(x, y, bw)
        y += 12

        # ── Speed selector ───────────────────────────────────────
        self._text(self.font_md, f"Velocidad: {SPEED_LABELS[self.speed_idx]}", x, y, C['text_dim'])
        y += 18

        seg = bw // len(SPEEDS)
        for i, label in enumerate(SPEED_LABELS):
            rect  = pygame.Rect(x + i * seg, y, seg - 2, 22)
            color = C['btn_active'] if i == self.speed_idx else C['btn']
            self._btn(rect, label, color, 'speed', i)
        y += 30

        self._hline(x, y, bw)
        y += 12

        # ── Run / Reset ──────────────────────────────────────────
        hw = (bw - 6) // 2
        if self.running:
            run_label = "PAUSE"
            run_color = C['btn_run']
        elif self.gen and not self.done:
            run_label = "RESUME"
            run_color = C['btn_active']
        else:
            run_label = "RUN"
            run_color = C['btn_active']

        r_run   = pygame.Rect(x,        y, hw, 30)
        r_reset = pygame.Rect(x + hw + 6, y, hw, 30)
        self._btn(r_run,   run_label, run_color,   'run',   None)
        self._btn(r_reset, "RESET",   C['btn_reset'], 'reset', None)
        y += 38

        self._hline(x, y, bw)
        y += 12

        # ── Metrics ──────────────────────────────────────────────
        self._text(self.font_md, "Metricas:", x, y, C['text_dim'])
        y += 20

        if self.done and self.path_length is None:
            path_val = "NO PATH"
            path_col = (220, 80, 80)
        elif self.path_length is not None:
            path_val = str(self.path_length)
            path_col = C['text']
        else:
            path_val = "-"
            path_col = C['text_dim']

        metrics = [
            ("Nodos explorados", str(self.nodes_explored), C['text']),
            ("Largo del camino", path_val,                 path_col),
            ("Tiempo",           f"{self.elapsed:.4f}s" if self.elapsed else "-", C['text']),
        ]
        for label, value, vcol in metrics:
            self._text(self.font_sm, label + ":", x, y, C['text_dim'])
            vs = self.font_md.render(value, True, vcol)
            self.screen.blit(vs, (x + bw - vs.get_width(), y))
            y += 20

        # ── Status ───────────────────────────────────────────────
        y += 6
        if self.running:
            status, scol = "Ejecutando...", (255, 200, 50)
        elif self.done and self.path_length is not None:
            status, scol = "Solucion encontrada!", (80, 200, 80)
        elif self.done:
            status, scol = "Sin camino", (220, 80, 80)
        else:
            status, scol = "Listo — presiona RUN", C['text_dim']
        self._text(self.font_md, status, x, y, scol)
        y += 22

        # ── Legend ───────────────────────────────────────────────
        self._hline(x, y, bw)
        y += 10
        self._text(self.font_sm, "Leyenda:", x, y, C['text_dim'])
        y += 16

        legend = [
            (C['start'],    "Inicio"),
            (C['goal'],     "Meta"),
            (C['explored'], "Explorado"),
            (C['path'],     "Camino"),
            (C['current'],  "Actual"),
        ]
        lx = x
        for color, label in legend:
            pygame.draw.rect(self.screen, color, (lx, y, 11, 11))
            lbl = self.font_sm.render(label, True, C['text'])
            self.screen.blit(lbl, (lx + 14, y))
            lx += 70
            if lx + 70 > x + bw:
                lx = x
                y += 16

        # ── Keyboard shortcuts hint ───────────────────────────────
        hint_y = WIN_H - 24
        hint = "SPACE: run/pause   R: reset   ←/→: cambiar laberinto"
        self._text(self.font_sm, hint, x, hint_y, C['text_dim'])

    # ── UI helpers ────────────────────────────────────────────────

    def _text(self, font, text, x, y, color):
        self.screen.blit(font.render(text, True, color), (x, y))

    def _hline(self, x, y, w):
        pygame.draw.line(self.screen, C['border'], (x, y), (x + w, y), 1)

    def _center_text(self, font, text, rect, color):
        surf = font.render(text, True, color)
        self.screen.blit(surf, (rect.x + (rect.w - surf.get_width()) // 2,
                                rect.y + (rect.h - surf.get_height()) // 2))

    def _btn(self, rect, label, base_color, kind, value):
        mx, my = pygame.mouse.get_pos()
        color  = C['btn_hover'] if rect.collidepoint(mx, my) and base_color not in (C['btn_active'], C['btn_reset'], C['btn_run']) else base_color
        pygame.draw.rect(self.screen, color, rect, border_radius=3)
        self._center_text(self.font_sm, label, rect, C['btn_text'])
        self._buttons.append((rect, kind, value))

    def _handle_click(self, pos):
        for rect, kind, value in self._buttons:
            if rect.collidepoint(pos):
                if kind == 'prev_file':
                    self.file_idx = (self.file_idx - 1) % len(self.maze_files)
                    self._load_maze()
                elif kind == 'next_file':
                    self.file_idx = (self.file_idx + 1) % len(self.maze_files)
                    self._load_maze()
                elif kind == 'algo':
                    self.algo_idx = value
                elif kind == 'speed':
                    self.speed_idx = value
                elif kind == 'run':
                    if self.running:
                        self.running = False
                    elif self.gen and not self.done:
                        self.running = True
                    else:
                        self._start_run()
                elif kind == 'reset':
                    self._reset()
                break

    # ── Main loop ─────────────────────────────────────────────────

    def run(self):
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    self._handle_click(event.pos)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if self.running:
                            self.running = False
                        elif self.gen and not self.done:
                            self.running = True
                        else:
                            self._start_run()
                    elif event.key == pygame.K_r:
                        self._reset()
                    elif event.key == pygame.K_RIGHT:
                        self.file_idx = (self.file_idx + 1) % len(self.maze_files)
                        self._load_maze()
                    elif event.key == pygame.K_LEFT:
                        self.file_idx = (self.file_idx - 1) % len(self.maze_files)
                        self._load_maze()

            if self.running:
                self._step()

            # reset button registry right before drawing so clicks from the
            # previous frame still resolve correctly via _handle_click
            self._buttons = []
            self.screen.fill(C['bg'])
            self.draw_maze()
            self.draw_panel()
            pygame.display.flip()
            self.clock.tick(60)


if __name__ == "__main__":
    MazeSolverGUI().run()
