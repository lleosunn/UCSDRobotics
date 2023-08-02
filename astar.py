import pygame
import math
from queue import PriorityQueue

WIDTH = 500
ROWS = 50
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

class Spot:
	def __init__(self, row, col, width, total_rows):
		self.row = row
		self.col = col
		self.x = row * width
		self.y = col * width
		self.color = WHITE
		self.neighbors = []
		self.width = width
		self.total_rows = total_rows

	def get_pos(self):
		return self.row, self.col

	def is_closed(self):
		return self.color == RED

	def is_open(self):
		return self.color == GREEN

	def is_barrier(self):
		return self.color == BLACK

	def is_inflation(self):
		return self.color == GREY
	
	def is_barrierinflation(self):
		return self.color == BLACK or self.color == GREY

	def is_start(self):
		return self.color == ORANGE

	def is_end(self):
		return self.color == TURQUOISE

	def reset(self):
		self.color = WHITE

	def make_start(self):
		self.color = ORANGE

	def make_closed(self):
		self.color = RED

	def make_open(self):
		self.color = GREEN

	def make_barrier(self):
		self.color = BLACK

	def make_inflation(self):
		self.color = GREY

	def make_end(self):
		self.color = TURQUOISE

	def make_path(self):
		self.color = PURPLE

	def draw(self, win):
		pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

	def update_neighbors(self, grid):
		self.neighbors = []
		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrierinflation(): # DOWN
			self.neighbors.append(grid[self.row + 1][self.col])

		if self.row > 0 and not grid[self.row - 1][self.col].is_barrierinflation(): # UP
			self.neighbors.append(grid[self.row - 1][self.col])

		if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrierinflation(): # RIGHT
			self.neighbors.append(grid[self.row][self.col + 1])

		if self.col > 0 and not grid[self.row][self.col - 1].is_barrierinflation(): # LEFT
			self.neighbors.append(grid[self.row][self.col - 1])

		if (self.row < self.total_rows - 1) and (not grid[self.row + 1][self.col].is_barrierinflation()) and (self.col > 0) and (not grid[self.row][self.col - 1].is_barrierinflation()): # DOWN LEFT
			self.neighbors.append(grid[self.row + 1][self.col - 1])

		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrierinflation() and self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrierinflation(): # DOWN RIGHT
			self.neighbors.append(grid[self.row + 1][self.col + 1])

		if self.row > 0 and not grid[self.row - 1][self.col].is_barrierinflation() and self.col > 0 and not grid[self.row][self.col - 1].is_barrierinflation(): # UP LEFT
			self.neighbors.append(grid[self.row - 1][self.col - 1])

		if self.row > 0 and not grid[self.row - 1][self.col].is_barrierinflation() and self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrierinflation(): # UP RIGHT
			self.neighbors.append(grid[self.row - 1][self.col + 1])

	def __lt__(self, other):
		return False

def inflate_obstacles(grid, inflate_range=1):
    inflation = []
    for row in grid:
        for spot in row:
            if spot.is_barrier():
                inflation.append(spot)

    for spot in inflation:
        obstacle_row, obstacle_col = spot.get_pos()
        for row in range(-inflate_range, inflate_range + 1):
            for col in range(-inflate_range, inflate_range + 1):
                if row == 0 and col == 0:  # Skip the obstacle itself
                    continue
                inflated_row, inflated_col = obstacle_row + row, obstacle_col + col
                if (0 <= inflated_row < len(grid) and 
                    0 <= inflated_col < len(grid[0]) and
                    not grid[inflated_row][inflated_col].is_barrier()):  # Make sure spot is in the grid and is not already an obstacle.
                    grid[inflated_row][inflated_col].make_inflation()  # Mark the spot as an obstacle

def h(p1, p2): #manhattan vs euclidean distance: heuristic function
	x1, y1 = p1
	x2, y2 = p2
	# return abs(x1 - x2) + abs(y1 - y2)
	return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def reconstruct_path(came_from, current, draw):
	while current in came_from:
		current = came_from[current]
		current.make_path()
		draw()
	
def reset_path(grid, start, end):
    for row in grid:
        for spot in row:
            if spot != start and spot != end and not spot.is_barrier() and not spot.is_inflation():
                spot.reset()

def algorithm(draw, grid, start, end):
	count = 0
	open_set = PriorityQueue()
	open_set.put((0, count, start)) #add to priority queue
	came_from = {}
	g_score = {spot: float("inf") for row in grid for spot in row}
	g_score[start] = 0 #distance from start
	f_score = {spot: float("inf") for row in grid for spot in row}
	f_score[start] = h(start.get_pos(), end.get_pos()) #distance to end

	open_set_hash = {start}

	while not open_set.empty():
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit() #way to exit the loop

		current = open_set.get()[2] #starting at the start node
		open_set_hash.remove(current)

		if current == end:
			reconstruct_path(came_from, end, draw)
			end.make_end()
			return True

		for neighbor in current.neighbors:
			if abs(neighbor.row - current.row) == 1 and abs(neighbor.col - current.col) == 1:
				temp_g_score = g_score[current] + math.sqrt(2)
			else:
				temp_g_score = g_score[current] + 1
			if temp_g_score < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
				if neighbor not in open_set_hash:
					count += 1
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_hash.add(neighbor)
					neighbor.make_open()
		draw()

		if current != start:
			current.make_closed()

	return False

def make_grid(rows, width): #makes grid
	grid = []
	gap = width // rows
	for i in range(rows):
		grid.append([])
		for j in range(rows):
			spot = Spot(i, j, gap, rows)
			grid[i].append(spot)

	return grid #2 dimensional list

def draw_grid(win, rows, width): #draws gridlines
	gap = width // rows
	for i in range(rows):
		pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
		for j in range(rows):
			pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))

def draw(win, grid, rows, width):
	win.fill(WHITE)

	for row in grid:
		for spot in row:
			spot.draw(win) #draws each box color

	draw_grid(win, rows, width) #draws gridlines
	pygame.display.update() #update display

def get_clicked_pos(pos, rows, width): #gets mouse click position
	gap = width // rows
	y, x = pos

	row = y // gap
	col = x // gap

	return row, col

def main(win, width):
	grid = make_grid(ROWS, width)
	start = None
	end = None
	run = True

	while run:
		draw(win, grid, ROWS, width)
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

			if pygame.mouse.get_pressed()[0]: # left click
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				if not start and spot != end:
					start = spot
					start.make_start()

				elif not end and spot != start:
					end = spot
					end.make_end()

				elif spot != end and spot != start:
					spot.make_barrier()
					print(spot.x / 50, -(spot.y / 50) + 10)

			elif pygame.mouse.get_pressed()[2]: # RIGHT
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				spot.reset()
				if spot == start:
					start = None
				elif spot == end:
					end = None

			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_c:
					start = None
					end = None
					grid = make_grid(ROWS, width)

				if event.key == pygame.K_SPACE and start and end:
					reset_path(grid, start, end)
					inflate_obstacles(grid)  # Inflate obstacles before running the algorithm
					for row in grid:
						for spot in row:
							spot.update_neighbors(grid)
					algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)

	pygame.quit()

main(WIN, WIDTH)