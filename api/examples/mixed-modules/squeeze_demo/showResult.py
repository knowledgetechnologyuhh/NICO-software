import pygame

 
# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

pygame.init()

# Set the width and height of the screen [width, height]
size = (1000, 300)
screen = pygame.display.set_mode(size)

screen.fill(WHITE)

pygame.display.set_caption("surprise egg")

# Used to manage how fast the screen updates
clock = pygame.time.Clock()
 
 

# for displaying text in the game 
pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.

# message for game over
mgItem = pygame.font.SysFont('Comic Sans MS', 200)

for x in range(36000):
    
    #check if file has changed
    if (True):
		screen.fill(WHITE)
		filename="./item.txt"
		with open(filename,'r') as f:
			item = f.read()
		textsurfaceItem = mgItem.render(item[:7], False, (0, 0, 0))
		text_rect = textsurfaceItem.get_rect(center=(1000/2, 300/2))
		screen.blit(textsurfaceItem,text_rect)
		# --- Go ahead and update the screen with what we've drawn.
		pygame.display.flip()
		pygame.display.flip()
		 
			# --- Limit to 2 frames per second
		clock.tick(2)
        

    


