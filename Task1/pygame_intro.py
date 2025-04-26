import pygame
pygame.init()

SCREEN_WIDTH = 600
SCREEN_HEIGHT = 400

x,y = 50,50
width,height = 40, 60
vel = 5
rad = 0
clicked = False
pos1 = (0,0)
pos2= (0,0)

window = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT)) 
pygame.display.set_caption("First Game")

run = True
while run:
    pygame.time.delay(100)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            clicked = True
        if event.type == pygame.MOUSEBUTTONUP:
            clicked = False

    if(clicked == True):
        pos1 = pygame.mouse.get_pos()
        x,y = pos1[0],pos1[1]
        rad += vel

    elif(clicked == False):
        pos2 = pygame.mouse.get_pos()
        rad = 0
    
    pygame.draw.circle(window, (255,0,0), (x,y), rad, width = 0)
    pygame.draw.line(window, (0,255,0), pos1, pos2, 5)
    pygame.display.update()

pygame.quit()