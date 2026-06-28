import pygame

WIDTH = 1000
HEIGHT = 1000
PX_PER_CM = 2.5 #scaling variable. increase for bigger objects and distances, decrease for greater reach.
# pygame setup
colours = {1:"red", 2:"green", 3:"blue", 4:"black", 5:"white"}
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
running = True
zooming = False

# Load robot sprite
robot_sprite = pygame.image.load("SPRITE_ROB.gif").convert_alpha()
sprite_width, sprite_height = robot_sprite.get_size()

def sort_linepoints(points):
    points_sorted = [points.pop(0)]
    for i in range(0, len(points)):
        min_dist = 99999
        min_index = -1
        index = -1
        lastpoint = points_sorted[-1]
        for p in points:
            index += 1
            distance_sqr = (p[0]-lastpoint[0])**2 + (p[1]-lastpoint[1])**2
            if distance_sqr < min_dist:
                min_dist = distance_sqr
                min_index = index
        points_sorted.append(points.pop(min_index))
    return points_sorted

def rescale(coord):
    return (coord-500)*PX_PER_CM+500

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close the window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if zooming == False:
                if event.key == pygame.K_UP:
                    PX_PER_CM = min(10, PX_PER_CM + 0.5)
                    zooming = True
                    print(PX_PER_CM)
                if event.key == pygame.K_DOWN:
                    PX_PER_CM = max(1, PX_PER_CM - 0.5)
                    zooming = True
                    print(PX_PER_CM)
        else: zooming = False

    # fill the screen with a color to wipe away anything from last frame; also serves as background
    screen.fill("orange")
    #draw the axes
    pygame.draw.line(screen, "black", (500, 0), (500, 1000), width=1)
    pygame.draw.line(screen, "black", (0, 500), (1000, 500), width=1)

    # render map objects
    with open("objects.txt") as objectfile:
        linepoints = []
        mountainpoints = []
        mountains = []
        robot1pos = []
        for line in objectfile:
            objecttype = int(line[6])
            objectproperties = [int(line[0:3]), int(line[3:6]), int(line[7])]
            xcoord = rescale(int(objectproperties[0]))
            ycoord = rescale(1000 - int(objectproperties[1]))
            if objecttype == 1: #small sample
                pygame.draw.rect(screen, colours[objectproperties[2]], pygame.Rect(xcoord, ycoord, 3*PX_PER_CM, 3*PX_PER_CM))
            elif objecttype == 2:#large sample
                pygame.draw.rect(screen, colours[objectproperties[2]], pygame.Rect(xcoord, ycoord, 5*PX_PER_CM, 5*PX_PER_CM))
            elif objecttype == 4: #mountain
                #print(f"object is a {objectproperties[0]} at coordinates ({objectproperties[1]}, {objectproperties[2]})")
                    #pygame.draw.rect(screen, "gray", pygame.Rect(int(objectproperties[0])+WIDTH/2, int(objectproperties[1])+HEIGHT/2, 150, 150))
                mountainpoints.append([xcoord, ycoord])
                #if the new point is the first, or too far from other mountain points, create a new mountain
            elif objecttype == 0: #no point of interest (POI)
                if objectproperties[2]==1:
                    robot1pos = [xcoord, ycoord]
                elif objectproperties[2]==2:
                    robot2pos = [xcoord, ycoord]
                
                #print(f"object is a robot at coordinates ({objectproperties[0]}, {objectproperties[1]})")
                pass
            elif objecttype == 3: #tape
                linepoints.append([xcoord, ycoord])
                linepoints = sort_linepoints(linepoints)
                #print(linepoints)
            
        #pygame.draw.polygon(screen, "black", linepoints, 4)
        for p in linepoints:
            pygame.draw.circle(screen, "black", p, 4)
        for m in mountainpoints:
            pygame.draw.circle(screen, "gray", m, 4)
        if robot2pos != []:
            screen.blit(robot_sprite, (robot2pos[0] - sprite_width // 2, robot2pos[1] - sprite_height // 2))
        if robot1pos != []:
            screen.blit(robot_sprite, (robot1pos[0] - sprite_width // 2, robot1pos[1] - sprite_height // 2))

        
            

            

    

    # flip() the display to put your work on screen
    pygame.display.flip()

    clock.tick(4)  # limits refresh rate
pygame.quit()