#include "allcode_api.h"

/**
 *Compass direction stored as an enum. e.g. north = 0, south = 2
 */
typedef enum Navigation {
    north, east, south, west
} Compass;
Compass compass;

/**
 * The first 4 ints state whether a wall is present in that location (0 = no 
 * wall, 1 = wall)
 * The next boolean states whether the node has been visited (false = not 
 * visited, true = visited)
 * The final boolean is a flag used to detect if the node is a nesting area 
 * (false = not a nesting area, true = nesting area)
 */
typedef struct mazeNode {
    int north;
    int east;
    int south;
    int west;
    bool visited;
    bool nestingArea;
} node;
node maze [5][5]; //2D array to store made nodes

/* These are used for the compass orientation in terms of x(horizontal) and 
 * y(vertical) in the nodes
 */
static int x = 2;
static int y = 0;

//a counter for the total number of lines it has crossed
static int lines = 0;

//a counter for number of nesting areas present (Should not be over 4)
static int nestingAreas = 0;

/**
 * This function detects when a wall is present and prioritises turning left if
 * the wall is clear otherwise it will go forwards or right. It also changes the 
 * compass orientation and uses modulus to make sure the value never exceed 3
 */
void detectWall() {

    int ir_front = FA_ReadIR(IR_FRONT);
    int ir_left = FA_ReadIR(IR_LEFT);
    int ir_right = FA_ReadIR(IR_RIGHT);

    if (ir_left < 100) {
        compass = (compass + 3) % 4;
        FA_Left(93);
        FA_DelayMillis(50);
    } else if (ir_front < 100) {
        FA_DelayMillis(50);
    } else if (ir_right < 100) {
        compass = (compass + 1) % 4;
        FA_Right(93);
        FA_DelayMillis(50);
    } else {
        compass = (compass + 2) % 4;
        FA_Right(192);
        FA_DelayMillis(50);
    }
}

/**
 * Responsible for detecting if the robot is misaligned and adjusts its course 
 * by 5 degrees, if it is too close to the wall. 
 */
void avoidWalls() {
    
    int ir_front = FA_ReadIR(IR_FRONT);
    int ir_front_left = FA_ReadIR(IR_FRONT_LEFT);
    int ir_front_right = FA_ReadIR(IR_FRONT_RIGHT);
    
    if (ir_front_left > 200) {
        FA_Right(5);
    } else if (ir_front_right > 200) {
        FA_Left(5);
    } else if (ir_front > 300) {
        FA_Backwards(10);
    }
}

/**
 * Changes the current position of the robot in the 2D array depending upon the 
 * orientation of the compass
 */
void currentPosition() {
    switch (compass) {
        case 0:
            x++;
            break;
        case 1:
            y--;
            break;
        case 2:
            x--;
            break;
        case 3:
            y++;
            break;
    }
}

/**
 * Detects if there is a line underneath the robot. If there is a line detected
 * then the function moves the robot forwards by 100mm, updating the x and y 
 * compasses
 * @return 1 if the robot detects a line; 0 if there is no line detected
 */
int detectLine() {
    static int state = 0;
    int avgLine = (FA_ReadLine(0) + FA_ReadLine(1)) / 2;
    switch (state) {
        case 0:
            if (avgLine > 150) {
                state = 1;
            }
            break;
        case 1:
            if (avgLine < 10) {
                state = 2;
            }
            break;
        case 2:
            if (avgLine > 150) {
                state = 0;
                currentPosition();
                FA_Forwards(100);
                return 1;
            }
            break;
    }
    return 0;
}

/**
 * Debug statement which uses the bluetooth connection to send information about
 * the current node to the receiver.
 */
void printNode() {
    FA_BTSendString("Lines counted: ", 15);
    FA_BTSendNumber(lines);
    FA_BTSendByte('\n');
    FA_BTSendString("Compass: ", 11);
    FA_BTSendNumber(compass);
    FA_BTSendByte('\n');
    FA_BTSendString("Array Positions (x,y): ", 23);
    FA_BTSendNumber(x);
    FA_BTSendString(",", 1);
    FA_BTSendNumber(y);
    FA_BTSendByte('\n');
    //These lines below send data about the node walls and the nesting area
    FA_BTSendString("North: ", 7);
    FA_BTSendNumber(maze[x][y].north);
    FA_BTSendByte('\n');
    FA_BTSendString("East: ", 6);
    FA_BTSendNumber(maze[x][y].east);
    FA_BTSendByte('\n');
    FA_BTSendString("South: ", 7);
    FA_BTSendNumber(maze[x][y].south);
    FA_BTSendByte('\n');
    FA_BTSendString("West: ", 6);
    FA_BTSendNumber(maze[x][y].west);
    FA_BTSendByte('\n');
    FA_BTSendString("Nesting area: ", 14);
    FA_BTSendNumber(maze[x][y].nestingArea);
    FA_BTSendByte('\n');
}

/**
 * Called after a line has been detected and a nodes data has been read. If the 
 * light intensity is low then the robots LEDs turn on for 1 second and 
 * then switch off to indicate that the node has been recognised as a nesting 
 * area. This is also displayed in the printNode() function
 */
void checkNestingArea() {
    if (FA_ReadLight() < 450 && maze[x][y].nestingArea == false) {
        nestingAreas++;
        maze[x][y].nestingArea = true;
        int i = 0;
        while (i <= 7) {
            FA_LEDOn(i);
            i++;
        }
        FA_DelayMillis(1000);
        while (i > -1) {
            FA_LEDOff(i);
            i--;
        }
    }
}

/**
 * Reads the data in the cell by checking the 4 main sensors around the robot 
 * and then updates the compass orientation followed by assigning walls in the 
 * maze to a value of (1 = wall, 0 = no wall). It then marks the node as 
 * visited, the nesting area set to false by default and then checks if the node
 * is a nesting area.
 */
void readNode() {
    static int state = 0;
    FA_DelayMillis(350);

    int direction[4] = {north, east, south, west};

    int compassDir = 0 + compass;
    int i = 0;
    while (i < 4) {

        int ir_front = FA_ReadIR(IR_FRONT);
        int ir_left = FA_ReadIR(IR_LEFT);
        int ir_right = FA_ReadIR(IR_RIGHT);
        int ir_back = FA_ReadIR(IR_REAR);

        switch (state) {
            case 0:
                if (ir_front < 100) {
                    direction[(compassDir) % 4] = 0;
                } else {
                    direction[(compassDir) % 4] = 1;
                }
                state = 1;
                break;
            case 1:
                if (ir_right < 100) {
                    direction[(compassDir + 1) % 4] = 0;
                } else {
                    direction[(compassDir + 1) % 4] = 1;
                }
                state = 2;
                break;
            case 2:
                if (ir_back < 100) {
                    direction[(compassDir + 2) % 4] = 0;
                } else {
                    direction[(compassDir + 2) % 4] = 1;
                }
                state = 3;
                break;
            case 3:
                if (ir_left < 100) {
                    direction[(compassDir + 3) % 4] = 0;
                } else {
                    direction[(compassDir + 3) % 4] = 1;
                }
                state = 0;
                break;
        }
        i++;
    }
    maze[x][y].north = direction[compassDir];
    maze[x][y].east = direction[(compassDir + 1) % 4];
    maze[x][y].south = direction[(compassDir + 2) % 4];
    maze[x][y].west = direction[(compassDir + 3) % 4];
    maze[x][y].visited = true;
    maze[x][y].nestingArea = false;
    checkNestingArea();
}

/**
 * Called once when the robot is initialised to mark all the cells as not 
 * visited
 */
void markAllNodesUnvisited() {
    int i = 0;
    int j = 0;
    while (i < 5) {
        while (j < 5) {
            maze[i][j].visited = false;
            j++;
        }
        j = 0;
        i++;
    }
}

/**
 * Checks if all the nodes have been visited
 * @return true if all nodes visited, otherwise false
 */
bool checkAllNodesVisited() {
    int i = 0;
    int j = 0;
    while (i < 5) {
        while (j < 5) {
            if (maze[i][j].visited == false) {
                return false;
            }
            j++;
        }
        j = 0;
        i++;
    }
    return true;
}

/**
 * Prints a start table on the robots LCD
 */
void printStartTable() {
    int nodeSize = 6;
    int x;
    int y;
    int x_line = 0;
    int y_line = 0;
    
    for (x = 0; x < 5; x++) {
        for (y = 0; y < 5; y++) {
            FA_LCDRectangle(x_line, y_line, x_line+nodeSize, y_line+nodeSize, 1, 0);
            x_line += nodeSize;
        }
        y_line += nodeSize;
        x_line = 0;
    }
}

/**
 * Supposed to print the final table on the LCD by iterating over each node and 
 * checking its walls
 */
void printFinalTable() {
    int nodeSize = 6;
    int x;
    int y;
    int x_line = 0;
    int y_line = 0;
    
    for (x = 0; x < 5; x++) {
        for (y = 0; y < 5; y++) {
            FA_LCDSetForeground(LCD_WHITE);
            if (maze[x][y].south == 0) {
                FA_LCDLine(x_line, y_line, x_line, y_line+nodeSize);
            }
            if (maze[x][y].west == 0) {
                FA_LCDLine(x_line, y_line, x_line+nodeSize, y_line);
            }
            if (maze[x][y].north == 0){
                FA_LCDLine(x_line+nodeSize, y_line, x_line+nodeSize, y_line+nodeSize);
            }
            if (maze[x][y].east == 0) {
                FA_LCDLine(x_line, y_line+nodeSize, x_line+nodeSize, y_line+nodeSize);
            }
            x_line += nodeSize;
        }
        y_line += nodeSize;
        x_line = 0;
    }
}

int main() {
    FA_RobotInit();
    FA_LCDBacklight(50);
    
    //Waits for a bluetooth connection to be made before starting
    FA_LCDPrint("connecting...", 13, 10, 2, FONT_NORMAL, LCD_TRANSPARENT);
    while (!FA_BTConnected()) {
    }
    FA_LCDClear();
    FA_LCDPrint("connected!", 11, 10, 2, FONT_NORMAL, LCD_TRANSPARENT);
    FA_BTSendString("connected!\n", 11);
    FA_DelayMillis(1000);
    
    FA_LCDClear();
    compass = north; //sets robot default compass direction to north
    markAllNodesUnvisited();
    printStartTable();

    while (1) {
        FA_SetMotors(30, 25);
        
        if (detectLine() == 1) {
            lines++;
            readNode();
            printNode();
            detectWall();
        }
        avoidWalls();
        
        if (checkAllNodesVisited() == true) {
            FA_SetMotors(0, 0);
            printFinalTable();
            FA_DelayMillis(10000);
            return 0;
        }
    }
    return 0;
}