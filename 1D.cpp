#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <random>
#include <SDL2/SDL.h>

#define PI 3.141592653589793238

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *     
 * 1D traffic simulator
 * To Do:
 *      [+] Lane Changes
 *      [ ] Car AI
 *              [ ] Merging
 *                      [+] Look for merge when slowed to less than 90% max speed
 *                      [ ] Merge when at least two cars away from any cars in the lane
 *                      [ ] Wait time after merge, list of refresh frames to wait, subtract on each pass, only merge if = 0
 *      [+] Fancy cars
 *              [+] Headlights
 *              [+] Taillights
 *              [+] Flat 2D look
 *                      [+] wheels
 *                      [+] windows
 *                      [+] Roof Shape
 *              [+] Spinning wheels
 *      [ ] Acceleration/Braking
 *      [ ] Two Way Traffic
 *              [ ] Merging allowed in other lane
 *      [ ] Lane Block: Construction
 *      [ ] Traffic light, Stop sign
 *      [ ] Curved Road
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */    

using namespace std;

const int DELAY_WAIT = 0;

const int SCREEN_WIDTH = 1000;
const int SCREEN_HEIGHT = 300;

const int LANES = 3;
const int DASH_LENGTH = 20;
const int DASH_WIDTH = 1;
const int DASH_SPACE = 40;
const int ROAD_UNIT = DASH_LENGTH+DASH_SPACE;
const int LANE_WIDTH = SCREEN_HEIGHT/LANES;

const int CARS = LANES*2;
const int TRACK_UNIT = SCREEN_WIDTH/CARS;
const int CAR_HEIGHT = LANE_WIDTH/2;
const int CAR_LENGTH = CAR_HEIGHT*2;
const int LIGHT_SIZE = CAR_HEIGHT/4;
const int CAR_WHEEL_RADIUS = CAR_HEIGHT/4;
const int CAR_WHEEL_RIM_DIVS = 5;
const float CAR_WHEEL_RIM_DIVS_ANGLE = 360./CAR_WHEEL_RIM_DIVS;

float carPosition[CARS];
float carLane[CARS];
float carVelocity[CARS];
float carMaxVelocity[CARS];
float carWheelAngle[CARS];
float carMergeWait[CARS];
const float DT = 0.1;
const float MAX_VELOCITY = 50;
const float MIN_VELOCITY = 10;

/* SDL Overhead */
bool init();
bool loadMedia();
bool close();

SDL_Window * window = NULL;
SDL_Renderer * renderer = NULL;

bool initSDL() {
    bool success = true;

    if ( SDL_Init( SDL_INIT_VIDEO ) < 0 ) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        success = false;
    } else {
        if ( !SDL_SetHint( SDL_HINT_RENDER_SCALE_QUALITY, "1") ) {
            printf("Warning: Linear texture filtering not enabled!");
        }

        window = SDL_CreateWindow("SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
        if ( window == NULL ) {
            printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
            success = false;
        } else {
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
            SDL_SetRenderDrawColor(renderer,0xFF,0xFF,0xFF,0xFF);
            SDL_RenderClear(renderer);
        }
    }
    return success;
}

bool loadMedia() {
    bool success = true;
    return success;
}

bool close() {
    SDL_DestroyRenderer(renderer);
    renderer = NULL;

    SDL_DestroyWindow(window);
    window = NULL;

    SDL_Quit();

    return true;
}

/* Helper Method */

int radToDeg(float r) {
    return 180*r/PI;
}

float degToRad(int d) {
    return PI*d/180.;
}

int directionCrossProduct(int x1, int y1, int x2, int y2) {
    return 0 <= (x1*y2 - y1*x2);
}

bool sameSide(int x1, int y1, int x2, int y2, int lx1, int ly1, int lx2, int ly2) {
    return directionCrossProduct(lx2-lx1,ly2-ly1,x1-lx1,y1-ly1) && directionCrossProduct(lx2-lx1,ly2-ly1,x2-lx1,y2-ly1);
}

bool inTriangle(int x, int y, int x1, int y1, int x2, int y2, int x3, int y3) {
    return sameSide(x,y,x3,y3,x1,y1,x2,y2) && sameSide(x,y,x2,y2,x3,y3,x1,y1) && sameSide(x,y,x1,y1,x2,y2,x3,y3);
}

double angleDegPoint(float y, float x) {
    double a = atan2(y,x);
    if (a < 0) {
        a = 2*PI + a;
    }
    return radToDeg(a);
}

float wrapDeg(float a) {
    if (a > 360) {
        return a - 360;
    } else {
        return a;
    }
}

bool angleBetween(double a, int lower_a, int upper_a) {
    if (lower_a < upper_a) {
        return lower_a <= a && a <= upper_a;
    } else {
        return (lower_a <= a && a <= 360+upper_a) || (lower_a-360 <= a && a <= upper_a);
    }
}

/* Drawing funcitons */

void fillCircle(int x, int y, int r) {
    int i, j;
    for (i = -r; i <= r; i++) {
        for (j = -r; j <= r; j++) {
            if (i*i + j*j <= r*r) {
                SDL_RenderDrawPoint(renderer,x+i,y+j);
            }
        }
    }
}

void fillCircleSlice(int x, int y, int r, int a1, int a2) {
    int i, j;
    double a;
    for (i = -r; i <= r; i++) {
        for (j = -r; j <= r; j++) {
            a = angleDegPoint(j,i);
            if (i*i + j*j <= r*r && angleBetween(a,a1,a2)) {
                SDL_RenderDrawPoint(renderer,x+i,y+j);
            }
        }
    }
}

void fillTriangle(int x1, int y1, int x2, int y2, int x3, int y3) {
    int max_x = max(x1,max(x2,x3)),
        min_x = min(x1,min(x2,x3)),
        max_y = max(y1,max(y2,y3)),
        min_y = min(y1,min(y2,y3));
    int x, y;
    for (x = min_x; x <= max_x; x++) {
        for (y = min_y; y <= max_y; y++) {
            if (inTriangle(x,y,x1,y1,x2,y2,x3,y3)) {
                SDL_RenderDrawPoint(renderer,x,y);
            }
        }
    }
}

void drawRoad(int lanes) {
    SDL_SetRenderDrawColor(renderer,0x00,0x00,0x00,0xFF);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawColor(renderer,0xFF,0xFF,0x00,0xFF);
    int lane, dash;
    for (lane = 1; lane < lanes; lane++) {
        for (dash = 0; dash < SCREEN_WIDTH; dash += ROAD_UNIT) {
            const SDL_Rect rect = {dash, LANE_WIDTH*lane - DASH_WIDTH/2, DASH_LENGTH, DASH_WIDTH}; 
            SDL_RenderFillRect(renderer, &rect);
            
        }
    }
}

void drawWheel(int x, int y, int r, int a) {
    // Tire color
    SDL_SetRenderDrawColor(renderer,0x36,0x45,0x4F,0xFF);
    fillCircle(x,y,r);
    // Rim color
    SDL_SetRenderDrawColor(renderer,0xFF,0xFF,0xFF,0xFF);
    fillCircle(x,y,r/2.);
    SDL_SetRenderDrawColor(renderer,0x00,0x00,0x00,0xFF);
    for (int n = 0; n < CAR_WHEEL_RIM_DIVS; n++) {
        fillCircleSlice(x,y,r/2.,wrapDeg(a+n*CAR_WHEEL_RIM_DIVS_ANGLE),wrapDeg(a+(n+.5)*CAR_WHEEL_RIM_DIVS_ANGLE));
    }
    SDL_SetRenderDrawColor(renderer,0xFF,0xFF,0xFF,0xFF);
    fillCircle(x,y,r/8.);
}

void drawCar(int n) {
    int X = carPosition[n]-CAR_LENGTH/2;
    int Y = LANE_WIDTH/2 + (carLane[n]-1)*LANE_WIDTH - CAR_HEIGHT/2;
    SDL_SetRenderDrawColor(renderer,0x00,0xFF*(1.0 - 1.0*n/CARS),0x55,0xFF);

    // Roof
    const SDL_Rect roof = {X + 3*CAR_LENGTH/8, Y, CAR_LENGTH/4, CAR_HEIGHT/2}; 
    SDL_SetRenderDrawColor(renderer,0x00,0xFF*(1.0 - 1.0*n/CARS),0x55,0xFF);
    SDL_RenderFillRect(renderer, &roof);
    fillTriangle(X,Y+CAR_HEIGHT/2,X+3.*CAR_LENGTH/8,Y,X+3.*CAR_LENGTH/8,Y+CAR_HEIGHT/2);
    fillTriangle(X+5.*CAR_LENGTH/8,Y+CAR_HEIGHT/2,X+5*CAR_LENGTH/8,Y,X+CAR_LENGTH,Y+CAR_HEIGHT/2);
    
    // Windows
    SDL_SetRenderDrawColor(renderer,0xCC,0xCC,0xCC,0xFF);
    // Windshield
    int offset = 2; // offset from roof
    fillTriangle(X+5*CAR_LENGTH/8+offset,Y+CAR_HEIGHT/2+offset,
                 X+5*CAR_LENGTH/8+offset,Y,
                 X+CAR_LENGTH,Y+CAR_HEIGHT/2);
    // backseat window 
    const SDL_Rect bw = {X + 3*CAR_LENGTH/8, Y+offset, CAR_LENGTH/8 - 1, CAR_HEIGHT/2}; 
    SDL_RenderFillRect(renderer, &bw);
    // frontseat window 
    const SDL_Rect fw = {X + CAR_LENGTH/2 + 1, Y+offset, CAR_LENGTH/8 - 2, CAR_HEIGHT/2}; 
    SDL_RenderFillRect(renderer, &fw);
    
    // Body
    const SDL_Rect body = {X, Y + CAR_HEIGHT/4, CAR_LENGTH, CAR_HEIGHT/2}; 
    SDL_SetRenderDrawColor(renderer,0x00,0xFF*(1.0 - 1.0*n/CARS),0x55,0xFF);
    SDL_RenderFillRect(renderer, &body);
    
    // Blinker
    const SDL_Rect bl = {X, Y + CAR_HEIGHT/4, LIGHT_SIZE, LIGHT_SIZE}; 
    SDL_SetRenderDrawColor(renderer,0xFF,0xFF,0x00,0xFF);
    SDL_RenderFillRect(renderer, &bl);

    // Taillight
    const SDL_Rect tl = {X, Y + CAR_HEIGHT/4, LIGHT_SIZE, LIGHT_SIZE/2}; 
    SDL_SetRenderDrawColor(renderer,0xFF,0x00,0x00,0xFF);
    SDL_RenderFillRect(renderer, &tl);
    
    // Headlight
    const SDL_Rect hl = {X+CAR_LENGTH-LIGHT_SIZE, Y + CAR_HEIGHT/4, LIGHT_SIZE, LIGHT_SIZE}; 
    SDL_SetRenderDrawColor(renderer,0xFF,0xFF,0xFF,0xFF);
    SDL_RenderFillRect(renderer, &hl);
    
    drawWheel(X+3*CAR_LENGTH/4,Y+3*CAR_HEIGHT/4,CAR_WHEEL_RADIUS,carWheelAngle[n]);
    drawWheel(X+CAR_LENGTH/4,Y+3*CAR_HEIGHT/4,CAR_WHEEL_RADIUS,carWheelAngle[n]);
}

void drawCars() {
    for (int n = 0; n < CARS; n++) {
        drawCar(n);
    }
}

void draw() {
    drawRoad(LANES);
    drawCars();
}

/* Random numbers */

float R(int min, int max) {
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    return min + r*(max-min);
}

/* Bumper to bumper distance + small padding */

float b2b(int carA, int carB) {
    float x1 = carPosition[carA], x2 = carPosition[carB];
    if (x1 > x2) {
        x2 = x2 + SCREEN_WIDTH;
    }
    return x2 - x1 - (CAR_LENGTH + 2);
}

/* Driving AI */

int nextCar(int n) {
    int min_i = n, min = SCREEN_WIDTH*2;
    float dx;
    for (int c = 0; c < CARS; c++) {
        if (c != n && carLane[n] == carLane[c]) {
            if ((dx = b2b(n,c)) < min) {
                min_i = c;
                min = dx;
            }
        }
    }
    return min_i;
}

bool checkLaneClear(int n, int lane) {
    if (lane > LANES || lane < 1) {return false;} // No merging off road
    float x = carPosition[n];
    float xmin = x - 2*CAR_LENGTH;
    float xmax = x + 2*CAR_LENGTH;
    float x2, x2min, x2max;
    for (int c = 0; c < CARS; c++) {
        if (carLane[c] == lane) {
            x2 = carPosition[c];
            x2min = x2 - CAR_LENGTH/2; 
            x2max = x2 + CAR_LENGTH/2; 
            if (x2min < xmax && x2min > xmin) {
                return false; 
            } else if (x2max < xmax && x2max > xmin) {
                return false; 
            }
        }
    }
    return true;
}

/* Rendering */

bool init() {
    bool success = true;
    R(MIN_VELOCITY,MAX_VELOCITY);
    for (int n = 0; n < CARS; n++) {
        carPosition[n] = 5*n*CAR_LENGTH;
        carVelocity[n] = 0;
        carLane[n] = 1 + (n%LANES);
        carMaxVelocity[n] = R(MIN_VELOCITY,MAX_VELOCITY);
        carMergeWait[n] = 0;
    }
    draw();
    return success;
} 

float wrap(float pos) {
    if (pos >= SCREEN_WIDTH) {
        return pos-SCREEN_WIDTH;
    } else {
        return pos;
    } 
}

void update(int n) {
    float x1 = carPosition[n], x2 = carPosition[(n+1)%CARS];
    carPosition[n] = wrap(x1 + DT*carVelocity[n]);
    int nc = nextCar(n);
    float dx;
    if (nc == n) {
        dx = 1.0;
    } else {
        dx = b2b(n,nc);
        if (dx > 2*CAR_LENGTH) {
            dx = 1.0;
        } else {
            dx = dx/(2*CAR_LENGTH);
        }
    }
    carMergeWait[n] = carMergeWait[n] == 0 ? 0 : carMergeWait[n] - 1;
    carWheelAngle[n] = wrapDeg(carWheelAngle[n]+carVelocity[n]/CAR_WHEEL_RADIUS);
    if (carMergeWait[n] == 0 && carMaxVelocity[n]*.9 > carVelocity[n]) {
        if (checkLaneClear(n,carLane[n] + 1)) { 
            carLane[n] = carLane[n] + 1; 
            carMergeWait[n] = 50;
        } else if (checkLaneClear(n,carLane[n] - 1)) { 
            carLane[n] = carLane[n] - 1; 
            carMergeWait[n] = 50;
        }
    }
    carVelocity[n] = carMaxVelocity[n]*dx;
}

bool render() {
    bool success = true;
    for (int c = 0; c < CARS; c++) {
        update(c);
    }
    draw();
    SDL_Delay(DELAY_WAIT);

    SDL_RenderPresent(renderer);
    return success;
}

int main(int argc, char *argv[ ] )
{
    if ( !initSDL() || !init() ) {
        printf("Failed to initialize!\n");
    } else {
        if ( !loadMedia() ) {
            printf("Failed to load media!\n");
        } else {
            bool quit = false;
            SDL_Event e;
            while ( !quit ) {
                while ( SDL_PollEvent( &e ) != 0 ) {
                    if ( e.type == SDL_QUIT ) {
                        quit = true;
                    }
                }
                if (!render()){
                    quit = true;    
                }
            }
        }
    }
    
    close();
    return 0;
}
