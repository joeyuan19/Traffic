#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctime>
#include <random>
#include <SDL2/SDL.h>

#include "helper.h"


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * 2D traffic simulator
 * To Do:
 *      [+] Lane Changes
 *      [ ] Car AI
 *              [ ] Merging
 *                      [+] Look for merge when slowed to less than 90% max speed
 *                      [ ] Merge when at least two cars away from any cars in the lane
 *                      [ ] Wait time after merge, list of refresh frames to wait, subtract on each pass, only merge if = 0
 *      [ ] Fancy cars
 *              [+] Headlights
 *              [+] Taillights
 *              [+] Flat 2D look
 *                      [+] wheels
 *                      [+] windows
 *                      [+] Roof Shape
 *              [+] Spinning wheels
 *              [ ] Blinkers
 *              [ ] Brake Lights 
 *      [ ] Acceleration/Braking
 *      [ ] Two Way Traffic
 *              [ ] Merging allowed in other lane
 *      [ ] Lane Block: Construction
 *      [ ] Traffic light
 *      [ ] Stop sign
 *      [ ] Rain
 *      [ ] First person view
 *      [ ] Curved Road
 *      [ ] Any non-linear (follow equation?)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

using namespace std;

const int DELAY_WAIT = 30;

const int SCREEN_WIDTH = 1200;
const int SCREEN_HEIGHT = 300;

const int LANES = 3 + 1;
const int LANE_WIDTH = SCREEN_HEIGHT/LANES;
const int DASH_LENGTH = max(1,SCREEN_WIDTH/100);
const int DASH_WIDTH = max(1,SCREEN_HEIGHT/100);
const int DASH_SPACE = DASH_LENGTH;

const int CARS = 10;
const int TRACK_UNIT = SCREEN_WIDTH/CARS;
const int CAR_HEIGHT = LANE_WIDTH/2;
const int CAR_LENGTH = CAR_HEIGHT*2;
const int LIGHT_SIZE = CAR_HEIGHT/4;
const int CAR_WHEEL_RADIUS = CAR_HEIGHT/4;
const int CAR_WHEEL_RIM_DIVS = 5;
const float CAR_WHEEL_RIM_DIVS_ANGLE = 360./CAR_WHEEL_RIM_DIVS;

float carPositionX[CARS];
float carPositionY[CARS];
float carVelocityX[CARS];
float carVelocityY[CARS];
float carVelocity[CARS];
int carVelocityAngle[CARS];
float carMaxVelocity[CARS];
float carWheelAngle[CARS];
int carMerging[CARS];
int carMergeWait[CARS];
int carLane[CARS];
bool carBrakes[CARS];
int carBlinkers[CARS];
const float DT = 1;
const float MIN_VELOCITY = 5;
const float MAX_VELOCITY = MIN_VELOCITY*5;
const float PERCENT_FOR_MERGE = .5;
const float PERCENT_SPEED_FOR_MERGE = .25;
const int MERGE_WAIT_TIME= 20;
const int BLINKER_TIME = 10;

/* Log data */

void deletePositionFile() {
    remove("position");
}

void writePositionData(int n, int pos) {
    ofstream ofs;
    ofs.open("position",ofstream::app);
    ofs << n << "," << pos << endl;
    ofs.close();
}

void writeMergeLimit(int limit) {
    ofstream ofs;
    ofs.open("position",ofstream::app);
    ofs << limit << endl;
    ofs.close();
}
void writeClearanceData(int cars, int lanes, float t) {
    ofstream ofs;
    ofs.open("clearance_time",ofstream::app);
    ofs << cars << "," << lanes << "," << t << endl;
    ofs.close();
}


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

        window = SDL_CreateWindow("2D Traffic", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
        if ( window == NULL ) {
            printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
            success = false;
        } else {
            renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
            SDL_SetRenderDrawColor(renderer,0xFF,0xFF,0xFF,0xFF);
            SDL_SetRenderDrawBlendMode(renderer,SDL_BLENDMODE_BLEND);
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

int laneY(int laneNumber) {
    return LANE_WIDTH/2 + (laneNumber-1)*LANE_WIDTH;
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
            a = helper::angleDegPoint(j,i);
            if (i*i + j*j <= r*r && helper::angleBetween(a,a1,a2)) {
                SDL_RenderDrawPoint(renderer,x+i,y+j);
            }
        }
    }
}

void filledFadingCircleSlice(int x, int y, int r, int a1, int a2, int red, int grn, int blu) {
    int i, j;
    double a, dr;
    for (i = -r; i <= r; i++) {
        for (j = -r; j <= r; j++) {
            a = helper::angleDegPoint(j,i);
            if ((dr = i*i + j*j) <= r*r && helper::angleBetween(a,a1,a2)) {
                SDL_SetRenderDrawColor(renderer,red,grn,blu,0xFF*(1-dr/(r*r)));
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
            if (helper::inTriangle(x,y,x1,y1,x2,y2,x3,y3)) {
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
    for (lane = 1; lane < lanes-1; lane++) {
        for (dash = 0; dash < SCREEN_WIDTH; dash += 2*DASH_LENGTH) {
            const SDL_Rect rect = {dash, LANE_WIDTH*lane - DASH_WIDTH/2, DASH_LENGTH, DASH_WIDTH}; 
            SDL_RenderFillRect(renderer, &rect);
        }
    }
    const SDL_Rect rect1 = {0, LANE_WIDTH*lane - DASH_WIDTH/2, SCREEN_WIDTH/3, DASH_WIDTH}; 
    SDL_RenderFillRect(renderer, &rect1);
    for (dash = SCREEN_WIDTH/3; dash < SCREEN_WIDTH/2; dash += 2*DASH_LENGTH) {
        const SDL_Rect rect2 = {dash, LANE_WIDTH*lane - DASH_WIDTH/2, DASH_LENGTH, DASH_WIDTH}; 
        SDL_RenderFillRect(renderer, &rect2);
    }
    const SDL_Rect rect3 = {3*SCREEN_WIDTH/4, LANE_WIDTH*lane - DASH_WIDTH/2, SCREEN_WIDTH/4, DASH_WIDTH}; 
    SDL_RenderFillRect(renderer, &rect3);
    const SDL_Rect rect4 = {3*SCREEN_WIDTH/4, LANE_WIDTH*lane - DASH_WIDTH/2, SCREEN_WIDTH/4, LANE_WIDTH}; 
    SDL_RenderFillRect(renderer, &rect4);
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
        fillCircleSlice(x,y,r/2.,helper::wrap(a+n*CAR_WHEEL_RIM_DIVS_ANGLE,360),helper::wrap(a+(n+.5)*CAR_WHEEL_RIM_DIVS_ANGLE,360));
    }
    SDL_SetRenderDrawColor(renderer,0xFF,0xFF,0xFF,0xFF);
    fillCircle(x,y,r/8.);
}

void drawCar(int n) {
    int X = carPositionX[n] - CAR_LENGTH/2;
    int Y = carPositionY[n] - CAR_HEIGHT/2;
    SDL_SetRenderDrawColor(renderer,0x00,0xFF*(1.0 - 1.0*n/CARS),0x55,0xFF);
    if (n == 5) {
        SDL_SetRenderDrawColor(renderer,0xFF,0x00,0x00,0xFF);
    }

    // Roof
    const SDL_Rect roof = {X + 3*CAR_LENGTH/8, Y, CAR_LENGTH/4, CAR_HEIGHT/2}; 
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
    
    SDL_SetRenderDrawColor(renderer,0x00,0xFF*(1.0 - 1.0*n/CARS),0x55,0xFF);
    if (n == 5) {
        SDL_SetRenderDrawColor(renderer,0xFF,0x00,0x00,0xFF);
    }

    // Body
    const SDL_Rect body = {X, Y + CAR_HEIGHT/4, CAR_LENGTH, CAR_HEIGHT/2}; 
    SDL_RenderFillRect(renderer, &body);
    
    // Blinker
    const SDL_Rect bl = {X, Y + CAR_HEIGHT/4, LIGHT_SIZE, LIGHT_SIZE}; 
    if (carBlinkers[n] > BLINKER_TIME) {
        SDL_SetRenderDrawColor(renderer,0xFF,0xFF,0x00,0xFF);
    } else {
        SDL_SetRenderDrawColor(renderer,0xAA,0xAA,0x00,0xFF);
    }
    SDL_RenderFillRect(renderer, &bl);

    // Taillight
    const SDL_Rect tl = {X, Y + CAR_HEIGHT/4, LIGHT_SIZE, LIGHT_SIZE/2}; 
    if (carBrakes[n]) {
        SDL_SetRenderDrawColor(renderer,0xFF,0x00,0x00,0xFF);
    } else {
        SDL_SetRenderDrawColor(renderer,0xAA,0x00,0x00,0xFF);
    }
    SDL_RenderFillRect(renderer, &tl);
    
    // Headlight
    const SDL_Rect hl = {X+CAR_LENGTH-LIGHT_SIZE, Y+CAR_HEIGHT/4, LIGHT_SIZE, LIGHT_SIZE}; 
    SDL_SetRenderDrawColor(renderer,0xFF,0xFF,0xFF,0xFF);
    SDL_RenderFillRect(renderer, &hl);
    filledFadingCircleSlice(X+CAR_LENGTH-LIGHT_SIZE, Y+CAR_HEIGHT/4+LIGHT_SIZE/2, 3*LIGHT_SIZE, 330, 30, 0xFF, 0xFF, 0xFF);
    
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

/* Driving AI */

bool carBehind(float fl, float fh, float bl, float bh) {
    return helper::pointInRange(fl,fh,bl) || helper::pointInRange(fl,fh,bh);
}

bool carBehind(int backCar, int frontCar) {
    float fl = carPositionY[frontCar] - CAR_HEIGHT/2,
          fh = carPositionY[frontCar] + CAR_HEIGHT/2,
          bl = carPositionY[backCar] - CAR_HEIGHT/2,
          bh = carPositionY[backCar] + CAR_HEIGHT/2;
    return helper::pointInRange(fl,fh,bl) || helper::pointInRange(fl,fh,bh);
}

bool carNextTo(int backCar, int frontCar) {
    float tl = carPositionX[frontCar] - CAR_LENGTH/2,
          tr = carPositionX[frontCar] + CAR_LENGTH/2,
          bl = carPositionX[backCar] - CAR_LENGTH/2,
          br = carPositionX[backCar] + CAR_LENGTH/2;
    return helper::pointInRange(tl,tr,br) || helper::pointInRange(tl,tr,bl);
}

void connectCars(int carA, int carB) {
    switch (carA) {
        case 0:
            SDL_SetRenderDrawColor(renderer,0x00,0xFF,0x00,0xFF);
            break;
        case 1:
            SDL_SetRenderDrawColor(renderer,0x00,0x00,0xFF,0xFF);
            break;
        case 2:
            SDL_SetRenderDrawColor(renderer,0xFF,0x00,0x00,0xFF);
            break;
        default:
            SDL_SetRenderDrawColor(renderer,0x00,0xFF*(1.0 - 1.0*carA/CARS),0x55,0xFF);
            break;
    }
    SDL_RenderDrawLine(renderer,carPositionX[carA],carPositionY[carA],carPositionX[carB],carPositionY[carB]);
}

float s2s(int carA, int carB) {
    // Side to side distance + small padding
    float y1 = carPositionY[carA], y2 = carPositionY[carB];
    float dy = y2 - y1;
    return dy + (dy < 0 ? +1 : -1)*(CAR_HEIGHT + 2);
}

float b2b(int carA, int carB) {
    // Bumper to bumper distance + small padding
    float x1 = carPositionX[carA], x2 = carPositionX[carB];
    if (x1 > x2) {
        x2 = x2 + SCREEN_WIDTH;
    }
    return x2 - x1 - (CAR_LENGTH + 2);
}

float * nextCarX(int n) {
    float min_i = n, min = SCREEN_WIDTH*2;
    float dx;
    if (carLane[n] != LANES) {
        for (int c = 0; c < CARS; c++) {
            if (c != n && carBehind(n,c)) {
                if ((dx = b2b(n,c)) < min) {
                    min_i = c;
                    min = dx;
                }
            }
        }
    } else {
        float x1 = carPositionX[n], x2, dx;
        for (int c = 0; c < CARS; c++) {
            if (c != n && carLane[c] == LANES) {
                x2 = carPositionX[c];
                if (x1 < x2) {
                    dx = x2 - x1 - (CAR_LENGTH + 2);
                    if ((dx = b2b(n,c)) < min) {
                        min_i = c;
                        min = dx;
                    }
                }
            }
        }
        if ((dx = (2*SCREEN_WIDTH/3 - carPositionX[n] - CAR_LENGTH - 2)) < min) {
            min_i = -1;
            min = dx;
        }
    }
    return new float[2]{min_i,min};
}

float * nextCarY(int n, bool above) {
    int min_i = n, min = SCREEN_HEIGHT*2;
    float dy;
    for (int c = 0; c < CARS; c++) {
        if (c != n && carNextTo(n,c)) {
            if ((above && carPositionY[n] < carPositionY[c]) ||
                    (!above && carPositionY[n] > carPositionY[c])) {
                if ((dy = s2s(n,c)) < min) {
                    min_i = c;
                    min = dy;
                }
            }
        }
    }
    return new float[2]{(float)min_i, dy};
}

bool checkLaneClear(int x, int newY) {
    if (newY > SCREEN_HEIGHT-LANE_WIDTH || newY < 0) {
        return false;
    }
    float yl = newY - CAR_HEIGHT/2,
        yh = newY + CAR_HEIGHT/2,
        xmin = x - 2*CAR_LENGTH,
        xmax = x + 2*CAR_LENGTH;
    float x2, x2min, x2max;
    for (int c = 0; c < CARS; c++) {
        if (carBehind(yl,yh,carPositionY[c] - CAR_HEIGHT/2,carPositionY[c] + CAR_HEIGHT/2)) {
            x2 = carPositionX[c];
            x2min = x2 - CAR_LENGTH/2; 
            x2max = x2 + CAR_LENGTH/2; 
            if (x2min < xmin && xmin < x2max) {
                return false; 
            } else if (x2min < xmax && xmax < x2max) {
                return false; 
            }
        }
    }
    return true;
}

/* Rendering */
bool started = false;
bool finished = false;
time_t t;
bool init() {
    deletePositionFile();
    writeMergeLimit(2*SCREEN_WIDTH/3);
    bool success = true;
    R(MIN_VELOCITY,MAX_VELOCITY);
    for (int n = 0; n < CARS; n++) {
        carPositionX[n]     = -2*n*CAR_LENGTH;
        carLane[n]          = LANES;
        carPositionY[n]     = laneY(carLane[n]); 
        carMerging[n]       = 0;
        carMergeWait[n]     = 0;
        carVelocity[n]      = 0;
        carVelocityAngle[n] = 0;
        carVelocityX[n]     = 0;
        carVelocityY[n]     = 0;
        carBlinkers[n]      = 0;
        carBrakes[n]        = 0;
        carMaxVelocity[n]   = R(MIN_VELOCITY,MAX_VELOCITY);
    }
    draw();
    return success;
} 

float x, y, vx, vy, v, vmax, vxmax, vymax;
int mw;

void update(int n) {
    // reduce memory calls
    x = carPositionX[n];
    y = carPositionY[n];
    vx = carVelocityX[n];
    vy = carVelocityY[n];
    v = sqrt(vx*vx + vy*vy);
    vmax = carMaxVelocity[n];
    vymax = 0; 
    vxmax = vmax;
    mw = carMergeWait[n];
    carPositionX[n] = helper::wrap(x + DT*vx,SCREEN_WIDTH);
    carPositionY[n] = y + DT*vy;
    float * r = nextCarX(n);
    int nc = r[0];
    float dx = r[1];

    carBrakes[n] = false;
    if (nc == n) {
        dx = 1.0;
    } else {
        if (dx > 2*CAR_LENGTH) {
            dx = 1.0;
        } else {
            dx = dx/(2*CAR_LENGTH);
            carBrakes[n] = true;
        }
    }
    carWheelAngle[n] = helper::wrap(carWheelAngle[n] + 10*v/CAR_WHEEL_RADIUS,360);
    carMergeWait[n] = helper::subZero(mw); 
    carBlinkers[n] = helper::subZero(carBlinkers[n]);
    if (carMerging[n] == 0) {
        carVelocityY[n] = 0;
        if (carLane[n] == LANES && carPositionX[n] >= SCREEN_WIDTH/3) {
            carMerging[n] = -1;
        } else if (carLane[n] != LANES && vmax*PERCENT_FOR_MERGE > v) {
            if (checkLaneClear(x,y+LANE_WIDTH)) { 
                carMerging[n] = 1;
            } else if (checkLaneClear(x,y-LANE_WIDTH)) { 
                carMerging[n] = -1;
            }
        }
    } else {
        vymax = PERCENT_SPEED_FOR_MERGE*vmax;
        vxmax = vmax*sqrt(1 - PERCENT_SPEED_FOR_MERGE);
        if (carBlinkers[n] == 0) {
            carBlinkers[n] = BLINKER_TIME*2;
        }
        int l = laneY(carLane[n]+carMerging[n]);
        float d2lane = l - carPositionY[n];
        float scale = 1.0;
        r = nextCarY(n,carMerging[n] > 0);
        nc = r[0];
        float dy = fabs(r[1]);
        if (nc == n) {
            dy = 1.0;
        } else {
            if (2*dy > CAR_HEIGHT) {
                dy = 1.0;
            } else {
                dy = 2*dy/CAR_HEIGHT;
            }
        }
        if (fabs(d2lane) <= .05*SCREEN_HEIGHT) {
            carPositionY[n] = l;
            carVelocityY[n] = 0;
            carLane[n] = carLane[n] + carMerging[n];
            carMerging[n] = 0;
            carMergeWait[n] = MERGE_WAIT_TIME;
        } else {
            float tau = d2lane/LANE_WIDTH;
            carVelocityY[n] = dy*vymax*tau/(1 + exp(fabs(tau)));
        }
    }
    carVelocityX[n] = helper::dmax(vxmax*dx,0);
}

bool render() {
    bool success = true;
    draw();
    for (int c = 0; c < CARS; c++) {
        update(c);
        if (carLane[c] == LANES) {
            writePositionData(c,carPositionX[c]);
        }
    }
    if (!finished) {
        finished = true;
        for (int c = 0; c < CARS; c++) {
            if (carLane[c] == LANES){
                finished = false;
            }
            if (!started && carLane[c] == LANES && carPositionX[c] > 0)  {
                started = true;
                t = time(nullptr);
                break;
            }
        }
        if (finished) {
            t = time(nullptr) - t;
            writeClearanceData(CARS,LANES,t);
            cout << "Finished: " << t << "seconds" << endl;
        }
    }
    SDL_Delay(DELAY_WAIT);

    SDL_RenderPresent(renderer);
    return success;
}

int main(int argc, char *argv[ ] ) {
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
