#include <math.h>

#define PI 3.141592653589793238
namespace helper {
    bool isNear(float a, float b, float d) {
        return b-d <= a && a <= b+d;
    }

    bool pointInRange(float low, float high, float point) {
        return low <= point && point <= high;
    }

    int subTo(int n, int lim) {
        return n > lim ? n - 1 : 0;
    }

    int subZero(int n) {
        return n > 0 ? n - 1 : 0;
    }

    float subZero(float x) {
        return x > 0 ? x - 1 : 0;
    }

    float absf(float x) {
        return x < 0 ? -x : x;
    }

    float dmax(float a, float b) {
        return a > b ? a : b;
    }

    int radToDeg(float r) {
        return 180*r/PI;
    }

    float degToRad(float d) {
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

    float wrap(float value, float max) {
        if (value > max) {
            return value - max;
        } else {
            return value;
        }
    }

    bool angleBetween(double a, int lower_a, int upper_a) {
        if (lower_a < upper_a) {
            return lower_a <= a && a <= upper_a;
        } else {
            return (lower_a <= a && a <= 360+upper_a) || (lower_a-360 <= a && a <= upper_a);
        }
    }
}
