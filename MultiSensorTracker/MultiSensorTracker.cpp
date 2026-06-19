#include <iostream>
#include <string>
#include <cmath>

using namespace std;

class Target {
private:
    string id;
    double x;
    double y;
    double vx;
    double vy;

public:
    Target(string targetId, double posX, double posY, double velocityX, double velocityY)
        : id(targetId), x(posX), y(posY), vx(velocityX), vy(velocityY) {
    }

    double speed() const {
        return sqrt(vx * vx + vy * vy);
    }

    void printInfo() const {
        cout << "Target ID: " << id << "\n";
        cout << "Position: (" << x << ", " << y << ")\n";
        cout << "Velocity: (" << vx << ", " << vy << ")\n";
        cout << "Speed: " << speed() << "\n";
    }
};

int main() {

    cout << "=== Multi Sensor Tracker ===\n";

    Target target("T1", 10.0, 20.0, 1.0, 0.0);
    target.printInfo();

    return 0;
}