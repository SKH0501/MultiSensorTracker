#include <iostream>
#include <string>
#include <cmath>

using namespace std;

const double PI = 3.14159265358979323846;
const double RADAR_WEIGHT = 0.7;
const double EO_WEIGHT = 0.3;

enum class SensorType {
    RADAR,
    EO
};

struct Coordinate {
    double x;
    double y;

    Coordinate(double posX, double posY)
        : x(posX), y(posY) {
    }

    void print() const {
        cout << "(" << x << ", " << y << ")";
    }
};

struct SensorData {
    int time;
    SensorType sensorType;
    string targetId;
    double value1;
    double value2;

    SensorData(int inputTime, SensorType inputSensorType, string inputTargetId, double inputValue1, double inputValue2)
        : time(inputTime),
        sensorType(inputSensorType),
        targetId(inputTargetId),
        value1(inputValue1),
        value2(inputValue2) {
    }
};

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

double degreeToRadian(double degree) {
    return degree * PI / 180.0;
}

Coordinate convertRadarToCoordinate(const SensorData& radarData) {
    double range = radarData.value1;
    double bearingDegree = radarData.value2;
    double bearingRadian = degreeToRadian(bearingDegree);

    double x = range * cos(bearingRadian);
    double y = range * sin(bearingRadian);

    return Coordinate(x, y);
}

Coordinate convertEoToCoordinate(const SensorData& eoData) {
    double x = eoData.value1;
    double y = eoData.value2;

    return Coordinate(x, y);
}

Coordinate fuseCoordinates(const Coordinate& radarCoordinate, const Coordinate& eoCoordinate) {
    double fusedX = radarCoordinate.x * RADAR_WEIGHT + eoCoordinate.x * EO_WEIGHT;
    double fusedY = radarCoordinate.y * RADAR_WEIGHT + eoCoordinate.y * EO_WEIGHT;

    return Coordinate(fusedX, fusedY);
}

int main() {
    cout << "=== Multi Sensor Tracker ===\n\n";

    Target target("T1", 10.0, 20.0, 1.0, 0.0);
    target.printInfo();

    cout << "\n=== RADAR Coordinate Conversion Test ===\n";

    SensorData radarData(0, SensorType::RADAR, "T1", 100.0, 30.0);
    Coordinate radarCoordinate = convertRadarToCoordinate(radarData);

    cout << "Input RADAR Data\n";
    cout << "time: " << radarData.time << "\n";
    cout << "target_id: " << radarData.targetId << "\n";
    cout << "range: " << radarData.value1 << "\n";
    cout << "bearing: " << radarData.value2 << " degrees\n";

    cout << "\nConverted RADAR Coordinate\n";
    cout << "x, y = ";
    radarCoordinate.print();
    cout << "\n";

    cout << "\n=== Sensor Fusion Test ===\n";

    SensorData eoData(0, SensorType::EO, "T1", 85.0, 50.0);
    Coordinate eoCoordinate = convertEoToCoordinate(eoData);

    cout << "RADAR Coordinate: ";
    radarCoordinate.print();
    cout << "\n";

    cout << "EO Coordinate: ";
    eoCoordinate.print();
    cout << "\n";

    Coordinate fusedCoordinate = fuseCoordinates(radarCoordinate, eoCoordinate);

    cout << "\nFused Coordinate: ";
    fusedCoordinate.print();
    cout << "\n";

    return 0;
}