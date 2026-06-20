#include <iostream>
#include <string>
#include <cmath>

using namespace std;

const double PI = 3.14159265358979323846;
const double RADAR_WEIGHT = 0.7;
const double EO_WEIGHT = 0.3;

const double WATCH_X_THRESHOLD = 85.0;
const double WATCH_Y_THRESHOLD = 50.0;
const double WARNING_X_THRESHOLD = 90.0;
const double WARNING_Y_THRESHOLD = 55.0;
const double WARNING_SPEED_THRESHOLD = 6.0;

enum class SensorType {
    RADAR,
    EO
};

enum class TargetStatus {
    NORMAL,
    WATCH,
    WARNING
};

string statusToString(TargetStatus status) {
    switch (status) {
    case TargetStatus::NORMAL:
        return "NORMAL";
    case TargetStatus::WATCH:
        return "WATCH";
    case TargetStatus::WARNING:
        return "WARNING";
    default:
        return "UNKNOWN";
    }
}

struct Coordinate {
    double x;
    double y;

    Coordinate() : x(0.0), y(0.0) {
    }

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
    Coordinate previousPosition;
    Coordinate currentPosition;
    int previousTime;
    int currentTime;
    bool hasPreviousPosition;

public:
    Target(string targetId)
        : id(targetId),
        previousPosition(),
        currentPosition(),
        previousTime(0),
        currentTime(0),
        hasPreviousPosition(false) {
    }

    void updatePosition(const Coordinate& newPosition, int time) {
        if (!hasPreviousPosition) {
            currentPosition = newPosition;
            currentTime = time;
            hasPreviousPosition = true;
            return;
        }

        previousPosition = currentPosition;
        previousTime = currentTime;

        currentPosition = newPosition;
        currentTime = time;
    }

    double calculateSpeed() const {
        if (!hasPreviousPosition || currentTime == previousTime) {
            return 0.0;
        }

        double dx = currentPosition.x - previousPosition.x;
        double dy = currentPosition.y - previousPosition.y;
        double distance = sqrt(dx * dx + dy * dy);
        double deltaTime = currentTime - previousTime;

        return distance / deltaTime;
    }

    Coordinate getCurrentPosition() const {
        return currentPosition;
    }

    string getId() const {
        return id;
    }

    void printTrackingInfo() const {
        cout << "Target ID: " << id << "\n";

        cout << "Previous Position: ";
        previousPosition.print();
        cout << "\n";

        cout << "Current Position: ";
        currentPosition.print();
        cout << "\n";

        cout << "Speed: " << calculateSpeed() << "\n";
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

TargetStatus evaluateStatus(const Coordinate& position, double speed) {
    if (position.x >= WARNING_X_THRESHOLD || position.y >= WARNING_Y_THRESHOLD || speed >= WARNING_SPEED_THRESHOLD) {
        return TargetStatus::WARNING;
    }

    if (position.x >= WATCH_X_THRESHOLD || position.y >= WATCH_Y_THRESHOLD) {
        return TargetStatus::WATCH;
    }

    return TargetStatus::NORMAL;
}

int main() {
    cout << "=== Multi Sensor Tracker ===\n\n";

    cout << "=== Sensor Fusion Test: time 0 ===\n";

    SensorData radarDataTime0(0, SensorType::RADAR, "T1", 100.0, 30.0);
    SensorData eoDataTime0(0, SensorType::EO, "T1", 85.0, 50.0);

    Coordinate radarCoordinateTime0 = convertRadarToCoordinate(radarDataTime0);
    Coordinate eoCoordinateTime0 = convertEoToCoordinate(eoDataTime0);
    Coordinate fusedCoordinateTime0 = fuseCoordinates(radarCoordinateTime0, eoCoordinateTime0);

    cout << "RADAR Coordinate: ";
    radarCoordinateTime0.print();
    cout << "\n";

    cout << "EO Coordinate: ";
    eoCoordinateTime0.print();
    cout << "\n";

    cout << "Fused Coordinate: ";
    fusedCoordinateTime0.print();
    cout << "\n\n";

    cout << "=== Sensor Fusion Test: time 1 ===\n";

    SensorData radarDataTime1(1, SensorType::RADAR, "T1", 105.0, 32.0);
    SensorData eoDataTime1(1, SensorType::EO, "T1", 89.0, 52.0);

    Coordinate radarCoordinateTime1 = convertRadarToCoordinate(radarDataTime1);
    Coordinate eoCoordinateTime1 = convertEoToCoordinate(eoDataTime1);
    Coordinate fusedCoordinateTime1 = fuseCoordinates(radarCoordinateTime1, eoCoordinateTime1);

    cout << "RADAR Coordinate: ";
    radarCoordinateTime1.print();
    cout << "\n";

    cout << "EO Coordinate: ";
    eoCoordinateTime1.print();
    cout << "\n";

    cout << "Fused Coordinate: ";
    fusedCoordinateTime1.print();
    cout << "\n\n";

    cout << "=== Target Tracking Test ===\n";

    Target target("T1");
    target.updatePosition(fusedCoordinateTime0, 0);
    target.updatePosition(fusedCoordinateTime1, 1);

    target.printTrackingInfo();

    Coordinate currentPosition = target.getCurrentPosition();
    double speed = target.calculateSpeed();
    TargetStatus status = evaluateStatus(currentPosition, speed);

    cout << "Status: " << statusToString(status) << "\n";

    return 0;
}