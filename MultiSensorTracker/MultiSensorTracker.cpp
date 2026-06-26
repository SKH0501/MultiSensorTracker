#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

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
    WARNING,
    SENSOR_LOST
};

string sensorTypeToString(SensorType sensorType) {
    switch (sensorType) {
    case SensorType::RADAR:
        return "RADAR";
    case SensorType::EO:
        return "EO";
    default:
        return "UNKNOWN";
    }
}

SensorType parseSensorType(const string& sensorTypeText) {
    if (sensorTypeText == "RADAR") {
        return SensorType::RADAR;
    }

    if (sensorTypeText == "EO") {
        return SensorType::EO;
    }

    cout << "Unknown sensor type: " << sensorTypeText << "\n";
    return SensorType::EO;
}

string statusToString(TargetStatus status) {
    switch (status) {
    case TargetStatus::NORMAL:
        return "NORMAL";
    case TargetStatus::WATCH:
        return "WATCH";
    case TargetStatus::WARNING:
        return "WARNING";
    case TargetStatus::SENSOR_LOST:
        return "SENSOR_LOST";
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

struct SimulationInput {
    int time;
    string targetId;
    bool hasRadar;
    bool hasEo;
    SensorData radarData;
    SensorData eoData;

    SimulationInput()
        : time(0),
        targetId(""),
        hasRadar(false),
        hasEo(false),
        radarData(0, SensorType::RADAR, "", 0.0, 0.0),
        eoData(0, SensorType::EO, "", 0.0, 0.0) {
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

vector<SensorData> loadSensorDataFromCsv(const string& filePath) {
    vector<SensorData> sensorDataList;

    ifstream file(filePath);

    if (!file.is_open()) {
        cout << "Failed to open file: " << filePath << "\n";
        return sensorDataList;
    }

    string line;

    // 첫 번째 줄은 헤더이므로 건너뛴다.
    getline(file, line);

    while (getline(file, line)) {
        stringstream ss(line);

        string timeText;
        string sensorTypeText;
        string targetId;
        string value1Text;
        string value2Text;

        getline(ss, timeText, ',');
        getline(ss, sensorTypeText, ',');
        getline(ss, targetId, ',');
        getline(ss, value1Text, ',');
        getline(ss, value2Text, ',');

        int time = stoi(timeText);
        SensorType sensorType = parseSensorType(sensorTypeText);
        double value1 = stod(value1Text);
        double value2 = stod(value2Text);

        SensorData data(time, sensorType, targetId, value1, value2);
        sensorDataList.push_back(data);
    }

    file.close();

    return sensorDataList;
}

vector<SimulationInput> groupSensorDataByTimeAndTarget(const vector<SensorData>& sensorDataList) {
    map<string, SimulationInput> groupedData;

    for (const SensorData& data : sensorDataList) {
        string key = to_string(data.time) + "_" + data.targetId;

        if (groupedData.find(key) == groupedData.end()) {
            SimulationInput input;
            input.time = data.time;
            input.targetId = data.targetId;
            groupedData[key] = input;
        }

        if (data.sensorType == SensorType::RADAR) {
            groupedData[key].radarData = data;
            groupedData[key].hasRadar = true;
        }
        else if (data.sensorType == SensorType::EO) {
            groupedData[key].eoData = data;
            groupedData[key].hasEo = true;
        }
    }

    vector<SimulationInput> result;

    for (const auto& pair : groupedData) {
        result.push_back(pair.second);
    }

    return result;
}

void printCsvLoadTest(const vector<SensorData>& sensorDataList) {
    cout << "\n=== CSV Load Test ===\n";

    for (const SensorData& data : sensorDataList) {
        cout << "time=" << data.time
            << ", sensor=" << sensorTypeToString(data.sensorType)
            << ", target=" << data.targetId
            << ", value1=" << data.value1
            << ", value2=" << data.value2
            << "\n";
    }
}

void runSimulationFromCsv(const string& filePath) {
    vector<SensorData> sensorDataList = loadSensorDataFromCsv(filePath);
    vector<SimulationInput> simulationInputs = groupSensorDataByTimeAndTarget(sensorDataList);

    map<string, Target> targets;

    cout << "\n=== CSV Simulation Test ===\n";

    for (const SimulationInput& input : simulationInputs) {
        if (!input.hasRadar || !input.hasEo) {
            cout << "[time=" << input.time << "] target=" << input.targetId
                << " status=SENSOR_LOST\n";
            continue;
        }

        Coordinate radarCoordinate = convertRadarToCoordinate(input.radarData);
        Coordinate eoCoordinate = convertEoToCoordinate(input.eoData);
        Coordinate fusedCoordinate = fuseCoordinates(radarCoordinate, eoCoordinate);

        if (targets.find(input.targetId) == targets.end()) {
            targets.emplace(input.targetId, Target(input.targetId));
        }

        Target& target = targets.at(input.targetId);
        target.updatePosition(fusedCoordinate, input.time);

        double speed = target.calculateSpeed();
        TargetStatus status = evaluateStatus(target.getCurrentPosition(), speed);

        cout << "[time=" << input.time << "] "
            << "target=" << input.targetId << " "
            << "fused=";
        fusedCoordinate.print();
        cout << " speed=" << speed
            << " status=" << statusToString(status)
            << "\n";
    }
}

int main() {
    cout << "=== Multi Sensor Tracker ===\n";

    string filePath = "..\\data\\normal_case.csv";

    vector<SensorData> sensorDataList = loadSensorDataFromCsv(filePath);

    printCsvLoadTest(sensorDataList);

    runSimulationFromCsv(filePath);

    return 0;
}