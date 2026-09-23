// Part A: This is an extension task that requires you to decode sensor data from CAN log files.
// CAN (Controller Area Network) is a communication standard used in automotive applications (including Redback cars)
// to allow communication between sensors and controllers.
//
// Your Task: Using the signal definitions in SteeringBench.dbc, read each CAN capture in data/
// and turn it into a CSV with one row per decoded frame:
// t,u_commanded,y_measured
// eg:
// 0,15.0,0.0
// 0.005,15.0,0.0
// ...
// where t is the frame timestamp minus the first kept frame's timestamp (s), u_commanded is
// the decoded CmdAngularRate (deg/s), and y_measured is the decoded MeasuredAngle (deg).
// The above values are not real numbers; they are only there to show the expected data output format.
// Do this for all three captures:
// data/step_test.log       ->  data/step_test.csv
// data/reversal_test.log   ->  data/reversal_test.csv
// data/deadband_test.log   ->  data/deadband_test.csv
//
// The Row type, writeCsv(), and main() below are provided -- they loop the three logs, call your
// decodeLog(), and write the CSV in exactly the format above. You just need to implement decodeLog().
//
// You do not need to use any external libraries. Use the resources below to understand how to
// extract sensor data.
// Hint: Think about manual bit masking and shifting, data types required,
// what formats are used to represent values, etc.
// Resources:
// https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial
// https://www.csselectronics.com/pages/can-dbc-file-database-intro
//
// Sanity check: plot your CSVs (python3 plot_data.py) and compare against the pre-plotted
// data/*.png files -- they should match.
//
// Build & run (from the TA/ folder):
//     c++ -std=c++17 Question-A.cc -o decode
//     ./decode

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cstdint>

// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    std::vector<Row> rows;
    std::ifstream f(path);
    if (!f) return rows;

    const unsigned int TARGET = 0x200;
    bool haveFirst = false;
    double t0 = 0.0;

    std::string line;
    while (std::getline(f, line)) {
        size_t tsEnd = line.find(')');
        if (line.empty() || line[0] != '(' || tsEnd == std::string::npos) continue;
        double ts = std::stod(line.substr(1, tsEnd - 1));


        std::istringstream rest(line.substr(tsEnd + 1));
        std::string iface, frame;
        rest >> iface >> frame;

        size_t hashPos = frame.find('#');
        if (hashPos == std::string::npos) continue;

        unsigned int id = std::stoul(frame.substr(0, hashPos), nullptr, 16);
        if (id != TARGET) continue;

        std::string dataStr = frame.substr(hashPos + 1);
        std::vector<uint8_t> data;
        for (size_t i = 0; i + 1 < dataStr.size(); i += 2)
            data.push_back(static_cast<uint8_t>(std::stoul(dataStr.substr(i, 2), nullptr, 16)));

        if (data.size() < 8) continue;

        int16_t rawMeas = static_cast<int16_t>(
            static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
        
        int16_t rawCmd = static_cast<int16_t>(
            static_cast<uint16_t>(data[2]) | (static_cast<uint16_t>(data[3]) << 8));


        double y_measured  = rawMeas * 0.1;
        double u_commanded = rawCmd  * 0.1;


        if (!haveFirst) { t0 = ts; haveFirst = true; }

        Row r;
        r.t = ts - t0;
        r.u_commanded = u_commanded;
        r.y_measured  = y_measured; 
        rows.push_back(r);
        
    }

    return rows;
}

// Provided -- writes the rows to a CSV in the required format. Do not change.
void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "t,u_commanded,y_measured\n";
    for (const Row& r : rows)
        f << r.t << "," << r.u_commanded << "," << r.y_measured << "\n";
}

// Provided -- runs decodeLog() + writeCsv() for each of the three captures.
int main() {
    const char* names[] = {"step_test", "reversal_test", "deadband_test"};
    for (const char* n : names) {
        const std::string in  = std::string("data/") + n + ".log";
        const std::string out = std::string("data/") + n + ".csv";
        const std::vector<Row> rows = decodeLog(in);
        writeCsv(out, rows);
        std::printf("%-14s %6zu frames -> %s\n", n, rows.size(), out.c_str());
    }
    return 0;
}
