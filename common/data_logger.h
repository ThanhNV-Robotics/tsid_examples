// Author: Van Thanh Nguyen

#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

// CSV data logger with named record items (API modelled on OpenLoong's
// DataLogger, but writing a plain CSV file with a header row).
//
// Usage:
//   DataLogger log("record/run.csv");
//   log.addItem("time", 1);                    // column "time"
//   log.addItem("q_cmd", joint_names);         // columns "q_cmd_<joint>"
//   log.addItem("tau", 12);                    // columns "tau_0" ... "tau_11"
//   log.finishItemAdding();                    // writes the header
//   while (...) {
//     log.startNewLine();
//     log.recItemData("time", t);              // any order
//     log.recItemData("q_cmd", q_ref);
//     log.recItemData("tau", tau);
//     log.finishLine();                        // every item must be recorded
//   }
//
// Parent directories are created automatically. Misuse (duplicate or unknown
// item names, wrong data length, missing items in a line) throws
// std::runtime_error.
class DataLogger {
public:
    explicit DataLogger(const std::string& file_path, int precision = 9);
    ~DataLogger();

    DataLogger(const DataLogger&) = delete;
    DataLogger& operator=(const DataLogger&) = delete;

    // ---- Item setup (before finishItemAdding) ----
    // len == 1 -> column "<name>", otherwise "<name>_0" ... "<name>_<len-1>"
    void addItem(const std::string& name, int len);
    // One column "<name>_<col>" per entry of column_names
    void addItem(const std::string& name, const std::vector<std::string>& column_names);
    void finishItemAdding();

    // ---- Recording (one line per control step) ----
    void startNewLine();
    void recItemData(const std::string& name, double data);
    void recItemData(const std::string& name, const double* data);
    void recItemData(const std::string& name, const Eigen::Ref<const Eigen::VectorXd>& data);
    void recItemData(const std::string& name, const std::vector<double>& data);
    void finishLine();

    void flush();
    void close();

    bool isOpen() const { return file_.is_open(); }
    const std::string& path() const { return path_; }
    int numColumns() const { return static_cast<int>(columns_.size()); }
    long numLines() const { return num_lines_; }

private:
    struct Item {
        std::string name;
        int start_col;
        int len;
        bool recorded{false};
    };

    Item& findItem(const std::string& name);
    void checkLength(const Item& item, long len) const;

    std::string path_;
    std::ofstream file_;
    std::vector<Item> items_;
    std::unordered_map<std::string, size_t> item_index_;
    std::vector<std::string> columns_;
    std::vector<double> line_values_;
    bool items_finished_{false};
    long num_lines_{0};
};
