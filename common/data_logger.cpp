// Author: Van Thanh Nguyen

#include "data_logger.h"

#include <filesystem>
#include <iostream>
#include <stdexcept>

DataLogger::DataLogger(const std::string& file_path, int precision) : path_(file_path) {
    const std::filesystem::path dir = std::filesystem::path(file_path).parent_path();
    if (!dir.empty()) std::filesystem::create_directories(dir);

    file_.open(file_path);
    if (!file_.is_open()) {
        throw std::runtime_error("[DataLogger] Failed to open '" + file_path + "'");
    }
    file_.precision(precision);
}

DataLogger::~DataLogger() { close(); }

void DataLogger::addItem(const std::string& name, int len) {
    if (len <= 0) {
        throw std::runtime_error("[DataLogger] Item '" + name + "' must have a positive length");
    }
    std::vector<std::string> column_names;
    if (len == 1) {
        column_names.push_back("");
    } else {
        for (int i = 0; i < len; ++i) column_names.push_back(std::to_string(i));
    }
    addItem(name, column_names);
}

void DataLogger::addItem(const std::string& name, const std::vector<std::string>& column_names) {
    if (items_finished_) {
        throw std::runtime_error("[DataLogger] Cannot add item '" + name + "' after finishItemAdding()");
    }
    if (item_index_.count(name)) {
        throw std::runtime_error("[DataLogger] Item '" + name + "' has already been added");
    }
    if (column_names.empty()) {
        throw std::runtime_error("[DataLogger] Item '" + name + "' has no columns");
    }

    item_index_[name] = items_.size();
    items_.push_back({name, numColumns(), static_cast<int>(column_names.size())});
    for (const auto& col : column_names) {
        columns_.push_back(col.empty() ? name : name + "_" + col);
    }
}

void DataLogger::finishItemAdding() {
    if (items_finished_) return;
    for (size_t i = 0; i < columns_.size(); ++i) {
        if (i > 0) file_ << ',';
        file_ << columns_[i];
    }
    file_ << '\n';
    line_values_.assign(columns_.size(), 0.0);
    items_finished_ = true;
}

void DataLogger::startNewLine() {
    if (!items_finished_) finishItemAdding();
    std::fill(line_values_.begin(), line_values_.end(), 0.0);
    for (auto& item : items_) item.recorded = false;
}

DataLogger::Item& DataLogger::findItem(const std::string& name) {
    if (!items_finished_) {
        throw std::runtime_error("[DataLogger] Call finishItemAdding() before recording '" + name + "'");
    }
    const auto it = item_index_.find(name);
    if (it == item_index_.end()) {
        throw std::runtime_error("[DataLogger] Item '" + name + "' has not been added");
    }
    return items_[it->second];
}

void DataLogger::checkLength(const Item& item, long len) const {
    if (len != item.len) {
        throw std::runtime_error("[DataLogger] Item '" + item.name + "' expects " + std::to_string(item.len) +
                                 " values but got " + std::to_string(len));
    }
}

void DataLogger::recItemData(const std::string& name, double data) {
    Item& item = findItem(name);
    std::fill_n(line_values_.begin() + item.start_col, item.len, data);
    item.recorded = true;
}

void DataLogger::recItemData(const std::string& name, const double* data) {
    Item& item = findItem(name);
    std::copy_n(data, item.len, line_values_.begin() + item.start_col);
    item.recorded = true;
}

void DataLogger::recItemData(const std::string& name, const Eigen::Ref<const Eigen::VectorXd>& data) {
    Item& item = findItem(name);
    checkLength(item, data.size());
    for (int i = 0; i < item.len; ++i) line_values_[item.start_col + i] = data(i);
    item.recorded = true;
}

void DataLogger::recItemData(const std::string& name, const std::vector<double>& data) {
    Item& item = findItem(name);
    checkLength(item, static_cast<long>(data.size()));
    std::copy(data.begin(), data.end(), line_values_.begin() + item.start_col);
    item.recorded = true;
}

void DataLogger::finishLine() {
    for (const auto& item : items_) {
        if (!item.recorded) {
            throw std::runtime_error("[DataLogger] Item '" + item.name + "' has not been recorded in line " +
                                     std::to_string(num_lines_));
        }
    }
    for (size_t i = 0; i < line_values_.size(); ++i) {
        if (i > 0) file_ << ',';
        file_ << line_values_[i];
    }
    file_ << '\n';
    ++num_lines_;
    for (auto& item : items_) item.recorded = false;
}

void DataLogger::flush() {
    if (file_.is_open()) file_.flush();
}

void DataLogger::close() {
    if (file_.is_open()) file_.close();
}
