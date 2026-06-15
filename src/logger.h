#pragma once

#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

namespace autorudder {

class Logger {
public:
    explicit Logger(const std::filesystem::path& path);

    void info(const std::string& message);
    void warn(const std::string& message);
    void error(const std::string& message);

private:
    void write(const char* level, const std::string& message);

    std::mutex mutex_;
    std::ofstream file_;
};

}  // namespace autorudder
