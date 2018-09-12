#pragma once

#include <functional>
#include <memory>
#include <vector>
#include "plugin_base.h"

namespace dronecode_sdk {

class LogFilesImpl;
class System;

/**
 * @brief The LogFiles class allows to download log files from the vehicle
 * after a flight is completed.
 */
class LogFiles : public PluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto log_files = std::make_shared<LogFiles>(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit LogFiles(System &system);

    /**
     * @brief Destructor (internal use only).
     */
    ~LogFiles();

    /**
     * @brief Results for log file requests.
     */
    enum class Result {
        SUCCESS = 0, /**< @brief Request succeeded. */
        NO_LOGFILES, /**< @brief No logfiles found. */
        TOO_MANY_RETRIES, /**< @brief Too many retries. */
        UNKNOWN /**< @brief Unknown error. */
    };

    /**
     * @brief Returns human-readable English string for LogFiles::Result.
     *
     * @param result Enum for which string is required.
     * @return result Human-readable string for LogFiles::Result.
     */
    static const char *result_str(Result result);

    struct Entry {
        unsigned id;
        std::string date;
        unsigned bytes;
    };

    /**
     * @brief Get list of log files (synchronous).
     *
     * @return Pair of result and list of entries.
     */
    std::pair<Result, std::vector<Entry>> get_entries();

    /**
     * @brief Callback type for logging requests.
     */
    typedef std::function<void(Result, std::vector<Entry>)> get_entries_callback_t;

    /**
     * @brief Get list of log files (asynchronous).
     *
     * @param callback Callback to get result from.
     */
    void get_entries_async(LogFiles::get_entries_callback_t callback);

    /**
     * @brief Copy constructor (object is not copyable).
     */
    LogFiles(const LogFiles &) = delete;
    /**
     * @brief Equality operator (object is not copyable).
     */
    const LogFiles &operator=(const LogFiles &) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<LogFilesImpl> _impl;
};

} // namespace dronecode_sdk
