#pragma once

#include <string>
#include <utility>

namespace studentlib {

struct Result {
    bool success{true};
    std::string message{};

    static Result ok() {
        return Result{};
    }

    static Result error(std::string message_text) {
        return Result{false, std::move(message_text)};
    }

    explicit operator bool() const {
        return success;
    }
};

}  // namespace studentlib