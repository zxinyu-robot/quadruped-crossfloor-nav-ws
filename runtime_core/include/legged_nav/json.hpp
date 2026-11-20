#pragma once
/// @file json.hpp
/// @brief 最小 JSON 值与解析器（契约层输入）；无第三方依赖。
/// 仅支持标准 JSON：object/array/string/number/bool/null。
/// 线程安全：纯值类型；解析失败返回 std::nullopt。

#include <cctype>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace legged_nav {

struct JsonValue {
    using Object = std::map<std::string, JsonValue>;
    using Array = std::vector<JsonValue>;

    std::variant<std::monostate, bool, double, std::string, Array, Object> v;

    JsonValue() = default;
    JsonValue(bool b) : v(b) {}
    JsonValue(double d) : v(d) {}
    JsonValue(int i) : v(static_cast<double>(i)) {}
    JsonValue(const char* s) : v(std::string(s)) {}
    JsonValue(std::string s) : v(std::move(s)) {}
    JsonValue(Array a) : v(std::move(a)) {}
    JsonValue(Object o) : v(std::move(o)) {}

    bool is_null() const { return std::holds_alternative<std::monostate>(v); }
    bool is_bool() const { return std::holds_alternative<bool>(v); }
    bool is_num() const { return std::holds_alternative<double>(v); }
    bool is_str() const { return std::holds_alternative<std::string>(v); }
    bool is_arr() const { return std::holds_alternative<Array>(v); }
    bool is_obj() const { return std::holds_alternative<Object>(v); }

    const std::string& str() const { return std::get<std::string>(v); }
    double num() const { return std::get<double>(v); }
    bool boolean() const { return std::get<bool>(v); }
    const Array& arr() const { return std::get<Array>(v); }
    const Object& obj() const { return std::get<Object>(v); }

    /// 取对象字段；不存在或类型不匹配返回 nullptr。
    const JsonValue* get(const std::string& key) const {
        if (!is_obj()) return nullptr;
        const auto& o = obj();
        auto it = o.find(key);
        return it == o.end() ? nullptr : &it->second;
    }
    std::string str_or(const std::string& dflt = "") const { return is_str() ? str() : dflt; }
    double num_or(double dflt = 0.0) const { return is_num() ? num() : dflt; }
};

/// 解析 JSON；失败返回 std::nullopt。
std::optional<JsonValue> parse_json(const std::string& text);

}  // namespace legged_nav
