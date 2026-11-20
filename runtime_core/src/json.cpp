#include "legged_nav/json.hpp"

#include <cstring>

namespace legged_nav {
namespace {

struct Parser {
    const std::string& s;
    size_t i = 0;

    void ws() {
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    }
    bool lit(const char* word) {
        size_t n = std::strlen(word);
        if (s.compare(i, n, word) == 0) {
            i += n;
            return true;
        }
        return false;
    }
    std::optional<std::string> string() {
        if (i >= s.size() || s[i] != '"') return std::nullopt;
        ++i;
        std::string out;
        while (i < s.size() && s[i] != '"') {
            if (s[i] == '\\') {
                if (++i >= s.size()) return std::nullopt;
                switch (s[i]) {
                    case 'n':
                        out += '\n';
                        break;
                    case 't':
                        out += '\t';
                        break;
                    case 'r':
                        out += '\r';
                        break;
                    case 'b':
                        out += '\b';
                        break;
                    case 'f':
                        out += '\f';
                        break;
                    case 'u':
                        if (i + 4 >= s.size()) return std::nullopt;
                        out += '?';  // 不展开 unicode；契约字段不含 unicode 语义
                        i += 4;
                        break;
                    default:
                        out += s[i];
                }
            } else {
                out += s[i];
            }
            ++i;
        }
        if (i >= s.size()) return std::nullopt;
        ++i;
        return out;
    }
    std::optional<JsonValue> value() {
        ws();
        if (i >= s.size()) return std::nullopt;
        char c = s[i];
        if (c == '{') {
            ++i;
            JsonValue::Object o;
            ws();
            if (i < s.size() && s[i] == '}') {
                ++i;
                return JsonValue(std::move(o));
            }
            while (true) {
                ws();
                auto k = string();
                if (!k) return std::nullopt;
                ws();
                if (i >= s.size() || s[i] != ':') return std::nullopt;
                ++i;
                auto val = value();
                if (!val) return std::nullopt;
                o[std::move(*k)] = std::move(*val);
                ws();
                if (i >= s.size()) return std::nullopt;
                if (s[i] == '}') {
                    ++i;
                    return JsonValue(std::move(o));
                }
                if (s[i] != ',') return std::nullopt;
                ++i;
            }
        }
        if (c == '[') {
            ++i;
            JsonValue::Array a;
            ws();
            if (i < s.size() && s[i] == ']') {
                ++i;
                return JsonValue(std::move(a));
            }
            while (true) {
                auto val = value();
                if (!val) return std::nullopt;
                a.push_back(std::move(*val));
                ws();
                if (i >= s.size()) return std::nullopt;
                if (s[i] == ']') {
                    ++i;
                    return JsonValue(std::move(a));
                }
                if (s[i] != ',') return std::nullopt;
                ++i;
            }
        }
        if (c == '"') {
            auto st = string();
            if (!st) return std::nullopt;
            return JsonValue(std::move(*st));
        }
        if (lit("true")) return JsonValue(true);
        if (lit("false")) return JsonValue(false);
        if (lit("null")) return JsonValue();
        // number
        size_t start = i;
        if (c == '-') ++i;
        while (i < s.size() && (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '.' ||
                                s[i] == 'e' || s[i] == 'E' || s[i] == '+' || s[i] == '-'))
            ++i;
        if (i == start) return std::nullopt;
        try {
            return JsonValue(std::stod(s.substr(start, i - start)));
        } catch (...) {
            return std::nullopt;
        }
    }
};

}  // namespace

std::optional<JsonValue> parse_json(const std::string& text) {
    Parser p{text};
    auto v = p.value();
    p.ws();
    if (!v || p.i != text.size()) return std::nullopt;
    return v;
}

}  // namespace legged_nav
