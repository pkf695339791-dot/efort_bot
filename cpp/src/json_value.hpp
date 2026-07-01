#pragma once

#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace tie::json {

class Value {
public:
    using Object = std::map<std::string, Value>;
    using Array = std::vector<Value>;
    using Data = std::variant<std::nullptr_t, bool, double, std::string, Array, Object>;

    Value() : data_(nullptr) {}
    explicit Value(Data data) : data_(std::move(data)) {}

    bool is_object() const { return std::holds_alternative<Object>(data_); }
    bool is_array() const { return std::holds_alternative<Array>(data_); }
    const Object& object() const { return std::get<Object>(data_); }
    const Array& array() const { return std::get<Array>(data_); }

    const Value* find(const std::string& key) const {
        if (!is_object()) return nullptr;
        const auto it = object().find(key);
        return it == object().end() ? nullptr : &it->second;
    }

    std::string string_or(const std::string& fallback) const {
        return std::holds_alternative<std::string>(data_)
            ? std::get<std::string>(data_) : fallback;
    }
    double number_or(double fallback) const {
        return std::holds_alternative<double>(data_) ? std::get<double>(data_) : fallback;
    }
    bool bool_or(bool fallback) const {
        return std::holds_alternative<bool>(data_) ? std::get<bool>(data_) : fallback;
    }

private:
    Data data_;
};

class Parser {
public:
    explicit Parser(std::string text) : text_(std::move(text)) {}

    Value parse() {
        Value result = parse_value();
        whitespace();
        if (position_ != text_.size()) fail("unexpected trailing data");
        return result;
    }

private:
    std::string text_;
    std::size_t position_{};

    [[noreturn]] void fail(const std::string& message) const {
        throw std::runtime_error(
            "JSON parse error at byte " + std::to_string(position_) + ": " + message);
    }
    void whitespace() {
        while (position_ < text_.size() &&
               std::isspace(static_cast<unsigned char>(text_[position_]))) {
            ++position_;
        }
    }
    bool consume(char token) {
        whitespace();
        if (position_ < text_.size() && text_[position_] == token) {
            ++position_;
            return true;
        }
        return false;
    }
    void expect(char token) {
        if (!consume(token)) fail(std::string("expected '") + token + "'");
    }
    Value parse_value() {
        whitespace();
        if (position_ >= text_.size()) fail("expected value");
        const char token = text_[position_];
        if (token == '{') return parse_object();
        if (token == '[') return parse_array();
        if (token == '"') return Value(parse_string());
        if (token == 't') return parse_literal("true", Value(true));
        if (token == 'f') return parse_literal("false", Value(false));
        if (token == 'n') return parse_literal("null", Value());
        return Value(parse_number());
    }
    Value parse_literal(const std::string& literal, Value result) {
        if (text_.compare(position_, literal.size(), literal) != 0) {
            fail("invalid literal");
        }
        position_ += literal.size();
        return result;
    }
    Value parse_object() {
        expect('{');
        Value::Object result;
        if (consume('}')) return Value(result);
        do {
            whitespace();
            if (position_ >= text_.size() || text_[position_] != '"') {
                fail("expected object key");
            }
            const std::string key = parse_string();
            expect(':');
            result.emplace(key, parse_value());
        } while (consume(','));
        expect('}');
        return Value(result);
    }
    Value parse_array() {
        expect('[');
        Value::Array result;
        if (consume(']')) return Value(result);
        do {
            result.push_back(parse_value());
        } while (consume(','));
        expect(']');
        return Value(result);
    }
    std::string parse_string() {
        expect('"');
        std::string result;
        while (position_ < text_.size()) {
            char value = text_[position_++];
            if (value == '"') return result;
            if (value != '\\') {
                result.push_back(value);
                continue;
            }
            if (position_ >= text_.size()) fail("unterminated escape");
            const char escaped = text_[position_++];
            switch (escaped) {
                case '"': result.push_back('"'); break;
                case '\\': result.push_back('\\'); break;
                case '/': result.push_back('/'); break;
                case 'b': result.push_back('\b'); break;
                case 'f': result.push_back('\f'); break;
                case 'n': result.push_back('\n'); break;
                case 'r': result.push_back('\r'); break;
                case 't': result.push_back('\t'); break;
                default: fail("unsupported string escape");
            }
        }
        fail("unterminated string");
    }
    double parse_number() {
        whitespace();
        const std::size_t start = position_;
        if (position_ < text_.size() && text_[position_] == '-') ++position_;
        while (position_ < text_.size() &&
               std::isdigit(static_cast<unsigned char>(text_[position_]))) ++position_;
        if (position_ < text_.size() && text_[position_] == '.') {
            ++position_;
            while (position_ < text_.size() &&
                   std::isdigit(static_cast<unsigned char>(text_[position_]))) ++position_;
        }
        if (position_ < text_.size() &&
            (text_[position_] == 'e' || text_[position_] == 'E')) {
            ++position_;
            if (position_ < text_.size() &&
                (text_[position_] == '+' || text_[position_] == '-')) ++position_;
            while (position_ < text_.size() &&
                   std::isdigit(static_cast<unsigned char>(text_[position_]))) ++position_;
        }
        if (start == position_) fail("invalid number");
        return std::stod(text_.substr(start, position_ - start));
    }
};

inline Value read_file(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open JSON file: " + path);
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return Parser(buffer.str()).parse();
}

inline const Value* find(const Value& value, const std::string& key) {
    return value.find(key);
}
inline double number(const Value& value, const std::string& key, double fallback) {
    const Value* item = find(value, key);
    return item ? item->number_or(fallback) : fallback;
}
inline std::string string(
    const Value& value, const std::string& key, const std::string& fallback) {
    const Value* item = find(value, key);
    return item ? item->string_or(fallback) : fallback;
}
inline bool boolean(const Value& value, const std::string& key, bool fallback) {
    const Value* item = find(value, key);
    return item ? item->bool_or(fallback) : fallback;
}

}  // namespace tie::json

