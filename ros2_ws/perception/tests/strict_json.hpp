/*
 * Copyright (c) 2026 Hachist Robotics
 * Author: diaofanghaiyang@hachist.com
 * 上海哈船智能船舶技术有限公司
 * File: strict_json.hpp
 * Brief: Strict JSON validator and inf/nan token scan for host tests of the wire text (A19-ENC-1).
 *
 * Description:
 * What it is. Two independent checks a test runs on serialized text, the two halves of
 * A19-ENC-1 (19 section 14: the output passes a strict JSON parse, and the whole text holds no
 * inf / nan literal):
 *   validate()                the RFC 8259 grammar, nothing more lenient. A number is
 *                             -?(0|[1-9][0-9]*)(.[0-9]+)?([eE][+-]?[0-9]+)?, the only bare words are
 *                             true, false and null (so inf, -inf, nan, NaN and Infinity all fail),
 *                             strings allow no raw control character and only the eight short
 *                             escapes and the four-hex-digit unicode escape, whitespace is space,
 *                             tab, LF and CR, and nothing may follow the one top-level value.
 *   first_non_finite_token()  a scan for inf / nan words OUTSIDE strings. A plain substring search
 *                             would be wrong: Status carries the key "infer_gap_ms_p99" and the
 *                             reason "infer_gap".
 * Where validate() is stricter than the grammar, on purpose (receivers differ on these, so our
 * output must not depend on them): a key repeated in one object is rejected (keys are compared
 * after unescaping); a number that does not fit a double is rejected (1e999 is legal grammar but
 * most parsers read it as infinity); string bytes must be well-formed UTF-8; a unicode escape of
 * half a surrogate pair is rejected; nesting deeper than kMaxDepth is rejected.
 * No value tree is built: the result is only pass / fail with the byte offset and a reason for
 * the test log. It allocates (key lists), so a test with an allocation guard runs it after the guard.
 * The decimal point is read with strtod; the host tests never change the C locale.
 * Used by tests/test_rns_enc_strict.cpp, which also holds the negative controls of both checks.
 * Every function is inline, so a test that uses only one of them still builds with -Werror.
 */
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace perception::rns::test_json {
// Deepest nesting accepted. The three perception bodies inside the envelope need 5.
constexpr std::size_t kMaxDepth = 64;

// Outcome of validate(): on failure, the byte where the first violation was found and why.
struct Verdict {
    bool ok = true;
    std::size_t offset = 0;
    const char* reason = "";
    explicit operator bool() const { return ok; }
};

namespace detail {
inline bool is_digit(char c) { return c >= '0' && c <= '9'; }
inline bool is_letter(char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z'); }

// Append one code point as UTF-8 (used only to compare object keys after unescaping).
inline void append_utf8(std::string& out, unsigned code) {
    if (code < 0x80) {
        out.push_back(static_cast<char>(code));
    } else if (code < 0x800) {
        out.push_back(static_cast<char>(0xc0 | (code >> 6)));
        out.push_back(static_cast<char>(0x80 | (code & 63)));
    } else if (code < 0x10000) {
        out.push_back(static_cast<char>(0xe0 | (code >> 12)));
        out.push_back(static_cast<char>(0x80 | ((code >> 6) & 63)));
        out.push_back(static_cast<char>(0x80 | (code & 63)));
    } else {
        out.push_back(static_cast<char>(0xf0 | (code >> 18)));
        out.push_back(static_cast<char>(0x80 | ((code >> 12) & 63)));
        out.push_back(static_cast<char>(0x80 | ((code >> 6) & 63)));
        out.push_back(static_cast<char>(0x80 | (code & 63)));
    }
}

// Recursive descent over the text. at_ is the next unread byte; the first failure keeps its
// offset and reason, and every caller returns false straight away.
class Validator {
public:
    explicit Validator(std::string_view text) : s_(text) {}
    Verdict run() {
        skip_space();
        if (!value(0)) return {false, at_, reason_};
        skip_space();
        if (at_ != s_.size()) return {false, at_, "bytes after the JSON value"};
        return {};
    }

private:
    // NUL stands for "end of text"; a real NUL byte in the text matches no rule and fails too.
    char peek() const { return at_ < s_.size() ? s_[at_] : '\0'; }
    bool fail(const char* why) { reason_ = why; return false; }
    void skip_space() {
        while (peek() == ' ' || peek() == '\t' || peek() == '\n' || peek() == '\r') ++at_;
    }
    bool value(std::size_t depth) {
        const char c = peek();
        if (c == '{') return object(depth);
        if (c == '[') return array(depth);
        if (c == '"') return string(nullptr);
        if (c == '-' || is_digit(c)) return number();
        return word();
    }
    // true, false and null are the only bare words JSON has.
    bool word() {
        for (const std::string_view literal : {"true", "false", "null"}) {
            if (s_.substr(at_, literal.size()) == literal) { at_ += literal.size(); return true; }
        }
        return fail(is_letter(peek()) ? "bare word other than true, false and null" : "a value is expected here");
    }
    bool number() {
        const std::size_t begin = at_;
        if (peek() == '-') ++at_;
        if (!is_digit(peek())) return fail("a digit must follow the minus sign");
        if (peek() == '0') {
            ++at_;
            if (is_digit(peek())) return fail("leading zero in a number");
        } else {
            while (is_digit(peek())) ++at_;
        }
        if (peek() == '.') {
            ++at_;
            if (!is_digit(peek())) return fail("a digit must follow the decimal point");
            while (is_digit(peek())) ++at_;
        }
        if (peek() == 'e' || peek() == 'E') {
            ++at_;
            if (peek() == '+' || peek() == '-') ++at_;
            if (!is_digit(peek())) return fail("an exponent needs a digit");
            while (is_digit(peek())) ++at_;
        }
        // Legal grammar can still overflow a double (1e999); a receiver would read infinity.
        const std::string token(s_.substr(begin, at_ - begin));
        if (!std::isfinite(std::strtod(token.c_str(), nullptr))) {
            at_ = begin;
            return fail("number does not fit a finite double");
        }
        return true;
    }
    // Four hex digits at `from` -> value; false when they are missing or not hex.
    bool hex4(std::size_t from, unsigned& value) const {
        if (from > s_.size() || s_.size() - from < 4) return false;
        value = 0;
        for (std::size_t i = 0; i < 4; ++i) {
            const char c = s_[from + i];
            unsigned digit = 0;
            if (is_digit(c)) digit = static_cast<unsigned>(c - '0');
            else if (c >= 'a' && c <= 'f') digit = static_cast<unsigned>(c - 'a') + 10;
            else if (c >= 'A' && c <= 'F') digit = static_cast<unsigned>(c - 'A') + 10;
            else return false;
            value = value * 16 + digit;
        }
        return true;
    }
    // at_ is on the backslash. `key` collects the unescaped text when the string is an object key.
    bool escape(std::string* key) {
        char plain = 0;
        switch (at_ + 1 < s_.size() ? s_[at_ + 1] : '\0') {
            case '"': plain = '"'; break;
            case '\\': plain = '\\'; break;
            case '/': plain = '/'; break;
            case 'b': plain = '\b'; break;
            case 'f': plain = '\f'; break;
            case 'n': plain = '\n'; break;
            case 'r': plain = '\r'; break;
            case 't': plain = '\t'; break;
            case 'u': return unicode_escape(key);
            default: return fail("unknown escape in a string");
        }
        if (key) key->push_back(plain);
        at_ += 2;
        return true;
    }
    // Unicode escape: backslash, u, four hex digits. A high surrogate must be followed by an
    // escaped low surrogate (one code point); either half alone is not a character and is rejected.
    bool unicode_escape(std::string* key) {
        unsigned code = 0, low = 0;
        if (!hex4(at_ + 2, code)) return fail("\\u needs four hex digits");
        if (code >= 0xdc00 && code <= 0xdfff) return fail("\\u escape of a lone low surrogate");
        if (code >= 0xd800 && code <= 0xdbff) {
            if (s_.substr(at_ + 6, 2) != "\\u" || !hex4(at_ + 8, low) || low < 0xdc00 || low > 0xdfff)
                return fail("\\u escape of a lone high surrogate");
            code = 0x10000 + ((code - 0xd800) << 10) + (low - 0xdc00);
            at_ += 6;
        }
        at_ += 6;
        if (key) append_utf8(*key, code);
        return true;
    }
    // One multi-byte UTF-8 sequence at at_: no overlong form, no surrogate, at most U+10FFFF
    // (the same rule the encoder applies to its input strings).
    bool utf8(std::string* key) {
        const auto lead = static_cast<unsigned char>(s_[at_]);
        unsigned need = 0, code = 0, minimum = 0;
        if (lead >= 0xc2 && lead <= 0xdf) { need = 1; code = lead & 31; minimum = 0x80; }
        else if (lead >= 0xe0 && lead <= 0xef) { need = 2; code = lead & 15; minimum = 0x800; }
        else if (lead >= 0xf0 && lead <= 0xf4) { need = 3; code = lead & 7; minimum = 0x10000; }
        else return fail("byte that cannot start a UTF-8 sequence");
        if (s_.size() - at_ - 1 < need) return fail("truncated UTF-8 sequence");
        for (unsigned i = 1; i <= need; ++i) {
            const auto next = static_cast<unsigned char>(s_[at_ + i]);
            if ((next & 0xc0) != 0x80) return fail("bad UTF-8 continuation byte");
            code = (code << 6) | (next & 63);
        }
        if (code < minimum || code > 0x10ffff || (code >= 0xd800 && code <= 0xdfff))
            return fail("overlong, surrogate or out-of-range UTF-8 sequence");
        if (key) key->append(s_.substr(at_, need + 1));
        at_ += need + 1;
        return true;
    }
    // at_ is on the opening quote.
    bool string(std::string* key) {
        ++at_;
        for (;;) {
            if (at_ >= s_.size()) return fail("string is not closed");
            const auto c = static_cast<unsigned char>(s_[at_]);
            if (c == '"') { ++at_; return true; }
            if (c < 0x20) return fail("raw control character in a string");
            if (c == '\\') {
                if (!escape(key)) return false;
            } else if (c < 0x80) {
                if (key) key->push_back(static_cast<char>(c));
                ++at_;
            } else if (!utf8(key)) {
                return false;
            }
        }
    }
    bool object(std::size_t depth) {
        if (depth >= kMaxDepth) return fail("nesting is too deep");
        ++at_;
        skip_space();
        if (peek() == '}') { ++at_; return true; }
        // Keys of THIS object only; the same name in another object is not a repeat.
        std::vector<std::string> keys;
        for (;;) {
            if (peek() != '"') return fail("an object key must be a string");
            const std::size_t key_at = at_;
            std::string key;
            if (!string(&key)) return false;
            for (const auto& seen : keys) {
                if (seen == key) { at_ = key_at; return fail("key repeated in one object"); }
            }
            keys.push_back(std::move(key));
            skip_space();
            if (peek() != ':') return fail("':' expected after an object key");
            ++at_;
            skip_space();
            if (!value(depth + 1)) return false;
            skip_space();
            if (peek() == '}') { ++at_; return true; }
            if (peek() != ',') return fail("',' or '}' expected");
            ++at_;
            skip_space();
        }
    }
    bool array(std::size_t depth) {
        if (depth >= kMaxDepth) return fail("nesting is too deep");
        ++at_;
        skip_space();
        if (peek() == ']') { ++at_; return true; }
        for (;;) {
            if (!value(depth + 1)) return false;
            skip_space();
            if (peek() == ']') { ++at_; return true; }
            if (peek() != ',') return fail("',' or ']' expected");
            ++at_;
            skip_space();
        }
    }

    std::string_view s_;
    std::size_t at_ = 0;
    const char* reason_ = "";
};
}  // namespace detail

// Strict parse of one complete JSON text (see the file description for the exact rules).
inline Verdict validate(std::string_view text) { return detail::Validator(text).run(); }

// Offset of the first inf / nan word outside a JSON string, or npos when there is none. A word is
// a run of ASCII letters; it counts when it contains "inf" or "nan" in any letter case, which
// covers inf, -inf, Infinity, nan, NaN, nan(ind) and 1.#INF / 1.#QNAN. Strings are skipped with
// their escapes, so keys and reasons such as "infer_gap" never count.
inline std::size_t first_non_finite_token(std::string_view text) {
    for (std::size_t i = 0; i < text.size();) {
        if (text[i] == '"') {
            for (++i; i < text.size() && text[i] != '"'; ++i) {
                if (text[i] == '\\') ++i;
            }
            ++i;
        } else if (detail::is_letter(text[i])) {
            const std::size_t begin = i;
            std::string word;
            for (; i < text.size() && detail::is_letter(text[i]); ++i) {
                word.push_back(static_cast<char>(text[i] | 0x20));  // ASCII lower case
            }
            if (word.find("inf") != std::string::npos || word.find("nan") != std::string::npos) return begin;
        } else {
            ++i;
        }
    }
    return std::string_view::npos;
}
}  // namespace perception::rns::test_json
