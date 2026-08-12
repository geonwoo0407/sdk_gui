#include "motion_pattern.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <variant>

namespace irc_step {
namespace {

// 외부 JSON 라이브러리 없이 GUI의 JSON 파일을 읽기 위한 작은 표준 JSON 파서.
struct Json {
    using Object = std::map<std::string, Json>;
    using Array = std::vector<Json>;
    std::variant<std::nullptr_t, bool, double, std::string, Object, Array> value;

    const Object& object() const { return std::get<Object>(value); }
    const Array& array() const { return std::get<Array>(value); }
    const std::string& string() const { return std::get<std::string>(value); }
    double number() const { return std::get<double>(value); }
    bool boolean() const { return std::get<bool>(value); }
};

class JsonParser {
public:
    explicit JsonParser(std::string source) : source_(std::move(source)) {}

    Json parse() {
        auto result = parseValue();
        skipSpace();
        if (position_ != source_.size()) fail("trailing data");
        return result;
    }

private:
    Json parseValue() {
        skipSpace();
        if (position_ >= source_.size()) fail("unexpected end");
        const char c = source_[position_];
        if (c == '{') return Json{parseObject()};
        if (c == '[') return Json{parseArray()};
        if (c == '"') return Json{parseString()};
        if (c == 't') { consume("true"); return Json{true}; }
        if (c == 'f') { consume("false"); return Json{false}; }
        if (c == 'n') { consume("null"); return Json{nullptr}; }
        return Json{parseNumber()};
    }

    Json::Object parseObject() {
        Json::Object result;
        expect('{'); skipSpace();
        if (take('}')) return result;
        while (true) {
            skipSpace();
            const auto key = parseString();
            skipSpace(); expect(':');
            result.emplace(key, parseValue());
            skipSpace();
            if (take('}')) break;
            expect(',');
        }
        return result;
    }

    Json::Array parseArray() {
        Json::Array result;
        expect('['); skipSpace();
        if (take(']')) return result;
        while (true) {
            result.push_back(parseValue());
            skipSpace();
            if (take(']')) break;
            expect(',');
        }
        return result;
    }

    std::string parseString() {
        expect('"');
        std::string result;
        while (position_ < source_.size()) {
            char c = source_[position_++];
            if (c == '"') return result;
            if (c != '\\') { result += c; continue; }
            if (position_ >= source_.size()) fail("bad escape");
            const char escaped = source_[position_++];
            switch (escaped) {
                case '"': result += '"'; break;
                case '\\': result += '\\'; break;
                case '/': result += '/'; break;
                case 'b': result += '\b'; break;
                case 'f': result += '\f'; break;
                case 'n': result += '\n'; break;
                case 'r': result += '\r'; break;
                case 't': result += '\t'; break;
                // GUI의 프레임 이름은 UTF-8 그대로 저장되므로 \u는 보통 사용되지 않는다.
                default: fail("unsupported JSON escape");
            }
        }
        fail("unterminated string");
    }

    double parseNumber() {
        const auto start = position_;
        if (source_[position_] == '-') ++position_;
        while (position_ < source_.size() && std::isdigit(static_cast<unsigned char>(source_[position_]))) ++position_;
        if (position_ < source_.size() && source_[position_] == '.') {
            ++position_;
            while (position_ < source_.size() && std::isdigit(static_cast<unsigned char>(source_[position_]))) ++position_;
        }
        if (position_ < source_.size() && (source_[position_] == 'e' || source_[position_] == 'E')) {
            ++position_;
            if (position_ < source_.size() && (source_[position_] == '+' || source_[position_] == '-')) ++position_;
            while (position_ < source_.size() && std::isdigit(static_cast<unsigned char>(source_[position_]))) ++position_;
        }
        try { return std::stod(source_.substr(start, position_ - start)); }
        catch (...) { fail("invalid number"); }
    }

    void skipSpace() {
        while (position_ < source_.size() && std::isspace(static_cast<unsigned char>(source_[position_]))) ++position_;
    }
    bool take(char c) {
        if (position_ < source_.size() && source_[position_] == c) { ++position_; return true; }
        return false;
    }
    void expect(char c) { if (!take(c)) fail(std::string("expected ") + c); }
    void consume(const char* word) {
        while (*word) if (position_ >= source_.size() || source_[position_++] != *word++) fail("invalid literal");
    }
    [[noreturn]] void fail(const std::string& message) const {
        throw std::runtime_error("JSON parse error at " + std::to_string(position_) + ": " + message);
    }

    std::string source_;
    std::size_t position_{0};
};

const Json& required(const Json::Object& object, std::string_view key) {
    const auto it = object.find(std::string(key));
    if (it == object.end()) throw std::runtime_error("GUI motion JSON missing field: " + std::string(key));
    return it->second;
}

double optionalNumber(const Json::Object& object, std::string_view key, double fallback) {
    const auto it = object.find(std::string(key));
    return it == object.end() ? fallback : it->second.number();
}

bool optionalBool(const Json::Object& object, std::string_view key, bool fallback) {
    const auto it = object.find(std::string(key));
    return it == object.end() ? fallback : it->second.boolean();
}

std::string optionalString(
    const Json::Object& object, std::string_view key, std::string fallback) {
    const auto it = object.find(std::string(key));
    return it == object.end() ? std::move(fallback) : it->second.string();
}

MotionPattern parseMotionObject(const Json::Object& root) {
    const auto max_seq_ms = static_cast<std::int64_t>(required(root, "max_seq_ms").number());
    const auto repeat_count = static_cast<int>(optionalNumber(root, "repeat_count", 1));
    const auto playback_speed = optionalNumber(root, "playback_speed", 1.0);
    std::vector<MotionFrame> frames;
    for (const auto& frame_json : required(root, "frames").array()) {
        const auto& object = frame_json.object();
        MotionFrame frame;
        frame.name = required(object, "name").string();
        frame.start_ms = static_cast<std::int64_t>(required(object, "start_ms").number());
        frame.time_ms = static_cast<std::int64_t>(required(object, "time_ms").number());
        for (const auto& [id, value] : required(object, "angles").object())
            frame.angles[std::stoi(id)] = value.number();
        if (const auto it = object.find("torques"); it != object.end())
            for (const auto& [id, value] : it->second.object())
                frame.torques[std::stoi(id)] = value.boolean();
        frames.push_back(std::move(frame));
    }
    MotionCompletion completion;
    if (const auto it = root.find("completion"); it != root.end()) {
        const auto& completion_json = it->second.object();
        completion.position_tolerance_deg = optionalNumber(
            completion_json, "position_tolerance_deg",
            completion.position_tolerance_deg);
        completion.settle_duration_ms = static_cast<std::int64_t>(
            optionalNumber(
                completion_json, "settle_duration_ms",
                completion.settle_duration_ms));
        completion.settle_timeout_ms = static_cast<std::int64_t>(
            optionalNumber(
                completion_json, "settle_timeout_ms",
                completion.settle_timeout_ms));
    }
    const std::string default_start =
        frames.empty() ? std::string{} : frames.front().name;
    const std::string default_end =
        frames.empty() ? std::string{} : frames.back().name;
    return MotionPattern(
        std::move(frames), max_seq_ms, repeat_count, playback_speed,
        optionalBool(root, "repeatable", true),
        optionalString(root, "start_pose", default_start),
        optionalString(root, "end_pose", default_end),
        completion);
}

}  // namespace

MotionPattern MotionPattern::loadGuiJson(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open GUI motion JSON: " + path.string());
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return parseMotionObject(JsonParser(buffer.str()).parse().object());
}

MotionPattern::MotionPattern(std::vector<MotionFrame> frames, std::int64_t max_seq_ms,
                             int repeat_count, double playback_speed,
                             bool repeatable, std::string start_pose,
                             std::string end_pose,
                             MotionCompletion completion)
    : frames_(std::move(frames)),
      max_seq_ms_(max_seq_ms),
      repeat_count_(std::max(1, repeat_count)),
      playback_speed_(std::clamp(playback_speed, 0.1, 5.0)),
      repeatable_(repeatable),
      start_pose_(std::move(start_pose)),
      end_pose_(std::move(end_pose)),
      completion_(completion) {
    validate();
    if (start_pose_.empty() && !frames_.empty()) start_pose_ = frames_.front().name;
    if (end_pose_.empty() && !frames_.empty()) end_pose_ = frames_.back().name;
    if (completion_.position_tolerance_deg <= 0.0
        || completion_.settle_duration_ms < 0
        || completion_.settle_timeout_ms < completion_.settle_duration_ms) {
        throw std::runtime_error("invalid motion completion metadata");
    }
    buildTrajectories();
}

void MotionPattern::validate() {
    std::sort(frames_.begin(), frames_.end(), [](const auto& a, const auto& b) {
        return a.start_ms < b.start_ms;
    });
    duration_ms_ = 0;
    for (std::size_t i = 0; i < frames_.size(); ++i) {
        const auto& frame = frames_[i];
        if (frame.start_ms < 0 || frame.time_ms <= 0)
            throw std::runtime_error("invalid frame time: " + frame.name);
        if (i > 0 && frames_[i - 1].start_ms + frames_[i - 1].time_ms > frame.start_ms)
            throw std::runtime_error("overlapping GUI frames: " + frames_[i - 1].name + " / " + frame.name);
        duration_ms_ = std::max(duration_ms_, frame.start_ms + frame.time_ms);
    }
    if (max_seq_ms_ < duration_ms_)
        throw std::runtime_error("max_seq_ms is shorter than motion frames");
}

void MotionPattern::setInitialAngles(JointAngles angles) {
    initial_angles_ = std::move(angles);
    buildTrajectories();
}

double MotionPattern::shortestDelta(double from_deg, double to_deg) {
    double delta = std::fmod(to_deg - from_deg + 180.0, 360.0);
    if (delta < 0.0) delta += 360.0;
    return delta - 180.0;
}

double MotionPattern::evaluateSegment(const QuinticSegment& segment,
                                      std::int64_t motion_time_ms) {
    const auto duration_ms = segment.end_ms - segment.start_ms;
    if (duration_ms <= 0) return segment.c0;
    const double u = std::clamp(
        static_cast<double>(motion_time_ms - segment.start_ms) / duration_ms,
        0.0,
        1.0);
    return (((((segment.c5 * u + segment.c4) * u + segment.c3) * u
               + segment.c2) * u + segment.c1) * u + segment.c0);
}

void MotionPattern::buildTrajectories() {
    trajectories_.clear();
    if (frames_.empty()) return;

    std::unordered_set<int> joint_ids;
    for (const auto& [id, unused] : initial_angles_) joint_ids.insert(id);
    for (const auto& frame : frames_)
        for (const auto& [id, unused] : frame.angles) joint_ids.insert(id);

    for (const int joint_id : joint_ids) {
        double initial_angle = 0.0;
        if (const auto it = initial_angles_.find(joint_id); it != initial_angles_.end()) {
            initial_angle = it->second;
        } else {
            const auto first = std::find_if(
                frames_.begin(), frames_.end(),
                [joint_id](const MotionFrame& frame) {
                    return frame.angles.contains(joint_id);
                });
            if (first == frames_.end()) continue;
            initial_angle = first->angles.at(joint_id);
        }

        std::vector<std::int64_t> times{0};
        std::vector<double> positions{initial_angle};
        double previous_angle = initial_angle;

        for (const auto& frame : frames_) {
            if (const auto it = frame.angles.find(joint_id); it != frame.angles.end()) {
                const double unwrapped = previous_angle + shortestDelta(previous_angle, it->second);
                if (unwrapped < -180.0 || unwrapped > 180.0) {
                    throw std::runtime_error(
                        "joint " + std::to_string(joint_id)
                        + " trajectory crosses the single-turn position boundary");
                }
                previous_angle = unwrapped;
            }
            times.push_back(frame.start_ms + frame.time_ms);
            positions.push_back(previous_angle);
        }

        std::vector<double> velocities(positions.size(), 0.0);
        for (std::size_t index = 1; index + 1 < positions.size(); ++index) {
            const double previous_dt = (times[index] - times[index - 1]) / 1000.0;
            const double next_dt = (times[index + 1] - times[index]) / 1000.0;
            const double previous_slope =
                (positions[index] - positions[index - 1]) / previous_dt;
            const double next_slope =
                (positions[index + 1] - positions[index]) / next_dt;

            if (previous_slope * next_slope <= 0.0) {
                velocities[index] = 0.0;
                continue;
            }

            const double w1 = 2.0 * next_dt + previous_dt;
            const double w2 = next_dt + 2.0 * previous_dt;
            double velocity = (w1 + w2)
                / (w1 / previous_slope + w2 / next_slope);
            const double limit = 3.0 * std::min(
                std::abs(previous_slope), std::abs(next_slope));
            velocity = std::copysign(std::min(std::abs(velocity), limit), velocity);
            velocities[index] = velocity;
        }

        auto& segments = trajectories_[joint_id];
        segments.reserve(frames_.size());
        for (std::size_t index = 0; index + 1 < positions.size(); ++index) {
            const double duration_sec = (times[index + 1] - times[index]) / 1000.0;
            const double delta = positions[index + 1] - positions[index];
            const double m0 = velocities[index] * duration_sec;
            const double m1 = velocities[index + 1] * duration_sec;
            segments.push_back(QuinticSegment{
                times[index],
                times[index + 1],
                positions[index],
                m0,
                0.0,
                10.0 * delta - 6.0 * m0 - 4.0 * m1,
                -15.0 * delta + 8.0 * m0 + 7.0 * m1,
                6.0 * delta - 3.0 * m0 - 3.0 * m1,
            });
        }
    }
}

MotionTarget MotionPattern::sample(std::int64_t motion_time_ms) const {
    const auto t = std::max<std::int64_t>(0, motion_time_ms);
    MotionTarget target;
    target.time_ms = t;
    target.angles = initial_angles_;
    target.finished = t >= duration_ms_;

    for (const auto& [joint_id, segments] : trajectories_) {
        if (segments.empty()) continue;
        const auto segment = std::find_if(
            segments.begin(), segments.end(),
            [t](const QuinticSegment& candidate) { return t < candidate.end_ms; });
        const auto& selected = segment == segments.end() ? segments.back() : *segment;
        target.angles[joint_id] = evaluateSegment(selected, t);
    }

    for (const auto& frame : frames_) {
        if (frame.start_ms <= t) {
            for (const auto& [id, state] : frame.torques) target.torques[id] = state;
        }
        const auto arrival_ms = frame.start_ms + frame.time_ms;
        if (target.active_frame.empty() && t < arrival_ms)
            target.active_frame = frame.name;
    }
    if (target.active_frame.empty() && !frames_.empty())
        target.active_frame = frames_.back().name;
    return target;
}

MotionLibrary MotionLibrary::loadGuiJson(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open motion library JSON: " + path.string());
    std::ostringstream buffer;
    buffer << input.rdbuf();
    const auto root = JsonParser(buffer.str()).parse().object();
    MotionLibrary library;
    for (const auto& item : required(root, "motions").array()) {
        const auto& object = item.object();
        const auto name = required(object, "name").string();
        if (!library.motions_.emplace(name, parseMotionObject(object)).second)
            throw std::runtime_error("duplicate motion name: " + name);
    }
    return library;
}

const MotionPattern& MotionLibrary::motion(const std::string& name) const {
    const auto it = motions_.find(name);
    if (it == motions_.end()) throw std::out_of_range("motion not found: " + name);
    return it->second;
}

bool MotionLibrary::contains(const std::string& name) const {
    return motions_.contains(name);
}

std::vector<std::string> MotionLibrary::names() const {
    std::vector<std::string> result;
    result.reserve(motions_.size());
    for (const auto& [name, unused] : motions_) result.push_back(name);
    std::sort(result.begin(), result.end());
    return result;
}

}  // namespace irc_step
